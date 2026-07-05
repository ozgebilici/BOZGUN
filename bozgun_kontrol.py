import cv2
import serial
import time
import numpy as np
import pygame
import json
import math
import os

# ==============================================================================
# --- AYARLAR BÖLÜMÜ (DAHA GELİŞMİŞ) ---
# ==============================================================================
ARDUINO_PORT = "COM5"
BAUDRATE = 9600
CAMERA_INDEX = 0

# Varsayılan HSV (başlangıç) - kalibrasyonla güncellenecek
DEFAULT_HSV = {
    "low_h": 94, "low_s": 80, "low_v": 2,
    "high_h": 126, "high_s": 255, "high_v": 255
}
CALIB_FILE = "hsv_calib.json"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480

PAN_MIN_ANGLE = 45
PAN_MAX_ANGLE = 135
TILT_MIN_ANGLE = 60
TILT_MAX_ANGLE = 120

KILITLENME_SURESI = 1.5          # hedef kilitlenme süresi (s)
VURULAN_HEDEF_BEKLEME = 5.0     # vurulmuş hedefin unutulma süresi (s)
MIN_HEDEF_YARICAPI = 30         # minimum hedef boyutu (px)
LOOP_FPS_SMOOTH = 0.9           # FPS göstergesi için smoothing (0..1)

# Motor smoothing ayarı: artık sabit bir "katsayı" değil, bir zaman sabiti (tau)
# MOTOR_RESPONSE_TC: daha küçük => daha hızlı tepki; daha büyük => daha yumuşak
MOTOR_RESPONSE_TC = 0.12  # saniye cinsinden (tune edin; örn. 0.05..0.3)

# Takip parametreleri
TRACK_MAX_MISSES = 0.6   # saniye içinde görülmeyen bir track silinsin
TRACK_MATCH_DIST = 60.0  # px, centroid eşleştirme eşiği

# Pencere isimleri
WINDOW_NAME = "BOZGUN v5 - KONTROL"
CALIB_WINDOW = "HSV Kalibrasyon"

# ==============================================================================
# --- YARDIMCI FONKSİYONLAR: KALİBRASYON (TRACKBAR) VE DOSYA İŞLEMLERİ) ---
# ==============================================================================
def load_calibration(path=CALIB_FILE):
    if os.path.exists(path):
        try:
            with open(path, "r") as f:
                data = json.load(f)
            print(f"[Kalibrasyon] {path} yüklendi.")
            return data
        except Exception as e:
            print(f"[Kalibrasyon] {path} yüklenemedi: {e}")
    return DEFAULT_HSV.copy()

def save_calibration(cfg, path=CALIB_FILE):
    try:
        with open(path, "w") as f:
            json.dump(cfg, f, indent=2)
        print(f"[Kalibrasyon] Ayarlar kaydedildi -> {path}")
    except Exception as e:
        print(f"[Kalibrasyon] Kaydetme hatası: {e}")

def create_calibration_window(initial_cfg):
    cv2.namedWindow(CALIB_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CALIB_WINDOW, 400, 320)
    # yarat trackbar
    cv2.createTrackbar("LowH", CALIB_WINDOW, initial_cfg["low_h"], 179, lambda v: None)
    cv2.createTrackbar("HighH", CALIB_WINDOW, initial_cfg["high_h"], 179, lambda v: None)
    cv2.createTrackbar("LowS", CALIB_WINDOW, initial_cfg["low_s"], 255, lambda v: None)
    cv2.createTrackbar("HighS", CALIB_WINDOW, initial_cfg["high_s"], 255, lambda v: None)
    cv2.createTrackbar("LowV", CALIB_WINDOW, initial_cfg["low_v"], 255, lambda v: None)
    cv2.createTrackbar("HighV", CALIB_WINDOW, initial_cfg["high_v"], 255, lambda v: None)

def read_calibration_from_window():
    cfg = {}
    cfg["low_h"] = cv2.getTrackbarPos("LowH", CALIB_WINDOW)
    cfg["high_h"] = cv2.getTrackbarPos("HighH", CALIB_WINDOW)
    cfg["low_s"] = cv2.getTrackbarPos("LowS", CALIB_WINDOW)
    cfg["high_s"] = cv2.getTrackbarPos("HighS", CALIB_WINDOW)
    cfg["low_v"] = cv2.getTrackbarPos("LowV", CALIB_WINDOW)
    cfg["high_v"] = cv2.getTrackbarPos("HighV", CALIB_WINDOW)
    return cfg

# ==============================================================================
# --- HAFIZA TABANLI BASİT CENTROID-TRACKER ---
# ==============================================================================
class Track:
    def __init__(self, track_id, centroid, timestamp):
        self.id = track_id
        self.centroid = centroid  # (x, y)
        self.last_seen = timestamp
        self.first_seen = timestamp
        self.missed_time = 0.0

    def update(self, centroid, timestamp):
        self.centroid = centroid
        self.last_seen = timestamp
        self.missed_time = 0.0

class SimpleTracker:
    def __init__(self, max_miss_seconds=TRACK_MAX_MISSES, match_dist=TRACK_MATCH_DIST):
        self.tracks = {}   # id -> Track
        self._next_id = 1
        self.max_miss = max_miss_seconds
        self.match_dist = match_dist

    def step(self, detections, timestamp):
        """
        detections: list of (x, y) centroids
        timestamp: current time
        returns: list of tuples (track_id, centroid)
        """
        assignments = {}
        used_det = set()
        # 1) Match existing tracks to detections by nearest neighbor (within match_dist)
        for tid, tr in list(self.tracks.items()):
            best_det = None
            best_dist = None
            for i, d in enumerate(detections):
                if i in used_det: 
                    continue
                dist = math.hypot(tr.centroid[0] - d[0], tr.centroid[1] - d[1])
                if best_det is None or dist < best_dist:
                    best_det = i
                    best_dist = dist
            if best_det is not None and best_dist <= self.match_dist:
                # update track
                tr.update(detections[best_det], timestamp)
                assignments[tr.id] = tr.centroid
                used_det.add(best_det)
            else:
                # not matched this frame -> will increase missed duration
                # we don't update missed_time here; cleanup will remove by time
                pass

        # 2) Create tracks for unmatched detections
        for i, d in enumerate(detections):
            if i in used_det: continue
            tid = self._next_id
            self._next_id += 1
            newt = Track(tid, d, timestamp)
            self.tracks[tid] = newt
            assignments[tid] = d

        # 3) Remove old tracks
        for tid, tr in list(self.tracks.items()):
            if timestamp - tr.last_seen > self.max_miss:
                del self.tracks[tid]

        # Return current assignment list for visualization/logic
        return [(tid, tuple(tr.centroid)) for tid, tr in self.tracks.items()]

    def get_track_by_id(self, tid):
        return self.tracks.get(tid, None)

# ==============================================================================
# --- HUD çizme (geliştirilmiş) ---
# ==============================================================================
def draw_hud(frame, status_text, target_info_list=None, fps=None):
    h, w, _ = frame.shape
    color = (0, 255, 0)  # yeşil
    thickness = 2
    # köşe çizgileri
    L = 30
    cv2.line(frame, (0, 0), (L, 0), color, thickness)
    cv2.line(frame, (0, 0), (0, L), color, thickness)
    cv2.line(frame, (w, 0), (w - L, 0), color, thickness)
    cv2.line(frame, (w, 0), (w, L), color, thickness)
    cv2.line(frame, (0, h), (L, h), color, thickness)
    cv2.line(frame, (0, h), (0, h - L), color, thickness)
    cv2.line(frame, (w, h), (w - L, h), color, thickness)
    cv2.line(frame, (w, h), (w, h - L), color, thickness)

    cv2.putText(frame, f"DURUM: {status_text}", (16, h - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    if fps is not None:
        cv2.putText(frame, f"FPS: {fps:.1f}", (w - 140, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    if target_info_list:
        x0 = w - 300
        y0 = h - 90
        for i, (tid, (x, y, r)) in enumerate(target_info_list):
            cv2.putText(frame, f"ID:{tid} [{int(x)},{int(y)}] R:{int(r)}", (x0, y0 + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

# ==============================================================================
# --- ANA PROGRAM ---
# ==============================================================================
def main():
    # 1) Kalibrasyonu yükle veya başlat
    cfg = load_calibration()
    create_calibration_window(cfg)

    # 2) Ses yükleme
    pygame.mixer.init()
    try:
        shot_sound = pygame.mixer.Sound("laser_shot.mp3")
    except Exception:
        shot_sound = None
        print("!!! UYARI: 'laser_shot.mp3' bulunamadı veya pygame desteklenmiyor.")

    # 3) Arduino bağlantısı (hata halinde simülasyon)
    arduino = None
    arduino_connected = False
    try:
        arduino = serial.Serial(port=ARDUINO_PORT, baudrate=BAUDRATE, timeout=0.1)
        time.sleep(2)
        arduino_connected = True
        print("[Arduino] Bağlandı.")
    except Exception as e:
        print(f"[Arduino] Bağlanamadı ({e}). Simülasyon modunda çalışılacak (seri yok).")

    # 4) Kamera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Kamera açılamadı. Program sonlandırılıyor.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    cv2.namedWindow(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    tracker = SimpleTracker()
    vurulan_hedefler = {}   # track_id -> last_vurulma_zamani
    kilitlenen_hedef_id = None
    kilitlenme_baslangic_zamani = 0.0

    current_pan_angle = 90.0
    current_tilt_angle = 90.0

    prev_time = time.time()
    fps = 0.0
    smoothed_fps = None

    status_text = "Başlatıldı - Kalibrasyon yapın veya bekleyin..."

    print("Sistem başlatıldı. Kalibrasyon penceresinden renk aralığını ayarlayın. (S: kaydet, Q: çıkış)")

    while True:
        loop_start = time.time()
        dt = loop_start - prev_time if prev_time is not None else 0.03
        prev_time = loop_start

        # Kalibrasyon parametrelerini oku (trackbar)
        calib = read_calibration_from_window()
        lower = np.array([calib["low_h"], calib["low_s"], calib["low_v"]])
        upper = np.array([calib["high_h"], calib["high_s"], calib["high_v"]])

        ret, frame = cap.read()
        if not ret:
            print("Kare alınamadı, çıkılıyor.")
            break

            
        # Görüntü işlemleri
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Konturlar
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []   # centroid list
        detections_info = []  # (x,y,r) for drawing & mapping
        for c in contours:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if radius >= MIN_HEDEF_YARICAPI:
                detections.append((x, y))
                detections_info.append((x, y, radius))

        # Tracker güncelle
        tracked = tracker.step(detections, loop_start)  # returns list of (tid, centroid)
        # Create mapping centroid->r by nearest matching (for annotation & for target selection)
        det_map = {}
        for x, y, r in detections_info:
            # find nearest tracked id
            best_id = None
            best_dist = None
            for tid, ctr in tracked:
                dist = math.hypot(ctr[0]-x, ctr[1]-y)
                if best_id is None or dist < best_dist:
                    best_id = tid
                    best_dist = dist
            if best_id is not None and best_dist <= TRACK_MATCH_DIST:
                det_map[best_id] = (x, y, r)
            else:
                # If no track matched, we can try assign later by detection order; ignore for id
                pass

        # hedef seçimi: (öncelik: en büyük r, ama track ID'ye göre vurulmuşları kontrol et)
        chosen = None
        if len(detections_info) > 0:
            # choose largest detection that is not in vurulan_hedefler recently
            candidates = sorted(detections_info, key=lambda t: t[2], reverse=True)
            for x, y, r in candidates:
                # find corresponding track id if exists
                matched_id = None
                for tid, ctr in tracked:
                    dist = math.hypot(ctr[0] - x, ctr[1] - y)
                    if dist <= TRACK_MATCH_DIST:
                        matched_id = tid
                        break
                # skip if recently vurulmuş
                if matched_id is not None and matched_id in vurulan_hedefler:
                    if loop_start - vurulan_hedefler[matched_id] < VURULAN_HEDEF_BEKLEME:
                        continue
                # accept
                chosen = (matched_id, x, y, r)
                break

        lazer_state = 0
        target_pan_angle = 90
        target_tilt_angle = 90
        target_info_list = []

        if chosen:
            matched_id, x, y, radius = chosen
            # servo açısı hesapla
            target_pan_angle = np.interp(x, [0, FRAME_WIDTH], [PAN_MAX_ANGLE, PAN_MIN_ANGLE])
            target_tilt_angle = np.interp(y, [0, FRAME_HEIGHT], [TILT_MIN_ANGLE, TILT_MAX_ANGLE])

            # görsel için tüm trackleri yazdır
            for tid, ctr in tracked:
                # radius varsa ekle
                r = det_map.get(tid, (ctr[0], ctr[1], 0))[2]
                target_info_list.append((tid, (ctr[0], ctr[1], r)))
                # draw track centroid
                cv2.circle(frame, (int(ctr[0]), int(ctr[1])), 4, (0, 255, 0), -1)
                cv2.putText(frame, f"ID:{tid}", (int(ctr[0]) + 6, int(ctr[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,255,0), 1)

            # Kilitlenme mantığı (track tabanlı)
            if kilitlenen_hedef_id == matched_id and matched_id is not None:
                lazer_state = 1
                elapsed = loop_start - kilitlenme_baslangic_zamani
                status_text = f"KILITLENIYOR ID:{matched_id} [{elapsed:.2f}s]"
                if elapsed >= KILITLENME_SURESI:
                    # Vuruldu
                    vurulan_hedefler[matched_id if matched_id is not None else f"raw_{int(x)}_{int(y)}"] = loop_start
                    if shot_sound:
                        shot_sound.play()
                    print(f"!!! HEDEF VURULDU -> ID:{matched_id} (r={radius:.1f})")
                    lazer_state = 0
                    kilitlenen_hedef_id = None
                    # beyaz flash
                    white = np.ones_like(frame, dtype=np.uint8) * 255
                    cv2.imshow(WINDOW_NAME, white)
                    cv2.waitKey(80)
            else:
                # yeni kilitleme başlat
                kilitlenen_hedef_id = matched_id
                kilitlenme_baslangic_zamani = loop_start
                lazer_state = 1
                status_text = f"YENI HEDEF - ID:{matched_id}"
            # draw chosen circle
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        else:
            # hiçbir uygun hedef yok
            status_text = "UYGUN HEDEF YOK - BEKLEMEDE"
            kilitlenen_hedef_id = None
            lazer_state = 0

        # temizlenecek vurulan hedefleri sil (beklemeden sonra)
        to_delete = [tid for tid, ttime in vurulan_hedefler.items() if loop_start - ttime > VURULAN_HEDEF_BEKLEME]
        for tid in to_delete:
            del vurulan_hedefler[tid]

        # --- MOTOR SMOOTHING: zaman sabiti bazlı ---
        # alpha = 1 - exp(-dt / tau)
        if MOTOR_RESPONSE_TC <= 0:
            alpha = 1.0
        else:
            alpha = 1 - math.exp(-dt / MOTOR_RESPONSE_TC)
        # yeni açıları kademeli uygula
        current_pan_angle += (target_pan_angle - current_pan_angle) * alpha
        current_tilt_angle += (target_tilt_angle - current_tilt_angle) * alpha

        # Komut hazırla & gönder
        komut = f"{int(current_pan_angle)},{int(current_tilt_angle)},{int(lazer_state)}\n"
        if arduino_connected:
            try:
                arduino.write(komut.encode())
            except Exception as e:
                print(f"[Arduino] Yazma hatası: {e}")
                arduino_connected = False
        else:
            # simulation print (daha seyrek)
            if int(loop_start * 2) % 2 == 0:
                print(f"[SimCmd] {komut.strip()}")

        # FPS hesaplama (smoothing)
        if dt > 0:
            fps = 1.0 / dt
            if smoothed_fps is None:
                smoothed_fps = fps
            else:
                smoothed_fps = smoothed_fps * LOOP_FPS_SMOOTH + fps * (1 - LOOP_FPS_SMOOTH)

        draw_hud(frame, status_text, [(tid, (int(x), int(y), int(r))) for tid,(x,y,r) in target_info_list], fps=smoothed_fps if smoothed_fps else None)
        cv2.imshow(WINDOW_NAME, frame)
        cv2.imshow(CALIB_WINDOW, mask)  # mask gösterimi kalibrasyonu kolaylaştırır

        # klavye: q=quit, s=save calib
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # kaydet
            cfg_to_save = {
                "low_h": calib["low_h"], "low_s": calib["low_s"], "low_v": calib["low_v"],
                "high_h": calib["high_h"], "high_s": calib["high_s"], "high_v": calib["high_v"]
            }
            save_calibration(cfg_to_save)

    # döngü dışı temizlik
    print("Sistem kapanıyor...")
    try:
        if arduino_connected:
            arduino.write(b"90,90,0\n")
            time.sleep(0.3)
            arduino.close()
    except Exception:
        pass
    cap.release()
    cv2.destroyAllWindows()
    print("Sistem kapandı.")

if __name__ == "__main__":
    main()