# BOZGUN v5 - Kontrol Sistemi

Görüntü işleme tabanlı, çoklu hedef takibi yapabilen ve Arduino üzerinden pan-tilt servo motorlarını kontrol eden bir otomatik nişan/takip sistemi.

## Özellikler

- **Renk tabanlı hedef tespiti**: HSV renk uzayında ayarlanabilir eşik değerleriyle hedef tespiti
- **Canlı kalibrasyon paneli**: Trackbar'lar ile HSV aralığını gerçek zamanlı ayarlama ve kaydetme
- **Çoklu hedef takibi**: Basit centroid tabanlı tracker ile birden fazla hedefi eşzamanlı takip etme, kimlik (ID) atama
- **Kilitlenme mantığı**: Seçilen hedefe belirli bir süre kilitlenme, "vurulan" hedefleri belirli bir süre yeniden seçmeme
- **Yumuşatılmış motor hareketi**: Zaman sabiti (tau) tabanlı üstel yumuşatma ile ani servo sıçramalarını önleme
- **Arduino entegrasyonu**: Seri port üzerinden pan açısı, tilt açısı ve lazer durumu gönderimi
- **Simülasyon modu**: Arduino bağlı değilse otomatik olarak simülasyon moduna geçme
- **Ekran bilgilendirmesi (HUD)**: Durum metni, FPS ve hedef bilgilerinin görüntü üzerine çizilmesi
- **Ses efekti**: Hedef vurulduğunda ses çalma (opsiyonel)

## Kullanılan Teknolojiler

- Python 3
- OpenCV (`opencv-python`)
- PySerial (`pyserial`)
- NumPy
- Pygame (ses için)
- Arduino (servo ve lazer kontrolü için)

## Donanım Gereksinimleri

- Bilgisayara bağlı bir kamera (varsayılan index: `0`)
- Arduino (varsayılan port: `COM5`, baudrate: `9600`)
- 2 adet servo motor (pan ve tilt ekseni)
- Lazer modülü (opsiyonel, güvenli/göz teması olmayan düşük güçlü bir modül önerilir)

## Kurulum

```bash
pip install -r requirements.txt
```

`requirements.txt` içeriği:
```
opencv-python
pyserial
numpy
pygame
```

> Not: Ses efekti için proje klasöründe bir `laser_shot.mp3` dosyası bulunmalıdır. Bulunamazsa program uyarı vererek sessiz modda devam eder.

## Yapılandırma

Betiğin başındaki ayarlar bölümünden aşağıdaki parametreler özelleştirilebilir:

| Parametre | Açıklama |
|---|---|
| `ARDUINO_PORT` | Arduino'nun bağlı olduğu seri port |
| `CAMERA_INDEX` | Kullanılacak kamera indeksi |
| `PAN_MIN_ANGLE` / `PAN_MAX_ANGLE` | Pan servo açı sınırları |
| `TILT_MIN_ANGLE` / `TILT_MAX_ANGLE` | Tilt servo açı sınırları |
| `KILITLENME_SURESI` | Hedefe kilitlenme süresi (saniye) |
| `MOTOR_RESPONSE_TC` | Motor tepki zaman sabiti (küçük = hızlı, büyük = yumuşak) |
| `MIN_HEDEF_YARICAPI` | Algılanacak minimum hedef boyutu (piksel) |

## Kullanım

```bash
python bozgun_kontrol.py
```

Program başladığında iki pencere açılır:
1. **BOZGUN v5 - KONTROL**: Canlı kamera görüntüsü ve HUD bilgileri
2. **HSV Kalibrasyon**: Trackbar'lar ve maske görüntüsü

### Klavye Kısayolları

| Tuş | İşlev |
|---|---|
| `Q` | Programdan çıkış |
| `S` | Mevcut HSV kalibrasyon ayarlarını `hsv_calib.json` dosyasına kaydet |

### Kalibrasyon

HSV Kalibrasyon penceresindeki trackbar'ları kullanarak hedef renginizin alt/üst HSV sınırlarını ayarlayın. Maske penceresinde sadece hedef nesnenin beyaz göründüğünden emin olun, ardından `S` tuşu ile ayarları kaydedin. Bir sonraki çalıştırmada bu ayarlar otomatik olarak yüklenir.

## Arduino Haberleşme Protokolü

Python tarafından Arduino'ya her döngüde şu formatta bir komut satırı gönderilir:

```
<pan_açısı>,<tilt_açısı>,<lazer_durumu>\n
```

Örnek: `95,88,1`

Arduino tarafında bu satırın virgülle ayrılarak parse edilmesi ve servo/lazer pinlerine uygulanması gerekir.

## Proje Yapısı

```
.
├── bozgun_kontrol.py     # Ana kontrol betiği
├── hsv_calib.json        # Kaydedilmiş kalibrasyon ayarları (otomatik oluşur)
├── laser_shot.mp3        # Vuruş ses efekti (opsiyonel)
├── requirements.txt
└── README.md
```

## Notlar

- Arduino bağlı değilse program otomatik olarak simülasyon moduna geçer ve komutları terminale yazdırır.
- Bu proje eğitim/hobi amaçlı bir görüntü işleme ve gömülü sistem entegrasyon çalışmasıdır.
