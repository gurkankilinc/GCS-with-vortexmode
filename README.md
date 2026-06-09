# Çelebi Yer İstasyonu

ArduPilot/Pixhawk tabanlı sabit kanat İHA'lar için **PySide6** ile geliştirilmiş yer
kontrol istasyonu (GCS). MAVLink üzerinden araca bağlanır, temel telemetriyi canlı
gösterir, uçuş modlarını değiştirir ve takıma özel olarak yazılan **VORTEX** uçuş
moduna geçişi destekler. Tüm geliştirme ArduPilot SITL simülasyonu üzerinde yapılmış
olup, yalnızca bağlantı satırı değiştirilerek gerçek Pixhawk (Cube Orange) ile çalışır.

> TEKNOFEST – Savaşan İHA yarışması kapsamında Çelebi Takımı için geliştirilmiştir.

## Özellikler

- MAVLink bağlantısı (pymavlink), SITL ve gerçek donanım için tek satırlık geçiş
- Ayrı thread'de canlı telemetri okuma (arayüz hiç donmaz), sinyal/slot köprüsü
- PFD tarzı **yapay ufuk**: yatış açısı yayı, pitch merdiveni, sönümlü (akıcı) hareket
- Çevrimdışı **iz haritası**: kalkış noktasına göre konum, geçilen yol, mesafe halkaları
- Canlı **irtifa çubuğu** ve telemetri paneli (mod, hız, GPS, batarya, yön, konum)
- **Bağlantı sağlığı** izleme ve **uyarı bandı** (GPS/batarya/link kaybı annunciator)
- Uçuş modu değiştirme (GUIDED, CIRCLE, RTL, MANUAL) + özel **VORTEX** modu
- Siyah-sarı, aksan temelli arayüz

## Ekran Görüntüsü

![Yer istasyonu arayüzü](docs/images/arayuz.png)

## Kurulum

Gereksinimler: Python 3.10+ ve bir ArduPilot SITL veya gerçek bir Pixhawk.

```bash
python3 -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

## Çalıştırma

Simülasyon (ArduPilot SITL açıkken, varsayılan `udpin:127.0.0.1:14550`):

```bash
python main.py
```

Gerçek donanım (Cube Orange, USB): `main.py` içindeki bağlantı satırını değiştirin:

```python
BAGLANTI = "/dev/ttyACM0"   # ayrıca mavlink_connection'a baud=115200 ekleyin
```

## Mimari

Arayüz, uçuş mantığından ayrılmıştır:

- `AracBaglantisi` — MAVLink bağlantısı ve komutlar (mod değiştirme `DO_SET_MODE` ile).
- `TelemetriThread` — ayrı thread'de sürekli mesaj okur, `Signal` ile arayüze aktarır.
- `YapayUfuk`, `HaritaWidget` — `QPainter` ile çizilen göstergeler.
- `AnaPencere` — arayüz, sönümleme döngüsü ve uyarı mantığı.

Bu ayrım sayesinde gerçek donanıma geçişte yalnızca tek bir bağlantı satırı değişir.

## VORTEX Modu (firmware)

VORTEX, ArduPlane'e eklenen özel bir uçuş modudur (mod numarası **27**). Sabit yatış
açısıyla sürekli daire çizer; mevcut ve kanıtlanmış `ModeLoiter` mantığını miras alır.
Firmware'e entegrasyon adımları ve kaynak kodu için: [`firmware/vortex-modu/`](firmware/vortex-modu/)

## Lisans

MIT
