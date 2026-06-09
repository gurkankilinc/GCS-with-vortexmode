# VORTEX Uçuş Modu — ArduPlane Entegrasyonu

VORTEX, sabit yatış açısıyla sürekli daire çizen özel bir uçuş modudur. Uçuş mantığını
ArduPlane'in mevcut `ModeLoiter` modundan miras alır; bu yüzden güvenli ve derlenmesi
kolaydır. Mod numarası **27**'dir (30 numara "offboard" için ayrılmıştır).

> Bu klasördeki `mode_vortex.cpp` dosyası `ArduPlane/` dizinine kopyalanır; aşağıdaki
> küçük eklemeler de ilgili dosyalara elle yapılır. Tüm değişiklikler ArduPlane'in
> kendi kaynak ağacında yapılır.

## 1. `ArduPlane/mode_vortex.cpp`
Bu klasördeki dosyayı `ArduPlane/` içine kopyalayın.

## 2. `ArduPlane/mode.h`
`Mode::Number` enum'una ekleyin:
```cpp
VORTEX = 27,
```
`class ModeLoiter` tanımından **sonra** ekleyin (ondan türediği için sonra gelmeli):
```cpp
class ModeVortex : public ModeLoiter
{
public:
    Mode::Number mode_number() const override { return Mode::Number::VORTEX; }
    const char *name() const override { return "Vortex"; }
    const char *name4() const override { return "VRTX"; }
};
```

## 3. `ArduPlane/Plane.h`
Mod nesnesini ekleyin (diğer `ModeXxx mode_xxx;` satırlarının yanına):
```cpp
ModeVortex mode_vortex;
```

## 4. `ArduPlane/control_modes.cpp`
`mode_from_mode_num()` switch'ine ekleyin:
```cpp
case Mode::Number::VORTEX:
    ret = &mode_vortex;
    break;
```

## 5. `ArduPlane/events.cpp`
İki failsafe switch'inde, otomatik modların (LOITER vb.) bulunduğu gruplara
`case Mode::Number::VORTEX:` satırını ekleyin.

## 6. `ArduPlane/GCS_Plane.cpp`
`update_vehicle_sensor_status_flags()` switch'inde, AUTO/RTL/LOITER grubuna
`case Mode::Number::VORTEX:` ekleyin.

## 7. `ArduPlane/GCS_MAVLink_Plane.cpp`
- `base_mode()` switch'inde AUTO/GUIDED grubuna `case Mode::Number::VORTEX:` ekleyin.
- `send_available_mode()` içindeki `fw_modes[]` dizisine `&plane.mode_vortex,` ekleyin.

## Derleme ve Yükleme
```bash
# SITL'de test
cd ardupilot
./waf configure --board sitl
./waf plane

# Gerçek karta (Cube Orange), USB bağlıyken
./waf configure --board CubeOrange
./waf plane --upload
```

## Davranış Ayarı
`mode_vortex.cpp` içindeki `VORTEX_BANK_CD` sabiti yatış açısını belirler
(2500 = 25°). Dönüş yarıçapı ayrıca `WP_LOITER_RAD` parametresiyle değişir.

> Güvenlik: Gerçek uçuştan önce SITL'de bolca test edin; ilk denemede yüksek irtifada
> uçun ve yedek pilot her an MANUAL/FBWA/RTL'e geçmeye hazır olsun.
