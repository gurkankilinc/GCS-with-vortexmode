"""
Faz 12 - EFIS seritleri + harita derinligi + mod buton ikonlari (siyah-sari).

Eklenenler:
  - PFD: yapay ufkun solunda kayan HIZ seridi, saginda kayan IRTIFA seridi
    (degerler serit uzerinde akar, ortadaki kutuda anlik deger; sonumlu hareket)
  - Harita: eskidikce solan iz, ucak golgesi, "Home: X m" uzaklik etiketi
  - Mod butonlari: ikonlar + MANUAL icin uyari rengi

Calistirma (Terminal 2, .venv aktif, SITL Terminal 1'de acik):
    python faz12_efis.py
"""

import sys
import math
import time
import threading
from pymavlink import mavutil

from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QProgressBar, QSizePolicy,
    QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
)
from PySide6.QtCore import Qt, QThread, QTimer, Signal, QPointF, QRectF
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QPainterPath, QPolygonF,
    QLinearGradient, QFont, QPixmap, QIcon,
)

VORTEX_NO = 27
METRE_PER_DERECE = 111320.0

AKSAN = "#FFC400"
METIN = "#e8e8ea"
SOLUK = "#6f7480"
UYARI = "#ff9f1a"
TEHLIKE = "#ff4d4d"
CANLI = "#FFC400"
PASIF = "#3a3d44"

TEMA = """
QWidget { background-color: #0e0f12; color: #e8e8ea;
          font-family: "DejaVu Sans", sans-serif; font-size: 13px; }
QLabel#marka     { color: #e8e8ea; font-size: 19px; font-weight: 800; letter-spacing: 2px; }
QLabel#markaVurgu{ color: #FFC400; font-size: 19px; font-weight: 800; letter-spacing: 2px; }
QLabel#durumtxt  { color: #e8e8ea; font-size: 13px; font-weight: 700;
                   font-family: "DejaVu Sans Mono", monospace; }
QGroupBox {
    background-color: #16181d; border: 1px solid #23262d; border-radius: 10px;
    margin-top: 16px; font-size: 11px; font-weight: 700; color: #8b909c;
}
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 14px; padding: 3px 8px; }
QLabel#ad    { color: #6f7480; font-size: 12px; }
QLabel#deger { color: #e8e8ea; font-size: 14px; font-weight: 700; font-family: "DejaVu Sans Mono", monospace; }
QPushButton {
    background-color: #1b1e24; color: #e8e8ea; border: 1px solid #2c2f37;
    border-radius: 8px; padding: 10px 14px; font-weight: 700;
}
QPushButton:hover    { border-color: #FFC400; color: #FFC400; }
QPushButton:pressed  { background-color: #23262d; }
QPushButton:disabled { color: #4a4d55; border-color: #20232a; }
QPushButton[dikkat="true"] { border-color: #5a4517; color: #d9a648; }
QPushButton[dikkat="true"]:hover { border-color: #ff9f1a; color: #ffb74d; }
QPushButton[aktif="true"] { background-color: #FFC400; color: #0e0f12; border-color: #FFC400; }
QPushButton#baglan { background-color: #FFC400; color: #0e0f12; border: none; padding: 8px 20px; }
QPushButton#baglan:hover { background-color: #ffd23d; }
QProgressBar {
    border: 1px solid #23262d; border-radius: 6px; background-color: #101216;
    text-align: center; color: #8b909c; font-weight: 700; font-size: 11px;
}
QProgressBar::chunk { background-color: #FFC400; border-radius: 5px; }
"""


class AracBaglantisi:
    BAGLANTI = "udpin:127.0.0.1:14550"
    # Gercek Pixhawk (CubeOrange, USB): BAGLANTI = "/dev/ttyACM0"  (+ baud=115200)

    def __init__(self):
        self.master = None
        self.kilit = threading.Lock()

    def bagli_mi(self):
        return self.master is not None

    def baglan(self):
        self.master = mavutil.mavlink_connection(self.BAGLANTI)
        if self.master.wait_heartbeat(timeout=10) is None:
            self.master = None
            raise TimeoutError("Heartbeat gelmedi (SITL acik mi?)")
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

    def kes(self):
        if self.master:
            self.master.close()
            self.master = None

    def _mod_numarasi(self, mod_adi):
        if mod_adi == "VORTEX":
            return VORTEX_NO
        harita = self.master.mode_mapping() or {}
        return harita.get(mod_adi)

    def moda_gec(self, mod_adi):
        with self.kilit:
            if not self.bagli_mi():
                raise RuntimeError("Once baglanmalisin.")
            no = self._mod_numarasi(mod_adi)
            if no is None:
                raise ValueError(f"Gecersiz mod: {mod_adi}")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, no,
                0, 0, 0, 0, 0)


class TelemetriThread(QThread):
    veri = Signal(object)

    def __init__(self, arac):
        super().__init__()
        self.arac = arac
        self._calisiyor = True

    def run(self):
        durum = {
            "mod": "-", "arm": False, "irtifa": 0.0,
            "yer_hizi": 0.0, "hava_hizi": 0.0, "heading": 0,
            "gps_fix": 0, "uydu": 0, "lat": 0.0, "lon": 0.0,
            "voltaj": 0.0, "batarya_yuzde": 0, "roll": 0.0, "pitch": 0.0,
        }
        m = self.arac.master
        while self._calisiyor and self.arac.bagli_mi():
            with self.arac.kilit:
                msg = m.recv_match(blocking=False)
            if msg is None:
                self.msleep(10)
                continue
            tip = msg.get_type()
            if tip == "HEARTBEAT":
                durum["mod"] = "VORTEX" if msg.custom_mode == VORTEX_NO else m.flightmode
                durum["arm"] = bool(m.motors_armed())
            elif tip == "VFR_HUD":
                durum["yer_hizi"] = msg.groundspeed
                durum["hava_hizi"] = msg.airspeed
                durum["heading"] = msg.heading
            elif tip == "GLOBAL_POSITION_INT":
                durum["irtifa"] = msg.relative_alt / 1000.0
                durum["lat"] = msg.lat / 1e7
                durum["lon"] = msg.lon / 1e7
            elif tip == "GPS_RAW_INT":
                durum["gps_fix"] = msg.fix_type
                durum["uydu"] = msg.satellites_visible
            elif tip == "SYS_STATUS":
                durum["voltaj"] = msg.voltage_battery / 1000.0
                durum["batarya_yuzde"] = max(0, msg.battery_remaining)
            elif tip == "ATTITUDE":
                durum["roll"] = math.degrees(msg.roll)
                durum["pitch"] = math.degrees(msg.pitch)
            else:
                continue
            self.veri.emit(dict(durum))

    def durdur(self):
        self._calisiyor = False


class PFD(QWidget):
    """Yapay ufuk + sol HIZ seridi + sag IRTIFA seridi. Sonumlu hareket."""
    SERIT_SOL = 52   # hiz seridi genisligi
    SERIT_SAG = 58   # irtifa seridi genisligi

    def __init__(self):
        super().__init__()
        self.setMinimumSize(380, 260)
        # hedef ve goruntulenen (sonumlu) degerler
        self.h = {"roll": 0.0, "pitch": 0.0, "spd": 0.0, "alt": 0.0}
        self.d = {"roll": 0.0, "pitch": 0.0, "spd": 0.0, "alt": 0.0}

    def hedefle(self, roll, pitch, spd, alt):
        self.h.update(roll=roll, pitch=pitch, spd=spd, alt=alt)

    def adim(self):
        degisti = False
        for k, kat in (("roll", .18), ("pitch", .18), ("spd", .2), ("alt", .2)):
            yeni = self.d[k] + (self.h[k] - self.d[k]) * kat
            if abs(yeni - self.d[k]) > 0.02:
                self.d[k] = yeni
                degisti = True
        if degisti:
            self.update()

    # ---- yardimci: kayan serit ----
    def _serit(self, p, rect, deger, ppu, adim_kucuk, adim_etiket,
               etiket_sagda, birim):
        p.save()
        yol = QPainterPath()
        yol.addRoundedRect(rect, 6, 6)
        p.setClipPath(yol)
        p.fillPath(yol, QColor("#101216"))

        cy = rect.center().y()
        yari = rect.height() / 2
        kucuk_font = QFont("DejaVu Sans Mono", 8)
        p.setFont(kucuk_font)

        # gorunur deger araligi
        v_min = deger - yari / ppu
        v_max = deger + yari / ppu
        t = math.floor(v_min / adim_kucuk) * adim_kucuk
        while t <= v_max:
            if t >= 0:
                y = cy - (t - deger) * ppu
                buyuk = (abs(t % adim_etiket) < 1e-6)
                p.setPen(QPen(QColor("#5a5f6a" if buyuk else "#33373f"),
                              2 if buyuk else 1))
                if etiket_sagda:   # cizgiler solda, etiket sagda (irtifa)
                    p.drawLine(int(rect.left() + 2), int(y),
                               int(rect.left() + (12 if buyuk else 7)), int(y))
                    if buyuk:
                        p.setPen(QColor("#aab0bb"))
                        p.drawText(QRectF(rect.left() + 14, y - 8,
                                          rect.width() - 16, 16),
                                   Qt.AlignmentFlag.AlignVCenter |
                                   Qt.AlignmentFlag.AlignLeft, f"{int(t)}")
                else:              # cizgiler sagda, etiket solda (hiz)
                    p.drawLine(int(rect.right() - (12 if buyuk else 7)), int(y),
                               int(rect.right() - 2), int(y))
                    if buyuk:
                        p.setPen(QColor("#aab0bb"))
                        p.drawText(QRectF(rect.left() + 2, y - 8,
                                          rect.width() - 16, 16),
                                   Qt.AlignmentFlag.AlignVCenter |
                                   Qt.AlignmentFlag.AlignRight, f"{int(t)}")
            t += adim_kucuk
        p.restore()

        # cerceve + birim etiketi
        p.setPen(QPen(QColor("#23262d"), 1))
        p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        p.drawRoundedRect(rect, 6, 6)
        p.setPen(QColor(SOLUK))
        p.setFont(QFont("DejaVu Sans", 8, QFont.Weight.Bold))
        p.drawText(QRectF(rect.left(), rect.top() - 16, rect.width(), 14),
                   Qt.AlignmentFlag.AlignCenter, birim)

        # anlik deger kutusu (ortada)
        kutu = QRectF(rect.left() + 2, cy - 13, rect.width() - 4, 26)
        p.setPen(QPen(QColor(AKSAN), 1.5))
        p.setBrush(QColor("#0e0f12"))
        p.drawRoundedRect(kutu, 4, 4)
        p.setPen(QColor(AKSAN))
        p.setFont(QFont("DejaVu Sans Mono", 11, QFont.Weight.Bold))
        p.drawText(kutu, Qt.AlignmentFlag.AlignCenter, f"{deger:.0f}")

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        serit_h = h - 26
        sol_rect = QRectF(2, 20, self.SERIT_SOL, serit_h)
        sag_rect = QRectF(w - self.SERIT_SAG - 2, 20, self.SERIT_SAG, serit_h)

        # --- ortadaki ufuk dairesi ---
        ccx = (sol_rect.right() + sag_rect.left()) / 2
        cy = h / 2
        r = min(sag_rect.left() - sol_rect.right(), h) / 2 - 6
        px_der = 3.2

        yol = QPainterPath()
        yol.addEllipse(QPointF(ccx, cy), r, r)
        p.save()
        p.setClipPath(yol)
        p.translate(ccx, cy)
        p.rotate(-self.d["roll"])
        p.translate(0, self.d["pitch"] * px_der)
        b = r * 3
        gok = QLinearGradient(0, -b, 0, 0)
        gok.setColorAt(0.0, QColor("#3d4a5c"))
        gok.setColorAt(1.0, QColor("#222a35"))
        p.fillRect(QRectF(-b, -b, 2 * b, b), gok)
        yer = QLinearGradient(0, 0, 0, b)
        yer.setColorAt(0.0, QColor("#15181d"))
        yer.setColorAt(1.0, QColor("#070809"))
        p.fillRect(QRectF(-b, 0, 2 * b, b), yer)
        p.setPen(QPen(QColor(AKSAN), 2))
        p.drawLine(int(-b), 0, int(b), 0)
        p.setFont(QFont("DejaVu Sans", 7))
        for d in (-30, -20, -10, 10, 20, 30):
            yy = int(-d * px_der)
            p.setPen(QPen(QColor("#cfd3da"), 1))
            p.drawLine(-24, yy, 24, yy)
            p.drawText(30, yy + 4, str(abs(d)))
            p.drawText(-44, yy + 4, str(abs(d)))
        p.restore()

        # yatis acisi yayi
        p.save()
        p.translate(ccx, cy)
        for ang, major in [(-60, 1), (-45, 0), (-30, 1), (-20, 0), (-10, 0),
                           (0, 1), (10, 0), (20, 0), (30, 1), (45, 0), (60, 1)]:
            p.save()
            p.rotate(ang)
            ln = 9 if major else 5
            p.setPen(QPen(QColor(SOLUK), 2 if major else 1))
            p.drawLine(0, int(-r), 0, int(-r + ln))
            p.restore()
        p.save()
        p.rotate(self.d["roll"])
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(AKSAN))
        p.drawPolygon(QPolygonF([
            QPointF(0, -r + 11), QPointF(-5, -r + 20), QPointF(5, -r + 20)]))
        p.restore()
        p.restore()

        # sabit ucak referansi + cerceve
        p.setPen(QPen(QColor(AKSAN), 3))
        p.drawLine(int(ccx - 30), int(cy), int(ccx - 10), int(cy))
        p.drawLine(int(ccx + 10), int(cy), int(ccx + 30), int(cy))
        p.drawLine(int(ccx), int(cy - 2), int(ccx), int(cy + 2))
        p.setPen(QPen(QColor("#3a3d44"), 2))
        p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
        p.drawEllipse(QPointF(ccx, cy), r, r)

        # --- seritler ---
        self._serit(p, sol_rect, self.d["spd"], ppu=4.0,
                    adim_kucuk=5, adim_etiket=10, etiket_sagda=False, birim="HIZ m/s")
        self._serit(p, sag_rect, self.d["alt"], ppu=2.0,
                    adim_kucuk=10, adim_etiket=20, etiket_sagda=True, birim="İRT m")


class HaritaWidget(QWidget):
    UCAK = QPolygonF([
        QPointF(0, -11), QPointF(1.6, -4), QPointF(1.6, -1),
        QPointF(10, 2), QPointF(10, 4), QPointF(1.6, 3),
        QPointF(1.6, 8), QPointF(4.5, 10.5), QPointF(4.5, 11.8), QPointF(0, 9.8),
        QPointF(-4.5, 11.8), QPointF(-4.5, 10.5), QPointF(-1.6, 8),
        QPointF(-1.6, 3), QPointF(-10, 4), QPointF(-10, 2), QPointF(-1.6, -1), QPointF(-1.6, -4),
    ])

    def __init__(self):
        super().__init__()
        self.setMinimumSize(220, 260)
        self.home = None
        self.iz = []
        self.konum = None
        self.heading = 0.0

    def sifirla(self):
        self.home = None
        self.iz = []
        self.konum = None
        self.update()

    def guncelle(self, lat, lon, heading):
        if abs(lat) < 1e-6 and abs(lon) < 1e-6:
            return
        if self.home is None:
            self.home = (lat, lon)
        lat0, lon0 = self.home
        kuzey = (lat - lat0) * METRE_PER_DERECE
        dogu = (lon - lon0) * METRE_PER_DERECE * math.cos(math.radians(lat0))
        self.konum = (dogu, kuzey)
        self.heading = heading
        if not self.iz or (abs(dogu - self.iz[-1][0]) + abs(kuzey - self.iz[-1][1])) > 1.0:
            self.iz.append((dogu, kuzey))
            if len(self.iz) > 2000:
                self.iz = self.iz[-2000:]
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2
        r_px = min(w, h) / 2 - 22

        p.setPen(QPen(QColor("#23262d"), 1))
        p.setBrush(QColor("#101216"))
        p.drawEllipse(QPointF(cx, cy), r_px, r_px)

        if self.home is None or self.konum is None:
            p.setPen(QColor(SOLUK))
            p.drawText(QRectF(0, 0, w, h), Qt.AlignmentFlag.AlignCenter, "GPS bekleniyor...")
            return

        menzil = 50.0
        for e, n in self.iz + [self.konum]:
            menzil = max(menzil, abs(e), abs(n))
        menzil *= 1.15
        olcek = r_px / menzil

        def ekran(e, n):
            return QPointF(cx + e * olcek, cy - n * olcek)

        for frac in (0.5, 1.0):
            rr = r_px * frac
            p.setPen(QPen(QColor("#23262d"), 1))
            p.setBrush(QBrush(Qt.BrushStyle.NoBrush))
            p.drawEllipse(QPointF(cx, cy), rr, rr)
            p.setPen(QColor(SOLUK))
            p.drawText(int(cx + 4), int(cy - rr - 4), f"{int(menzil * frac)} m")

        p.setPen(QPen(QColor("#1c1f25"), 1))
        p.drawLine(int(cx - r_px), int(cy), int(cx + r_px), int(cy))
        p.drawLine(int(cx), int(cy - r_px), int(cx), int(cy + r_px))
        p.setPen(QColor(SOLUK))
        p.drawText(int(cx - 4), int(cy - r_px - 6), "K")

        # solan iz: eski kisim seffaf, yeni kisim parlak
        n_iz = len(self.iz)
        if n_iz >= 2:
            for i in range(1, n_iz):
                alfa = int(35 + 220 * (i / n_iz))
                p.setPen(QPen(QColor(255, 196, 0, alfa), 2))
                p.drawLine(ekran(*self.iz[i - 1]), ekran(*self.iz[i]))

        # home isareti
        hp = ekran(0, 0)
        p.setPen(QPen(QColor(SOLUK), 1))
        p.drawLine(int(hp.x() - 5), int(hp.y()), int(hp.x() + 5), int(hp.y()))
        p.drawLine(int(hp.x()), int(hp.y() - 5), int(hp.x()), int(hp.y() + 5))

        # ucak: once golge, sonra govde
        up = ekran(*self.konum)
        for golge, renk in ((True, QColor(0, 0, 0, 130)), (False, QColor(AKSAN))):
            p.save()
            p.translate(up.x() + (2 if golge else 0), up.y() + (3 if golge else 0))
            p.rotate(self.heading)
            p.setPen(Qt.PenStyle.NoPen if golge else QPen(QColor("#0e0f12"), 1))
            p.setBrush(renk)
            p.drawPolygon(self.UCAK)
            p.restore()

        # home'a uzaklik etiketi
        uzaklik = math.hypot(*self.konum)
        p.setPen(QColor("#aab0bb"))
        p.setFont(QFont("DejaVu Sans Mono", 9, QFont.Weight.Bold))
        p.drawText(QRectF(0, h - 20, w, 16),
                   Qt.AlignmentFlag.AlignCenter, f"Home: {uzaklik:.0f} m")


class AnaPencere(QWidget):
    MODLAR = [("GUIDED", "◎"), ("CIRCLE", "↻"), ("RTL", "⌂"),
              ("MANUAL", "⚠"), ("VORTEX", "◉")]

    def __init__(self):
        super().__init__()
        self.arac = AracBaglantisi()
        self.thread = None
        self.son_guncelleme = 0.0
        self.aktif_mod = None
        self.son_d = None
        self.link_kayip = False
        self.setWindowTitle("Celebi - Yer Istasyonu")
        self.setMinimumSize(1120, 620)
        self._arayuzu_kur()
        self.kontrol = QTimer(self)
        self.kontrol.setInterval(500)
        self.kontrol.timeout.connect(self.baglanti_kontrol)
        self.animasyon = QTimer(self)
        self.animasyon.setInterval(33)
        self.animasyon.timeout.connect(self._animate)

    def _arayuzu_kur(self):
        ana = QVBoxLayout(self)
        ana.setContentsMargins(16, 14, 16, 14)
        ana.setSpacing(10)

        baslik = QHBoxLayout()
        m1 = QLabel("ÇELEBİ"); m1.setObjectName("markaVurgu")
        m2 = QLabel("YER İSTASYONU"); m2.setObjectName("marka")
        baslik.addWidget(m1); baslik.addWidget(m2); baslik.addStretch()
        self.led = QLabel(); self.led.setFixedSize(12, 12); self._led(PASIF)
        self.durum = QLabel("Bağlantı yok"); self.durum.setObjectName("durumtxt")
        self.btn_baglan = QPushButton("BAĞLAN"); self.btn_baglan.setObjectName("baglan")
        self.btn_baglan.clicked.connect(self.baglanti_degistir)
        baslik.addWidget(self.led); baslik.addSpacing(4)
        baslik.addWidget(self.durum); baslik.addSpacing(14)
        baslik.addWidget(self.btn_baglan)
        ana.addLayout(baslik)

        self.uyari = QLabel("")
        self.uyari.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.uyari.setVisible(False)
        ana.addWidget(self.uyari)

        ust = QHBoxLayout(); ust.setSpacing(12)

        kutu_panel = QGroupBox("TELEMETRİ")
        kutu_panel.setMaximumWidth(250)
        izgara = QGridLayout(kutu_panel)
        izgara.setContentsMargins(14, 10, 14, 14)
        izgara.setVerticalSpacing(10)
        self.degerler = {}
        satirlar = [
            ("mod", "Mod"), ("arm", "Arm"), ("irtifa", "İrtifa"),
            ("yer_hizi", "Yer hızı"), ("hava_hizi", "Hava hızı"), ("heading", "Yön"),
            ("gps", "GPS"), ("konum", "Konum"), ("batarya", "Batarya"),
        ]
        for i, (anahtar, baslik_metin) in enumerate(satirlar):
            ad = QLabel(baslik_metin); ad.setObjectName("ad")
            deger = QLabel("—"); deger.setObjectName("deger")
            izgara.addWidget(ad, i, 0)
            izgara.addWidget(deger, i, 1, alignment=Qt.AlignmentFlag.AlignRight)
            self.degerler[anahtar] = deger
        izgara.setColumnStretch(0, 1)
        ust.addWidget(kutu_panel, 0)

        kutu_pfd = QGroupBox("PFD")
        yu = QVBoxLayout(kutu_pfd); yu.setContentsMargins(14, 10, 14, 14)
        self.pfd = PFD(); yu.addWidget(self.pfd)
        ust.addWidget(kutu_pfd, 6)

        kutu_harita = QGroupBox("HARİTA (İZ)")
        yh = QVBoxLayout(kutu_harita); yh.setContentsMargins(14, 10, 14, 14)
        self.harita = HaritaWidget(); yh.addWidget(self.harita)
        ust.addWidget(kutu_harita, 5)

        ana.addLayout(ust, 1)

        kutu_mod = QGroupBox("UÇUŞ MODLARI")
        mlay = QHBoxLayout(kutu_mod); mlay.setContentsMargins(14, 10, 14, 14); mlay.setSpacing(10)
        self.mod_butonlari = []
        for mod, ikon in self.MODLAR:
            b = QPushButton(f"{ikon}  {mod}")
            b.setEnabled(False)
            b.setProperty("mod_adi", mod)
            if mod == "MANUAL":
                b.setProperty("dikkat", "true")
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            b.clicked.connect(lambda _, mm=mod: self.moda_gec(mm))
            mlay.addWidget(b); self.mod_butonlari.append(b)
        ana.addWidget(kutu_mod)

    def _led(self, renk):
        self.led.setStyleSheet(f"background-color:{renk}; border-radius:6px;")

    @staticmethod
    def _renk(label, renk=METIN):
        label.setStyleSheet(f"color:{renk}; font-weight:700;"
                            f"font-family:'DejaVu Sans Mono', monospace; font-size:14px;")

    def _set_durum(self, metin, renk=METIN, led=CANLI):
        self.durum.setStyleSheet(f"color:{renk}; font-weight:700;"
                                 f"font-family:'DejaVu Sans Mono', monospace; font-size:13px;")
        self.durum.setText(metin)
        self._led(led)

    def _guncelle_aktif_mod(self, mod):
        if mod == self.aktif_mod:
            return
        self.aktif_mod = mod
        for b in self.mod_butonlari:
            b.setProperty("aktif", "true" if b.property("mod_adi") == mod else "false")
            b.style().unpolish(b); b.style().polish(b)

    def _uyari_guncelle(self):
        if not self.arac.bagli_mi():
            self.uyari.setVisible(False)
            return
        kritik, dikkat = [], []
        if self.link_kayip:
            kritik.append("BAĞLANTI KAYBI")
        d = self.son_d
        if d:
            if d["gps_fix"] < 3:
                kritik.append("GPS YOK")
            elif d["uydu"] < 6:
                dikkat.append("ZAYIF GPS")
            y = d["batarya_yuzde"]
            if 0 < y < 20:
                kritik.append("BATARYA KRİTİK")
            elif 0 < y < 40:
                dikkat.append("BATARYA DÜŞÜK")
        if kritik:
            self.uyari.setText("UYARI:   " + "    ·    ".join(kritik + dikkat))
            self.uyari.setStyleSheet(
                "background:#3a0d0d; color:#ff6b6b; border:1px solid #ff4d4d;"
                "border-radius:6px; padding:7px; font-weight:800; letter-spacing:1px;")
            self.uyari.setVisible(True)
        elif dikkat:
            self.uyari.setText("DİKKAT:   " + "    ·    ".join(dikkat))
            self.uyari.setStyleSheet(
                "background:#332600; color:#ffb74d; border:1px solid #ff9f1a;"
                "border-radius:6px; padding:7px; font-weight:800; letter-spacing:1px;")
            self.uyari.setVisible(True)
        else:
            self.uyari.setVisible(False)

    def _animate(self):
        self.pfd.adim()

    def baglanti_degistir(self):
        if self.arac.bagli_mi():
            self._baglantiyi_kes()
            return
        self._set_durum("Bağlanıyor...", METIN, UYARI)
        QApplication.processEvents()
        try:
            self.arac.baglan()
        except Exception as e:
            self._set_durum(f"HATA: {e}", TEHLIKE, TEHLIKE)
            return
        self.harita.sifirla()
        self._set_durum("BAĞLI", METIN, CANLI)
        self.btn_baglan.setText("KES")
        for b in self.mod_butonlari:
            b.setEnabled(True)
        self.son_guncelleme = time.monotonic()
        self.link_kayip = False
        self.thread = TelemetriThread(self.arac)
        self.thread.veri.connect(self.telemetri_guncelle)
        self.thread.start()
        self.kontrol.start()
        self.animasyon.start()

    def _baglantiyi_kes(self):
        self.kontrol.stop()
        self.animasyon.stop()
        if self.thread:
            self.thread.durdur(); self.thread.wait(); self.thread = None
        self.arac.kes()
        self._set_durum("Bağlantı yok", SOLUK, PASIF)
        self.btn_baglan.setText("BAĞLAN")
        for b in self.mod_butonlari:
            b.setEnabled(False)
        self._guncelle_aktif_mod(None)
        self.son_d = None
        self.uyari.setVisible(False)

    def moda_gec(self, mod):
        try:
            self.arac.moda_gec(mod)
        except Exception as e:
            self._set_durum(f"HATA: {e}", TEHLIKE, TEHLIKE)

    def baglanti_kontrol(self):
        if not self.arac.bagli_mi():
            return
        yas = time.monotonic() - self.son_guncelleme
        if yas > 3.0:
            self.link_kayip = True
            self._set_durum(f"VERİ YOK ({yas:.0f} sn)", TEHLIKE, TEHLIKE)
        else:
            self.link_kayip = False
            self._set_durum(f"BAĞLI · {yas:.1f} sn", METIN, CANLI)
        self._uyari_guncelle()

    def telemetri_guncelle(self, d):
        self.son_guncelleme = time.monotonic()
        self.son_d = d

        self.degerler["mod"].setText(d["mod"]); self._renk(self.degerler["mod"], AKSAN)
        self._guncelle_aktif_mod(d["mod"])

        if d["arm"]:
            self.degerler["arm"].setText("ARMED"); self._renk(self.degerler["arm"], UYARI)
        else:
            self.degerler["arm"].setText("DISARMED"); self._renk(self.degerler["arm"], METIN)

        self.degerler["irtifa"].setText(f"{d['irtifa']:.1f} m")
        self.degerler["yer_hizi"].setText(f"{d['yer_hizi']:.1f} m/s")
        self.degerler["hava_hizi"].setText(f"{d['hava_hizi']:.1f} m/s")
        self.degerler["heading"].setText(f"{int(d['heading'])}°")

        self.degerler["gps"].setText(f"fix {d['gps_fix']} · {d['uydu']}")
        self._renk(self.degerler["gps"], METIN if d["gps_fix"] >= 3 else TEHLIKE)

        self.degerler["konum"].setText(f"{d['lat']:.4f},{d['lon']:.4f}")

        y = d["batarya_yuzde"]
        self.degerler["batarya"].setText(f"{d['voltaj']:.1f}V · {y}%")
        if 0 < y < 20:
            self._renk(self.degerler["batarya"], TEHLIKE)
        elif 0 < y < 40:
            self._renk(self.degerler["batarya"], UYARI)
        else:
            self._renk(self.degerler["batarya"], METIN)

        self.pfd.hedefle(d["roll"], d["pitch"], d["hava_hizi"], d["irtifa"])
        self.harita.guncelle(d["lat"], d["lon"], d["heading"])
        self._uyari_guncelle()

    def closeEvent(self, event):
        self._baglantiyi_kes()
        event.accept()


def _uygulama_ikonu():
    pm = QPixmap(64, 64)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(QColor(AKSAN))
    p.drawRoundedRect(4, 4, 56, 56, 14, 14)
    p.translate(32, 32)
    p.scale(1.7, 1.7)
    p.setBrush(QColor("#0e0f12"))
    p.drawPolygon(HaritaWidget.UCAK)
    p.end()
    return QIcon(pm)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(TEMA)
    ikon = _uygulama_ikonu()
    app.setWindowIcon(ikon)
    pencere = AnaPencere()
    pencere.setWindowIcon(ikon)
    pencere.show()
    sys.exit(app.exec())
