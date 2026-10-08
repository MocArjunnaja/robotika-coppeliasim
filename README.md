<div align="center">

# Robotika — CoppeliaSim Pioneer P3-DX

Kontrol robot **Pioneer P3-DX** di CoppeliaSim lewat **Legacy Remote API** (Python):
baca 16 sensor ultrasonik, kemudi dengan keyboard, dan seterusnya.

![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-Edu-E5322D)
![Robot](https://img.shields.io/badge/Robot-Pioneer%20P3--DX-555555)
![Status](https://img.shields.io/badge/Progress-3%20dari%204%20tugas-F5A623)

</div>

---

## Progress Tugas

| # | Tugas | File | Status |
|:-:|-------|------|:------:|
| 1 | Akses 16 sensor dan tampilkan outputnya | [`Assignment_2_16sensor.py`](Assign/Assignment_2_16sensor.py) | Selesai |
| 2 | Gerakkan robot dengan keyboard | [`Assignment_2_16sensor_keyboard.py`](Assign/Assignment_2_16sensor_keyboard.py) | Selesai |
| 3 | Object follower (orang atau benda) | [`Assignment_2_16sensor_keyboard_object_follower.py`](Assign/Assignment_2_16sensor_keyboard_object_follower.py) | Selesai |
| 4 | Navigasi point-to-point | — | Belum |

---

## Struktur Folder

```text
.
├── Assign/
│   ├── Assignment_2.py                  # Basis: koneksi, sensor, motor (robot diam)
│   ├── Assignment_2_16sensor.py         # Tugas 1: cetak 16 sensor
│   ├── Assignment_2_16sensor_keyboard.py# Tugas 2: + kontrol keyboard & kinematika
│   ├── Assignment_2_16sensor_keyboard_object_follower.py # Tugas 3: object follower
│   ├── sim.py / simConst.py             # Binding Remote API
│   ├── remoteApi.dll                    # Library Remote API (Windows)
│   └── send*MovementSequence*.py        # Contoh bawaan CoppeliaSim (lengan robot)
└── Reff/
    └── sim_lib.zip                      # Paket binding Remote API
```

---

## Cara Menjalankan

**Prasyarat**

- CoppeliaSim Edu dengan scene berisi `PioneerP3DX`
- Python 3.10 dan paket berikut:

```bash
pip install numpy keyboard
```

**Langkah**

1. Buka scene di CoppeliaSim. Remote API server legacy default berjalan di port `19997`.
2. Jalankan **Start Simulation** di CoppeliaSim.
3. Dari folder `Assign/`, jalankan salah satu script:

```bash
python Assignment_2_16sensor_keyboard.py
```

4. Tekan `Esc` untuk menghentikan robot dan keluar.

> Library `keyboard` di Windows bisa membaca tombol global. Di Linux perlu hak akses root.

---

## Kontrol Keyboard (Tugas 2)

| Tombol | Fungsi |
|:------:|--------|
| `W` / `S` | Kecepatan linear maju / mundur |
| `A` / `D` | Kecepatan sudut |
| `Esc` | Berhenti dan keluar |

Kecepatan naik `0.025` per siklus (0.1 s) sampai batas `veloMax`, lalu **meluruh ke nol** saat tombol dilepas.

---

## Object Follower (Tugas 3)

| Tombol | Fungsi |
|:------:|--------|
| `F` | Mode object follower (default saat start) |
| `M` | Mode manual (`W`/`A`/`S`/`D` seperti Tugas 2) |
| `Esc` | Berhenti dan keluar |

```mermaid
flowchart LR
    S["16 sensor [S1..S16]"] --> F["Front Sensors Selection<br/>S3, S4, S5, S6"]
    F --> E["Estimasi<br/>d = min(S4, S5)<br/>θ = S6 − S3"]
    E --> C["Control<br/>v = K1(d − d_ref)<br/>ω = K2(θ − θ_ref)"]
    C --> I["Inverse Kinematics<br/>φR, φL"]
    I --> N["Velocity Normalization<br/>φRN, φLN"]
    N --> R((Pioneer P3-DX))
```

- **Front Sensors Selection.** S1..S16 di slide sama dengan `ultrasonicSensor[0..15]`, jadi S3..S6 = sensor `[2..5]`.
- **Estimasi.** `d = min(S4, S5)` adalah jarak objek di depan. `θ = S6 − S3` adalah selisih jarak kiri-kanan: positif berarti objek di kiri.
- **Control.** Slide menulis `v = K1(d_ref − d)` dan `ω = K2(θ_ref − θ)`. Dengan K1, K2 positif, rumus itu membuat robot mundur menjauhi objek dan berbelok ke arah yang salah, jadi kedua selisih dibalik. Hasilnya robot maju saat objek jauh, mundur saat terlalu dekat, dan berbelok ke arah objek.
- **Inverse Kinematics.** `[φR, φL]ᵀ = [[R/2, R/2], [R/2L, −R/2L]]⁻¹ [v, ω]ᵀ`, dengan `R = 0.195/2 m` dan `L = 0.381/2 m` (setengah lebar badan).
- **Velocity Normalization.** `φmax = max(|φR|, |φL|)`. Jika `φmax > φnorm` (`1.5 rad/s`), kedua roda dikali `φnorm/φmax` sehingga arah gerak tetap sama.
- Jika S3..S6 tidak mendeteksi apa pun, robot berhenti.

Parameter `D_REF = 0.4 m`, `K1 = 0.5`, `K2 = 0.5`, `VELO_NORM = 1.5`, `NO_DETECTION = 1.0` ada di bagian atas file.

---

## Cara Kerja

```mermaid
flowchart LR
    K[Keyboard W/A/S/D] --> V["velo_cmd = [v, w]"]
    V --> I["solveKinematics<br/>(inverse kinematics)"]
    I --> M["Kecepatan roda<br/>kiri & kanan"]
    M --> R((Pioneer P3-DX))
    R --> S["16 sensor ultrasonik"]
    S --> P[Cetak jarak ke terminal]
```

**Sensor.** 16 sensor `/PioneerP3DX/ultrasonicSensor[0..15]` dibaca tiap 0.1 s.
Jarak yang dipakai adalah komponen z dari `detectedPoint`. Jika tidak ada objek terdeteksi, nilainya `2.0`.

**Kinematika.** Perintah `[v, w]` diubah menjadi kecepatan sudut roda dengan matriks invers:

```text
[ R/2   R/2 ] [wl]   [v]
[ R/L  -R/L ] [wr] = [w]        R = 0.195/2 m (jari-jari roda), L = 0.381 m (lebar badan)
```

Hasilnya `[wl, wr]` dikirim ke `leftMotor` dan `rightMotor`.

---

## Catatan

- Jika robot berbelok berlawanan dengan tombol `A`/`D`, tukar tombol di `setRobotMotionUsingkeyboard`.
- Batas `veloMax = 5.` m/s sangat tinggi. Turunkan ke sekitar `0.5` jika robot terlalu cepat atau terbalik.

---

<div align="center">

Mata kuliah **Robotika** — Semester 7

</div>
