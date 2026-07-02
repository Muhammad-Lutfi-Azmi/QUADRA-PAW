# QUADRA-PAW GUI REV10

GUI untuk QUADRA-PAW ESP32.

## Jalankan

```bash
py -m pip install pyserial
py main.py
```

## Revisi REV10

- Angkat kaki depan dan belakang dipisah.
- Default kaki depan dibuat lebih rendah, kaki belakang lebih tinggi.
- Parameter baru: LIFTF dan LIFTB.
- S6 lurus maju/mundur tetap target langsung.

## Default

- SPEED = 220
- LIFTF = 650
- LIFTB = 900
- S5A = 1200
- S5B = 1800
- S6MAJU = 1450
- S6MUNDUR = 1500
- S6KANAN = 1330
- S6KIRI = 1100
