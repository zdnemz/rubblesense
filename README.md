# RubbleSense — alat bantu SAR berbasis radar tanda-hidup (ESP32)

Prototipe riset untuk membantu pencarian korban di reruntuhan. ESP32 membaca data radar (I/Q kompleks per bin jarak),
mencari pola napas dengan pemrosesan sinyal yang mengendalikan laju alarm palsu, lalu menampilkan hasil yang
**tidak pernah** menyatakan area aman. Versi firmware 1.0.0 (24 Oktober 2026).

> **Status jujur — baca dulu.** Seluruh logika (DSP, keputusan, firmware, protokol serial, UI, alat bantu) berjalan dan
> lulus uji **di komputer, dengan simulator radar**. Belum ada bagian yang diuji pada radar sungguhan, pada manusia, atau
> pada reruntuhan; firmware belum pernah dikompilasi untuk ESP32 dan waktu DSP di ESP32 belum diukur. Semua angka
> kinerja adalah angka simulasi (derau Gaussian putih, pemantul titik ideal). **Jangan memakai alat ini untuk keputusan
> lapangan** sebelum melewati validasi bertahap (`docs/ARSITEKTUR.md`, bagian 11). Penetrasi puing menuntut front-end
> berfrekuensi rendah / UWB; modul 60 GHz tidak boleh diandalkan (bagian 1.3).

## Prinsip keselamatan

* **Tidak ada hasil "aman" atau "kosong".** Hasil paling negatif selalu berbunyi: *"No reliable life-sign pattern detected
  in this scan. DO NOT clear area."* — tanpa LED, tanpa persentase.
* **Kandidat bukan kepastian.** Bukti hanya ordinal (strong / moderate / weak); setiap kandidat menyuruh konfirmasi dengan
  metode lain dan menyatakan "bukan bukti kehidupan".
* **Data atau kondisi yang tidak layak tidak pernah dibaca sebagai "negatif":** gerak sensor, ADC jenuh, frame hilang,
  cap waktu tidak cocok, IMU hilang ⇒ `INVALID_SCAN`; derau jauh di atas baseline ⇒ `LOW_SENSITIVITY`; garis yang sama
  dengan getaran sensor/platform ⇒ `CONFOUNDED`.
* Aturan-aturan itu diperiksa otomatis setelah setiap scan di dalam uji, dan pada log scan oleh
  `tools/validate_scan_log.py`.

## Yang ada di sini

| Bagian | Isi |
|---|---|
| `lib/rs_core/` | inti DSP portabel (C++11): FFT, CA-CFAR dengan alarm palsu keluarga, konfirmasi di segmen independen, kompensasi dan deteksi getaran platform, keputusan, IMU, protokol I/Q, simulator |
| `src/` | firmware ESP32 (Arduino-ESP32): task akuisisi / DSP / UI, sumber `SIM` dan `SERIAL`, layar ILI9341, LED, buzzer, konsol, log JSON di LittleFS |
| `diagram.json`, `wokwi.toml` | rangkaian Wokwi (dibangkitkan dari `src/rs_pins.h` dan diperiksa sebagai netlist) |
| `test/`, `tools/` | uji inti, uji firmware di host (header Arduino/ESP32 tiruan, jam virtual), Monte-Carlo, alat rekam/putar ulang, validator log |
| `docs/ARSITEKTUR.md` | arsitektur, aturan keselamatan, alur DSP, protokol, hasil verifikasi, batasan, rencana validasi lapangan |
| `docs/results/`, `docs/screens/` | hasil Monte-Carlo dan harness (CSV / Markdown), gambar layar UI |

## Mulai cepat (di komputer)

Prasyarat: `g++` (C++17), `make`, Python 3; Pillow hanya untuk `make screens`.

```
make check          # semua uji host: inti DSP, firmware, alat bantu, sintaks, diagram (beberapa menit)
make montecarlo     # studi Monte-Carlo penuh (beberapa menit) -> docs/results/
make screens        # gambar layar UI -> build/screens_png/
make help
```

Hasil terakhir `make check`: inti DSP 313 pemeriksaan; firmware di host 9.610 (UART2) dan 9.626 (UART0); alat bantu 124;
204 kompilasi sintaks; diagram 15 bagian dan 38 kabel cocok dengan peta pin; 181 catatan log tanpa pelanggaran — semuanya
lulus. Ini bukti bahwa logika bekerja di bawah simulator, **bukan** bukti kinerja pada puing nyata.

Contoh memutar rekaman ke firmware tanpa perangkat (setelah `make tools` atau `make check`):

```
build/bin/dump_sim --preset A --frames 1800 --out a.csv
python3 tools/send_iq_replay.py --file a.csv --out a.bin --quiet
build/bin/fw_replay --stream a.bin --cmd-at 500 "src serial" --cmd-at 3500 scan --until-results 1
```

## Build ESP32 dan Wokwi (belum pernah dijalankan oleh pembuatnya)

```
pio run -e esp32dev            # lapangan: I/Q front-end di UART2 (GPIO36 RX / GPIO14 TX)
pio run -e esp32dev_usb_iq     # hardware-in-the-loop: I/Q lewat port USB serial
pio run -e wokwi               # simulator Wokwi (VS Code + ekstensi Wokwi -> "Start Simulator")
```

Lakukan daftar periksa build pertama di `docs/ARSITEKTUR.md` bagian 12.6 (waktu DSP, watchdog, heap, flash) sebelum
mempercayai apa pun dari build itu. Pemutaran rekaman ke ESP32 / Wokwi lewat `--port` (pyserial) juga belum diuji.

## Dokumentasi

Mulai dari `docs/ARSITEKTUR.md`: bagian 1 (status dan batasan), 3 (aturan keselamatan), 6 (arti setiap hasil), 8
(verifikasi, hanya simulasi), 10 (keterbatasan), 11 (rencana validasi lapangan), 12 (cara menjalankan).

Lisensi belum ditetapkan oleh tim; tambahkan sebelum dibagikan.
