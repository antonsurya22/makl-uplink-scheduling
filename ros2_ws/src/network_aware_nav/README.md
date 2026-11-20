# network_aware_nav — paket manuskrip & eksperimen

Paket lengkap untuk submission ke *Intelligent Service Robotics* (Springer).

> **Mau langsung mulai simulasi? Buka `MULAI-DISINI.md`.**
> Dokumen ini untuk manuskrip; dokumen itu untuk eksperimen.

## Isi

| File | Keterangan |
|---|---|
| `sn-article-final.tex` | Manuskrip. Butuh `sn-jnl.cls`, `sn-*.bst`, `references.bib`, dan `fig1`–`fig4` PDF di direktori yang sama. |
| `sn-bibliography.bib` | Sumber BibTeX 47 entri. Tidak wajib dipakai (bibliografi sudah ditanam di `.tex`), tapi berguna kalau mau menambah referensi. **Verifikasi tiap entri** ke rekaman penerbit. |
| `references_additions.bib` | 10 entri baru saja, kalau kamu lebih suka memakai `references.bib` lamamu dan hanya menambahkan. |
| `sn-article-COMPILED-preview.pdf` | Bukti kompilasi (20 hal., nol warning). |
| `rencana-eksperimen.md` | Rencana bertahap dengan gerbang go/no-go. Baca ini pertama. |
| `worldgen.py` | Generator peta prosedural: 16 peta training + `eval_building` + `arena_lab`. |
| `worlds_sample.tar.gz` | Keluaran `worldgen.py --seed 3`, siap pakai. |
| `stage0_runner.py` | Runner episode ROS 2 untuk Tahap 0. |
| `analyze.py` | Statistik + generator tabel LaTeX. |
| `preview_*.png` | Pratinjau dua peta evaluasi. |
| `MULAI-DISINI.md` | Panduan langkah demi langkah menuju verdict Gerbang 0. |
| `stage0_arena.launch.py` | Bringup Gazebo + Nav2 + emulator untuk Tahap 0. |
| `preflight.py` | Cek nama topic/action/service/parameter sebelum eksperimen. |

## Urutan kerja

```bash
# 0. baca rencananya
$EDITOR rencana-eksperimen.md

# 1. bangkitkan peta (kunci seed, commit manifest.json)
python3 worldgen.py --out ./worlds --n-train 16 --seed 3

# 2. TAHAP 0 — uji apakah masalahnya ada. Jangan lewati ini.
ros2 run <your_pkg> stage0_runner.py --episodes 40 --out stage0.csv
python3 analyze.py gate0 --csv stage0.csv
#    exit code 0 = lanjut ke Tahap A. exit code 1 = berhenti, perbaiki dulu.

# 3. ... Tahap A–C sesuai rencana ...

# 4. TAHAP D — setelah semua gerbang lolos
python3 analyze.py tables   --csv episodes.csv --out tables/
python3 analyze.py hardware --csv hardware.csv --out tables/
#    lalu \input{} berkas tables/*.tex ke manuskrip
```

## Cara memasang di Overleaf

1. Ganti isi `sn-article.tex` dengan `sn-article-final.tex`.
2. Unggah `fig1_architecture.pdf` … `fig4_supervisor.pdf` ke folder `Figure/`.
   Dua PNG peta sudah ada di sana.
3. Kompilasi **pdfLaTeX dua kali**. Tidak perlu BibTeX.

Bibliografi 47 entri sudah **ditanam langsung** di `.tex` sebagai
`thebibliography`. Ini menghapus seluruh langkah BibTeX, jadi sitasi tidak bisa
lagi muncul sebagai `[? ]` karena nama file `.bib` tidak cocok — itulah
penyebab masalahmu: `\bibliography{references}` mencari `references.bib`,
sedangkan proyekmu berisi `sn-bibliography.bib`.

Kalau nanti kamu ingin kembali ke BibTeX (misalnya untuk menambah referensi
dengan mudah), hapus blok `thebibliography` di akhir file, kembalikan
`\bibliography{sn-bibliography}`, dan pakai `sn-bibliography.bib` di paket ini.

## Status verifikasi

**Manuskrip sudah dikompilasi** dengan `sn-jnl.cls` resmi (paket Desember 2024):

```
20 halaman     abstrak 285 kata
0 sitasi / referensi undefined
0 overfull hbox
0 underfull hbox
0 LaTeX warning
```

Diuji dengan direktori yang **tidak berisi satu pun file `.bib`**, untuk
membuktikan dokumennya benar-benar mandiri.

`sn-article-COMPILED-preview.pdf` adalah hasilnya, memakai kotak placeholder
untuk empat gambar TikZ yang belum ada.

Juga sudah diuji:

- `worldgen.py` — 18 world, semua SDF parse bersih, PGM/YAML sesuai format
  `nav2_map_server`, konektivitas start–goal terverifikasi tiap peta.
- `analyze.py` — ketiga subperintah dijalankan dengan data sintetis.

Yang **belum** diuji dan harus diverifikasi lokal:

- `stage0_runner.py` — **belum dijalankan terhadap ROS 2 hidup**. Verifikasi
  dulu:
  ```bash
  ros2 param list /controller_server | grep -i vel   # nama parameter kecepatan
  ros2 action list  | grep navigate                  # navigate_to_pose ada?
  ros2 service list | grep set_entity_state          # reset pose Gazebo
  ```
  Lalu sesuaikan `CONSERVATIVE_PARAMS`, `NORMAL_PARAMS`, dan `PROFILE_CMD`.

## Catatan tata letak halaman 1

Kalau suatu saat halaman 1 terlihat terpotong di bagian atas, cari warning ini
di log:

```
LaTeX Warning: Optional argument of \twocolumn too tall on page 1.
```

Itu berarti blok judul + abstrak + keywords melebihi tinggi yang disediakan
`sn-jnl`. Penyebabnya gabungan panjang judul dan panjang abstrak, bukan salah
satunya saja. Abstrak sekarang 285 kata dan judul 85 karakter, dan kombinasi itu
muat dengan margin aman. Kalau kamu memanjangkan salah satunya, cek log-nya lagi.

---

## Yang masih harus kamu kerjakan di manuskrip

- Isi metadata penulis dan afiliasi (masih placeholder).
- Gambar ulang **Fig. 1**: agen DQN sekarang di sisi *onboard*, bukan off-board.
- Buat **Fig. 5**: permukaan kecepatan admissible dari Eq. (2), pakai `b`,
  `Δt`, dan `d_margin` hasil kalibrasi Tahap B.
- Verifikasi 10 entri `references_additions.bib` ke rekaman penerbit.
- Isi Section 6–7 setelah Tahap D.

## Klaim yang harus dibuktikan atau dihapus

Manuskrip menyatakan bahwa `tc netem` tidak bisa mengemulasi degradasi untuk
stack ROS 2 bersimulasi-waktu karena `/clock` ikut tertunda. Ini kuat, dan
karena itu harus didukung figur validasi tiga arah (Tahap A, ±1 hari):

```
1. Sim + tc netem                    → age diperkirakan TIDAK naik
2. Sim + network_emulator            → age naik sesuai injeksi
3. Hardware dua mesin + tc netem     → age naik (ground truth)
```

Kalau (1) ternyata menaikkan age, **hapus klaimnya** dari §5.2 dan dari daftar
kontribusi — jangan diperhalus.
