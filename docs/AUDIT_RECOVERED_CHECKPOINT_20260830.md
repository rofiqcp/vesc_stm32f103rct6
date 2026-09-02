# Audit Recovered Checkpoint - 2026-08-30

Checkpoint ini dibuat ulang langsung dari baseline `vesc_stm32f103rct6(3).zip` karena file checkpoint yang sebelumnya ditautkan tidak benar-benar tersimpan di runtime.

Perubahan yang diterapkan ulang:

1. `isr_overruns` hanya bertambah jika total ISR > `FOC_ISR_SLOT_CYCLES`; ambang 85% hanya `near_deadline`.
2. ISR menyimpan cycle mentah; pembagian float ke detik dipindah ke getter task-side.
3. Accumulator RT data Id/Iq/Vd/Vq/Iin/Imotor memakai satu seqlock snapshot koheren.
4. APPCONF mempertahankan full VESC wire image agar field UI/pasif tidak memicu false truncation.
5. Kegagalan flash write memakai record committed lama sebagai fallback bila masih valid.

Tidak ada `pio run` yang dijalankan. Verifikasi hardware/build tetap diperlukan.
