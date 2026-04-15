import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import control
import sys
import os

def analyze_sopdt(csv_file):
    print(f"--- Đang phân tích SOPDT cho file: {csv_file} ---")
    try:
        df = pd.read_csv(csv_file, comment='#', names=['t_ms', 'pwm_pct', 'alpha_deg'])
    except Exception as e:
        print(f"Lỗi: {e}")
        return

    t = (df['t_ms'] - df['t_ms'].iloc[0]).values / 1000.0
    u = df['pwm_pct'].values
    y = df['alpha_deg'].values

    # Khử offset
    u_detrend = u - np.mean(u)
    y_detrend = y - np.mean(y)
    fs = 1.0 / np.mean(np.diff(t))

    # 1. ƯỚC LƯỢNG ĐÁP ỨNG TẦN SỐ (ETFE)
    nperseg = 1024
    f, Pxy = signal.csd(u_detrend, y_detrend, fs=fs, nperseg=nperseg)
    f, Pxx = signal.welch(u_detrend, fs=fs, nperseg=nperseg)
    H_data = Pxy / Pxx # Đáp ứng tần số thực tế (số phức)
    
    # Chỉ lấy dải tần số có ích (ví dụ 0.1Hz đến 8Hz)
    mask = (f > 0.05) & (f < 8.0)
    f_fit = f[mask]
    H_fit = H_data[mask]

    # 2. ĐỊNH NGHĨA HÀM KHỚP SỐ PHỨC (SOPDT)
    # Vì curve_fit không hỗ trợ số phức trực tiếp, ta nối phần thực và phần ảo
    def sopdt_model(freqs, K, wn, zeta, L):
        s = 1j * 2 * np.pi * freqs
        # G(s) bậc 2
        G = K * (wn**2) / (s**2 + 2*zeta*wn*s + wn**2)
        # Nhân thêm thành phần trễ e^(-Ls)
        G_delayed = G * np.exp(-s * L)
        return np.concatenate([np.real(G_delayed), np.imag(G_delayed)])

    # Nối phần thực và ảo của dữ liệu thực tế
    H_fit_stacked = np.concatenate([np.real(H_fit), np.imag(H_fit)])

    # 3. THỰC HIỆN KHỚP DỮ LIỆU
    p0 = [1.0, 10.0, 0.5, 0.02] 
    bounds = (0, [100, 200, 2.0, 0.5]) 

    try:
        popt, _ = curve_fit(sopdt_model, f_fit, H_fit_stacked, p0=p0, bounds=bounds)
        K, wn, zeta, L = popt
        
        # --- TÍNH ĐỘ KHỚP (FIT ACCURACY) ---
        H_pred_stacked = sopdt_model(f_fit, K, wn, zeta, L)
        
        # R-squared cho dữ liệu phức (Real + Imag)
        res = H_fit_stacked - H_pred_stacked
        ss_res = np.sum(res**2)
        ss_tot = np.sum((H_fit_stacked - np.mean(H_fit_stacked))**2)
        r2_complex = 1 - (ss_res / ss_tot)

        # R-squared riêng cho Magnitude (dB) - trực quan hơn
        mag_db_data = 20 * np.log10(np.abs(H_fit))
        mag_db_pred = 20 * np.log10(np.abs(H_pred_stacked[:len(f_fit)] + 1j*H_pred_stacked[len(f_fit):]))
        res_mag = mag_db_data - mag_db_pred
        r2_mag = 1 - (np.sum(res_mag**2) / np.sum((mag_db_data - np.mean(mag_db_data))**2))

        print(f"\n--- KẾT QUẢ NHẬN DẠNG SOPDT ---")
        print(f"Gain (K)          : {K:.4f}")
        print(f"Nat. Freq (wn)    : {wn:.4f} rad/s")
        print(f"Damping (zeta)    : {zeta:.4f}")
        print(f"Dead time (L)     : {L*1000:.2f} ms")
        print(f"-------------------------------")
        print(f"Độ khớp Magnitude : {r2_mag*100:.2f} %")
        print(f"Độ khớp Tổng thể  : {r2_complex*100:.2f} %")

        # Tạo hàm truyền (Sử dụng Pade approximation bậc 2)
        num_base = [K * wn**2]
        den_base = [1, 2 * zeta * wn, wn**2]
        G_base = control.TransferFunction(num_base, den_base)
        num_pade, den_pade = control.pade(L, n=2)
        G_dead = control.TransferFunction(num_pade, den_pade)
        G_total = G_base * G_dead
        
    except Exception as e:
        print(f"Lỗi khi khớp dữ liệu: {e}")
        return

    # 4. VẼ ĐỒ THỊ BODE KIỂM CHỨNG
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Biên độ
    ax1.semilogx(f, 20 * np.log10(np.abs(H_data)), 'k.', alpha=0.1, label='Dữ liệu thực tế')
    s_model = 1j * 2 * np.pi * f_fit
    H_model = K * (wn**2) / (s_model**2 + 2*zeta*wn*s_model + wn**2) * np.exp(-s_model * L)
    
    ax1.semilogx(f_fit, 20 * np.log10(np.abs(H_model)), 'r-', linewidth=2.5, label=f'SOPDT Model (Fit: {r2_mag*100:.1f}%)')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.grid(True, which="both")
    ax1.legend()
    ax1.set_title(f"SOPDT Identification: {os.path.basename(csv_file)}")

    # Pha
    ax2.semilogx(f, np.angle(H_data, deg=True), 'k.', alpha=0.1)
    ax2.semilogx(f_fit, np.angle(H_model, deg=True), 'r-', linewidth=2.5)
    ax2.set_ylabel('Phase (deg)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.grid(True, which="both")
    
    # Thêm text box hiển thị thông số chính ngay trên đồ thị
    info_text = f"K = {K:.3f}\nwn = {wn:.2f} rad/s\nzeta = {zeta:.3f}\nL = {L*1000:.1f} ms\n\nFit = {r2_complex*100:.1f}%"
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    ax1.text(0.02, 0.05, info_text, transform=ax1.transAxes, fontsize=10, verticalalignment='bottom', bbox=props)

    plt.tight_layout()
    plt.show()

    plt.tight_layout()
    plt.show()

    # 5. MÔ PHỎNG KIỂM CHỨNG STEP RESPONSE
    plt.figure(figsize=(8, 4))
    T_step, Y_step = control.step_response(G_total)
    plt.plot(T_step, Y_step)
    plt.title(f"Step Response (K={K:.2f}, L={L*1000:.1f}ms)")
    plt.xlabel("Time (s)")
    plt.ylabel("Output")
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Sử dụng: python sys_id_sopdt.py <file.csv>")
    else:
        analyze_sopdt(sys.argv[1])
