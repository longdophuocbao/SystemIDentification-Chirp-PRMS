import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import control
import sys
# python sys_id_analysis.py sweep_data.csv
def analyze_system(csv_file):
    # 1. ĐỌC DỮ LIỆU
    print(f"--- Đang phân tích file: {csv_file} ---")
    try:
        # Đọc bỏ qua các dòng comment '#'
        df = pd.read_csv(csv_file, comment='#', names=['t_ms', 'pwm_pct', 'alpha_deg'])
    except Exception as e:
        print(f"Lỗi đọc file: {e}")
        return

    # Chuyển sang đơn vị chuẩn
    t = (df['t_ms'] - df['t_ms'].iloc[0]).values / 1000.0 # giây
    u = df['pwm_pct'].values   # Tín hiệu vào (PWM %)
    y = df['alpha_deg'].values # Tín hiệu ra (Góc độ)

    # Khử Offset (Đưa về 0 để nhận dạng đặc tính động học xung quanh điểm làm việc)
    u_detrend = u - np.mean(u)
    y_detrend = y - np.mean(y)

    fs = 1.0 / np.mean(np.diff(t))
    print(f"Tần số lấy mẫu thực tế: {fs:.2f} Hz")

    # 2. ƯỚC LƯỢNG ĐÁP ỨNG TẦN SỐ (ETFE)
    # Sử dụng phương pháp Welch để tính mật độ phổ công suất
    f, Pxy = signal.csd(u_detrend, y_detrend, fs=fs, nperseg=1024)
    f, Pxx = signal.welch(u_detrend, fs=fs, nperseg=1024)
    
    # Hàm truyền thực nghiệm H(f) = Pxy / Pxx
    H = Pxy / Pxx
    mag = np.abs(H)
    phase = np.angle(H, deg=True)

    # Tính độ gắn kết (Coherence) - Kiểm tra chất lượng dữ liệu
    f_coh, coh = signal.coherence(u_detrend, y_detrend, fs=fs, nperseg=1024)

    # 3. KHỚP MÔ HÌNH (SỬ DỤNG HỆ BẬC 2 MẪU)
    # G(s) = K * wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
    def model_func(freqs, K, wn, zeta):
        s = 1j * 2 * np.pi * freqs
        sys_gs = K * (wn**2) / (s**2 + 2*zeta*wn*s + wn**2)
        return np.abs(sys_gs)

    # Giới hạn dải tần số để khớp (thường từ f_start đến f_end của sweep)
    mask = (f > 0.1) & (f < 10.0) 
    try:
        popt, _ = curve_fit(model_func, f[mask], mag[mask], p0=[1.0, 10.0, 0.5], bounds=(0, [100, 200, 2.0]))
        K_est, wn_est, zeta_est = popt
        print(f"\n--- KẾT QUẢ NHẬN DẠNG (Mô hình bậc 2) ---")
        print(f"Hệ số truyền (K)      : {K_est:.4f}")
        print(f"Tần số riêng (wn)     : {wn_est:.4f} rad/s ({wn_est/(2*np.pi):.2f} Hz)")
        print(f"Hệ số tắt dần (zeta)  : {zeta_est:.4f}")
        
        # Tạo hàm truyền trong thư viện Control
        num = [K_est * wn_est**2]
        den = [1, 2 * zeta_est * wn_est, wn_est**2]
        G_s = control.TransferFunction(num, den)
        print(f"Hàm truyền G(s) = \n{G_s}")
    except Exception as e:
        print(f"Không thể khớp mô hình: {e}")
        G_s = None

    # 4. VẼ ĐỒ THỊ BODE
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Biên độ (Magnitude)
    ax1.semilogx(f, 20 * np.log10(mag), 'b.', alpha=0.3, label='Dữ liệu thực tế')
    if G_s is not None:
        mag_model, _, w_model = control.bode(G_s, omega=2*np.pi*f[mask], plot=False)
        ax1.semilogx(f[mask], 20 * np.log10(mag_model), 'r-', linewidth=2, label='Mô hình khớp được')
    ax1.set_ylabel('Magnitude (dB)')
    ax1.grid(True, which="both")
    ax1.legend()
    ax1.set_title(f"Bode Plot Analysis: {csv_file}")

    # Pha (Phase)
    ax2.semilogx(f, phase, 'b.', alpha=0.3)
    if G_s is not None:
        _, phase_model, _ = control.bode(G_s, omega=2*np.pi*f[mask], plot=False)
        ax2.semilogx(f[mask], np.degrees(phase_model), 'r-', linewidth=2)
    ax2.set_ylabel('Phase (deg)')
    ax2.grid(True, which="both")

    # Độ gắn kết (Coherence) - Càng gần 1 càng tốt
    ax3.semilogx(f_coh, coh, 'g-', label='Coherence (chất lượng dữ liệu)')
    ax3.axhline(0.6, color='r', linestyle='--', label='Ngưỡng tin cậy (0.6)')
    ax3.set_ylabel('Coherence')
    ax3.set_xlabel('Frequency (Hz)')
    ax3.grid(True, which="both")
    ax3.legend()

    plt.tight_layout()
    plt.show()

    # 5. MÔ PHỎNG KIỂM CHỨNG (Dùng hàm bước nhảy - Step Response)
    if G_s is not None:
        plt.figure(figsize=(8, 4))
        T_step, Y_step = control.step_response(G_s)
        plt.plot(T_step, Y_step)
        plt.title("Đáp ứng bước nhảy của mô hình (Step Response)")
        plt.xlabel("Time (s)")
        plt.ylabel("Góc độ (deg)")
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Sử dụng: python sys_id_analysis.py <ten_file_csv>")
    else:
        analyze_system(sys.argv[1])
