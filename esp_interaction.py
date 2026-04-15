import serial
import time
import csv
import argparse
import threading
import sys
import matplotlib.pyplot as plt
import pandas as pd
#python esp_interaction.py --port COM3

# Biến toàn cục để điều khiển luồng đọc
stop_thread = False

def read_from_serial(ser, csv_writer, csv_file):
    """Luồng đọc dữ liệu từ ESP32 và ghi vào file CSV"""
    global stop_thread
    print("# Bắt đầu luồng đọc dữ liệu...")
    while not stop_thread:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # In ra console để theo dõi
                print(f"\r[ESP32]: {line}")
                print("Command >> ", end="", flush=True)

                # Lưu vào CSV nếu là dòng dữ liệu (không bắt đầu bằng #)
                if not line.startswith('#'):
                    parts = line.split(',')
                    if len(parts) == 3:
                        csv_writer.writerow(parts)
                        csv_file.flush()
                
                # Nếu nhận được marker kết thúc từ ESP32
                if "# SWEEP_COMPLETE" in line:
                    print("\n[INFO]: Thí nghiệm kết thúc.")
            else:
                time.sleep(0.01) # Tránh chiếm dụng CPU
        except Exception as e:
            print(f"\n[Lỗi luồng đọc]: {e}")
            break

def plot_data(filename):
    """Vẽ đồ thị từ file CSV đã lưu"""
    try:
        data = pd.read_csv(filename, names=['t_ms', 'pwm_pct', 'alpha_deg'])
        data['t_sec'] = (data['t_ms'] - data['t_ms'].iloc[0]) / 1000.0
        
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
        
        ax1.plot(data['t_sec'], data['pwm_pct'], 'r-', label='PWM (%)')
        ax1.set_ylabel('PWM (%)')
        ax1.grid(True)
        ax1.legend()
        
        ax2.plot(data['t_sec'], data['alpha_deg'], 'b-', label='Alpha (deg)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Alpha (deg)')
        ax2.grid(True)
        ax2.legend()
        
        plt.suptitle(f"Experimental Data: {filename}")
        plt.show()
    except Exception as e:
        print(f"Không thể vẽ đồ thị: {e}")

def main():
    parser = argparse.ArgumentParser(description="Giao tiếp với ESP32 Logarithmic Chirp Logger")
    parser.add_argument("--port", type=str, required=True, help="Cổng Serial (ví dụ: COM3 hoặc /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (mặc định 115200)")
    parser.add_argument("--output", type=str, default="log_chirp_data.csv", help="Tên file CSV lưu kết quả")
    
    args = parser.parse_args()

    try:
        # Khởi tạo Serial
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2)  # Chờ ESP32 khởi động lại
        ser.reset_input_buffer()
        
        print(f"--- Đã kết nối với {args.port} ---")
        print("Lệnh có sẵn:")
        print("  w f1 f2 amp off dur : Chạy Sweep (vd: w 0.05 5.0 0.25 0.0 60)")
        print("  s                   : Dừng khẩn cấp")
        print("  ?                   : Xem trạng thái")
        print("  q                   : Thoát và vẽ đồ thị")

        with open(args.output, mode='w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            
            # Chạy luồng đọc
            reader_thread = threading.Thread(target=read_from_serial, args=(ser, writer, csv_file))
            reader_thread.daemon = True
            reader_thread.start()

            while True:
                cmd = input("Command >> ").strip()
                
                if cmd.lower() == 'q':
                    global stop_thread
                    stop_thread = True
                    break
                
                if cmd:
                    ser.write((cmd + '\n').encode('utf-8'))

        print("\nĐang đóng kết nối...")
        time.sleep(1)
        
        # Hỏi xem có muốn vẽ đồ thị không
        ans = input("Bạn có muốn vẽ đồ thị dữ liệu vừa thu thập không? (y/n): ")
        if ans.lower() == 'y':
            plot_data(args.output)

    except serial.SerialException as e:
        print(f"Lỗi Serial: {e}")
    except KeyboardInterrupt:
        print("\nĐã dừng.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("--- Hoàn tất ---")

if __name__ == "__main__":
    main()
