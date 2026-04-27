# CLAUDE.md — System Identification Project

## Tổng quan

Hệ thống nhận dạng (system identification) cho động cơ DC dùng ESP32.
Hai tín hiệu kích thích: **Logarithmic Chirp** và **PRMS** (Pseudo-Random Multi-level Sequence theo Roman et al. 2021).
GUI Python (PyQt5 + pyqtgraph) điều khiển ESP32 qua Serial và ghi dữ liệu ra CSV.

---

## Phần cứng

| Thành phần | Chi tiết |
|---|---|
| Vi điều khiển | ESP32 DevKit v1 (FreeRTOS) |
| Cảm biến góc | Chiết áp WDD35D4 → ADS1115 (I2C) |
| I2C | SDA=GPIO14, SCL=GPIO13 |
| H-Bridge | IN1=GPIO33 (thuận), IN2=GPIO32 (nghịch) |
| PWM | 10kHz, 10-bit, thư viện ESP32Servo |
| Hành trình | α ∈ [2°, 42°], tâm 22°, `alpha = 155 - readAlpha()` |
| Hệ số đo góc | `α_raw = 84.22851 × V_ads` |
| Băng thông hệ thống | ~0.1 Hz |

---

## Cấu trúc file

```
src/main.cpp      — Firmware ESP32 (FreeRTOS, 3 tasks)
esp_gui.py        — GUI Python (PyQt5 + pyqtgraph)
platformio.ini    — PlatformIO config (board: esp32doit-devkit-v1)
sys_id_analysis.py — Phân tích dữ liệu sau thí nghiệm
sys_id_sopdt.py   — Fit mô hình SOPDT
sweep_data_*.csv  — Dữ liệu ghi được (chirp/PRMS)
```

---

## Firmware — `src/main.cpp`

### Hằng số quan trọng

```cpp
#define SAMPLE_MS       2        // 500 Hz
#define SERIAL_BAUD     460800
#define ALPHA_MIN_DEG   2.0f
#define ALPHA_MAX_DEG   42.0f
#define PWM_MAX_ABS     0.85f
```

### FreeRTOS Tasks

| Task | Priority | Chu kỳ | Vai trò |
|---|---|---|---|
| `TaskSensor` | 4 | 2 ms | Đọc ADS1115 → Queue `qAlpha` |
| `TaskSweep` | 3 | 2 ms | Tính PWM, gửi motor, phát packet |
| `TaskSerial` | 1 | 100 ms | Đọc lệnh ASCII từ PC (RX only) |

> **Lưu ý:** CSV data đi ra từ `TaskSweep` (TX, 500 Hz). `TaskSerial` chỉ đọc lệnh ĐẾN.

### State Machine

```
STATE_IDLE → STATE_BASELINE (1s) → STATE_SWEEP (Chirp)
                                 → STATE_PRMS  (PRMS)
                               → STATE_DONE → STATE_IDLE
```

Biến `g_nextState` quyết định sau Baseline đi vào Chirp hay PRMS.

### Giao thức Serial

**Lệnh ĐẾN (ASCII từ PC → ESP32):**

| Lệnh | Mô tả |
|---|---|
| `w f1 f2 amp off dur` | Bắt đầu Logarithmic Chirp |
| `r amp off clk_ms n_bits seed` | Bắt đầu PRMS |
| `s` | Emergency stop |
| `m val` | PWM thủ công (-0.85..+0.85) |
| `p val` | Test PWM 200ms rồi tắt |
| `d` | Đọc alpha ngay |
| `?` | Xem trạng thái |

**Dữ liệu RA (Binary packet, 12 bytes):**

```
Byte  0    : 0xAA  (sync 1)
Byte  1    : 0x55  (sync 2)
Byte  2-5  : uint32_t  t_ms       (little-endian)
Byte  6-7  : int16_t   pwm×100    (e.g. 5023 = 50.23%)
Byte  8-9  : int16_t   alpha×100  (e.g. 2150 = 21.50°)
Byte 10-11 : uint16_t  CRC-16/CCITT của bytes [2..9]
```

Comment ASCII (luôn bắt đầu `#`) vẫn gửi xen kẽ và kết thúc bằng `\n`.

### PRMS — Tham số và thuật toán

Theo bài báo: **Roman et al., Engineering Research Journal 172, 2021** (Electro-hydraulic SBW system identification + SMC).

```
PRMS = PRBS_sign × Uniform_random_amplitude
```

- **2 LFSR độc lập** (polynomial x^tap_a + x^tap_b + 1):
  - `g_lfsrPRBS`: bit 0 → dấu ±1
  - `g_lfsrAmp` (seed = seed_PRBS XOR 0x55AA): normalize → biên độ [0, A]
- **Duration tự tính** = N × clock_ms/1000 (đúng 1 chu kỳ PRBS → phổ phẳng)

**Bảng LFSR (LFSR_TABLE):**

| n_bits | Polynomial | N bước | @4s/step | Ghi chú |
|---|---|---|---|---|
| 7 | x^7+x^6+1 | 127 | 508 s | Khuyến nghị BW=0.1Hz |
| 9 | x^9+x^5+1 | 511 | 2044 s | |
| 10 | x^10+x^7+1 | 1023 | 4092 s | Bài báo (clock=30ms→31s) |
| 15 | x^15+x^14+1 | 32767 | 36 giờ | |

**Thiết kế tham số (BW = 0.1 Hz):**
- `clock_ms = 1000 / (2.5 × BW_Hz) = 4000 ms`
- `f_upper = f_c/3 = 0.083 Hz` (83% băng thông)
- Lệnh: `r 0.4 0.0 4000 7 1`


## GUI — `esp_gui.py`

### Classes

**`SerialReader(QObject)`** — chạy trong thread riêng

- Đọc raw bytes (không dùng `readline`)
- State machine binary parser: `_ST_SYNC1 → _ST_SYNC2 → _ST_PAYLOAD → validate`
- Comment ASCII (`#`) xử lý song song
- Resync khi CRC fail: quét buffer tìm `0xAA` tiếp theo
- `stats()` → thống kê packet OK/ERR
- Baud rate mặc định: **460800**

**`RealTimeGui(QMainWindow)`**

- Signal type selector: Chirp | PRMS
- PRMS panel: clock_ms, n_bits (7/9/10 bit), duration label (tự tính), seed
- Chirp panel: f_start, f_end
- Shared: amplitude, offset
- Đồ thị real-time: PWM(%) và Alpha(°) — pyqtgraph, 30ms refresh, buffer 2500 điểm
- CSV: tên file tự động có tag `_chirp_` hoặc `_prms_` + timestamp
- Log console: tô màu theo loại tin nhắn (lỗi đỏ, cảnh báo cam, event xanh)

### CRC-16/CCITT (Python)

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc
```

Phải khớp hoàn toàn với `crc16_ccitt()` trong `main.cpp`.

---

## Lệnh PRMS nhanh

```bash
# BW = 0.1 Hz, 7-bit PRBS (127 bước, ~8.5 phút)
r 0.4 0.0 4000 7 1

# Thay đổi seed để lấy chuỗi khác (cùng tính chất phổ)
r 0.4 0.0 4000 7 42

# Biên độ nhỏ hơn nếu hay bị CPC can thiệp
r 0.2 0.0 4000 7 1
```

## Lệnh Chirp nhanh

```bash
# f: 0.05→5 Hz, amp=0.25, offset=0, duration=90s
w 0.05 5.0 0.25 0.0 90
```

---

## Lưu ý khi phát triển tiếp

- **Null bytes:** Windows đôi khi inject null bytes vào file UTF-8. Trước khi edit dùng: `python3 -c "open(f,'wb').write(open(f,'rb').read().replace(b'\x00',b''))"`
- **monitor_speed trong platformio.ini** vẫn là 115200 — đây là baud của Serial Monitor, KHÔNG ảnh hưởng firmware (firmware dùng `SERIAL_BAUD = 460800`). Nên cập nhật thành 460800 nếu dùng PlatformIO monitor.
- **ADS1115** chạy ở `RATE_ADS1115_860SPS` continuous mode, `GAIN_ONE` (±4.096V). Hệ số đo: `alpha_raw = 84.22851 × V`. Giá trị thực: `alpha = 155 - alpha_raw`.
