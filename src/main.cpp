/**
 * ============================================================
 *  CHƯƠNG TRÌNH NHẬN DẠNG HỆ THỐNG — CHIRP + PRMS
 *  Platform : ESP32 (FreeRTOS)
 *  Phần cứng: ADS1115 (I2C) đọc góc α qua chiết áp WDD35D4
 *             H-bridge DC motor, điều khiển qua 2 chân PWM
 *
 *  PWM: Dùng lớp ESP32PWM (từ thư viện ESP32Servo)
 *       attachPin(pin, freq, bits) + writeScaled(0.0 – 1.0)
 *       IN1 (thuận): writeScaled(duty)  khi pwm_val > 0
 *       IN2 (nghịch): writeScaled(duty) khi pwm_val < 0
 *       Kênh không dùng → writeScaled(0.0)
 *
 *  Giao tiếp Serial (460800 baud):
 *    LỆNH ĐẾN (ASCII, từ PC):
 *      "w <f_start> <f_end> <amplitude> <offset> <duration>"
 *      "r <amplitude> <offset> <clock_ms> <duration> <seed>"
 *      "s"  "m <val>"  "p <val>"  "d"  "?"
 *
 *    DỮ LIỆU RA (Binary packet, 12 bytes):
 *      [0x00] 0xAA          sync byte 1
 *      [0x01] 0x55          sync byte 2
 *      [0x02-05] uint32_t   t_ms  (little-endian)
 *      [0x06-07] int16_t    pwm_pct × 100  (e.g. 5023 = 50.23%)
 *      [0x08-09] int16_t    alpha_deg × 100 (e.g. 2150 = 21.50°)
 *      [0x0A-0B] uint16_t   CRC-16/CCITT của bytes [2..9]
 *      → 12 bytes vs 21 bytes ASCII = 43% nhẹ hơn
 *
 *    COMMENT (ASCII, luôn bắt đầu bằng '#'):
 *      # STATUS / ERROR / SWEEP_COMPLETE v.v.
 *
 *  PRMS:
 *    PRMS = PRBS_sign × Uniform_random_amplitude (Roman et al.)
 *    Dùng 2 LFSR 15-bit độc lập (polynomial x^15 + x^14 + 1)
 *    Chu kỳ 32767 bước. clock_ms = 1000/(2.5×BW_Hz).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <ESP32Servo.h>

// ============================================================
//  BINARY PROTOCOL — DATA PACKET (12 bytes)
//  Phân biệt với comment ASCII bằng byte đầu tiên:
//    0xAA → binary data packet
//    '#'  → ASCII comment line (đến '\n')
// ============================================================
#define PKT_SYNC1        0xAA
#define PKT_SYNC2        0x55
#define PKT_PAYLOAD_LEN  8     // t_ms(4) + pwm_i16(2) + alpha_i16(2)
#define PKT_TOTAL_LEN    12    // sync(2) + payload(8) + crc16(2)

// CRC-16/CCITT  polynomial 0x1021, init 0xFFFF
// Tính trên 8 bytes payload [t_ms | pwm_raw | alpha_raw]
static uint16_t crc16_ccitt(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// Gửi 1 data packet nhị phân: 12 bytes, non-blocking (ghi vào TX HW buffer)
static void send_data_packet(uint32_t t_ms, float pwm_pct, float alpha_deg) {
    // Payload: t_ms(4) + pwm_raw(2) + alpha_raw(2) — little-endian
    uint8_t pay[PKT_PAYLOAD_LEN];
    pay[0] = (uint8_t)(t_ms);
    pay[1] = (uint8_t)(t_ms >>  8);
    pay[2] = (uint8_t)(t_ms >> 16);
    pay[3] = (uint8_t)(t_ms >> 24);
    int16_t pwm_raw   = (int16_t)(pwm_pct   * 100.0f);
    int16_t alpha_raw = (int16_t)(alpha_deg * 100.0f);
    pay[4] = (uint8_t)(pwm_raw);
    pay[5] = (uint8_t)(pwm_raw   >> 8);
    pay[6] = (uint8_t)(alpha_raw);
    pay[7] = (uint8_t)(alpha_raw >> 8);

    uint16_t crc = crc16_ccitt(pay, PKT_PAYLOAD_LEN);

    uint8_t pkt[PKT_TOTAL_LEN];
    pkt[0]  = PKT_SYNC1;
    pkt[1]  = PKT_SYNC2;
    memcpy(pkt + 2, pay, PKT_PAYLOAD_LEN);
    pkt[10] = (uint8_t)(crc);
    pkt[11] = (uint8_t)(crc >> 8);

    Serial.write(pkt, PKT_TOTAL_LEN);
}

// ============================================================
//  CẤU HÌNH PHẦN CỨNG — chỉnh sửa tại đây cho phù hợp
// ============================================================

// I2C cho ADS1115
#define PIN_SDA         26
#define PIN_SCL         25
 
// H-bridge: 2 kênh PWM độc lập
#define PIN_MOTOR_IN1   33
#define PIN_MOTOR_IN2   32

// Cài đặt PWM
#define PWM_FREQ_HZ     10000
#define PWM_BITS        10

// Chu kỳ lấy mẫu và điều khiển
#define SAMPLE_MS       2       // 500 Hz  (ADS1115 max = 860 SPS ✓)
#define SERIAL_BAUD     460800  // cần >= 230400 để không tràn buffer ở 500 Hz

// ============================================================
//  GIỚI HẠN AN TOÀN
// ============================================================
#define ALPHA_MIN_DEG   2.0f
#define ALPHA_MAX_DEG  42.0f
#define PWM_MAX_ABS     1.0f

// ============================================================
//  ĐỐI TƯỢNG TOÀN CỤC
// ============================================================
Adafruit_ADS1115 ads;

ESP32PWM pwmIN1;
ESP32PWM pwmIN2;

// Máy trạng thái thí nghiệm
enum SweepState {
    STATE_IDLE,         // đứng chờ lệnh
    STATE_BASELINE,     // 1 giây đứng yên (dùng chung cho chirp và PRMS)
    STATE_SWEEP,        // đang phát chirp logarithmic
    STATE_PRMS,         // đang phát PRMS
    STATE_DONE          // hoàn thành, tắt motor
};

// Tham số chirp — được set qua lệnh 'w'
struct SweepParams {
    float f_start   = 0.05f;
    float f_end     = 5.0f;
    float amplitude = 0.25f;
    float offset    = 0.0f;
    float duration  = 90.0f;
};

// Bảng LFSR maximal-length (2 taps): x^tap_a + x^tap_b + 1
// n  tap_a tap_b  N=2^n-1   Duration @ 4s/step
//  7    7    6      127        508s  (~8.5 min)   ← thực tế BW=0.1Hz
//  9    9    5      511       2044s  (~34 min)
// 10   10    7     1023       4092s  (~68 min)    ← bài báo Roman et al.
// 15   15   14    32767      131068s (36 giờ)
struct LFSRConfig {
    uint8_t  n_bits;
    uint8_t  tap_a;
    uint8_t  tap_b;
    uint16_t mask;    // (1<<n)-1
    uint16_t period;  // N = mask
};
static const LFSRConfig LFSR_TABLE[] = {
    {  7,  7,  6, 0x007F,   127 },
    {  9,  9,  5, 0x01FF,   511 },
    { 10, 10,  7, 0x03FF,  1023 },
    { 15, 15, 14, 0x7FFF, 32767 },
};
static const uint8_t LFSR_N_CONFIGS = 4;

// Runtime LFSR config (được đặt khi nhận lệnh 'r')
static uint8_t  g_lfsr_tap_a  = 7;
static uint8_t  g_lfsr_tap_b  = 6;
static uint16_t g_lfsr_mask   = 0x007F;
static uint16_t g_lfsr_period = 127;   // N = số bước 1 chu kỳ đầy đủ

// Tham số PRMS
// Duration KHÔNG cần truyền — tự tính = N × clock_ms/1000
// → luôn chạy đúng 1 chu kỳ PRBS đầy đủ (flat spectrum)
struct PRMSParams {
    float    amplitude = 0.4f;
    float    offset    = 0.0f;
    uint16_t clock_ms  = 4000;  // ms/bước = 1000/(2.5×BW_Hz)
    uint8_t  n_bits    = 7;     // PRBS bits: 7, 9, 10, 15
    uint16_t seed      = 1;     // seed LFSR_PRBS [1..period]
};

volatile SweepState  sweepState   = STATE_IDLE;
volatile SweepState  g_nextState  = STATE_SWEEP; // trạng thái sau baseline

SweepParams          params;
PRMSParams           prmsParams;

unsigned long        phaseStartMs  = 0;

// PRMS runtime state — 2 LFSR độc lập (theo bài báo Roman et al.)
//   g_lfsrPRBS : sinh chuỗi nhị phân ±1 (bit 0 = dấu)
//   g_lfsrAmp  : sinh biên độ ngẫu nhiên đều trong [0, A]
//   Output     = sign × (g_lfsrAmp / 32767) × A  ∈ [-A, +A]
volatile uint16_t    g_lfsrPRBS     = 1;
volatile uint16_t    g_lfsrAmp      = 1;
volatile int         g_prmsLastTick = -1;
volatile float       g_prmsOutput   = 0.0f;

// PWM thủ công từ slider GUI
volatile float g_manualPWM = 0.0f;

// Queue mailbox cho dữ liệu cảm biến (size=1)
QueueHandle_t qAlpha;

// ============================================================
//  HÀM ĐỌC CẢM BIẾN GÓC ALPHA
// ============================================================
float readAlpha() {
    int16_t adc = ads.getLastConversionResults();
  // Serial.println(liftingsensor.computeVolts(adc), 5);
  return 84.22851563f * ads.computeVolts(adc);

}

// =========================================================
//  HÀM XUẤT PWM RA H-BRIDGE
// ============================================================
void sendPWM(float pwm_val) {
    pwm_val = constrain(pwm_val, -PWM_MAX_ABS, PWM_MAX_ABS);
    if (pwm_val > 0.0f) {
        pwmIN1.writeScaled(pwm_val);
        pwmIN2.writeScaled(0.0f);
    } else if (pwm_val < 0.0f) {
        pwmIN1.writeScaled(0.0f);
        pwmIN2.writeScaled(-pwm_val);
    } else {
        pwmIN1.writeScaled(0.0f);
        pwmIN2.writeScaled(0.0f);
    }
}

// ============================================================
//  HÀM SINH LOGARITHMIC CHIRP
// ============================================================
float logChirp(float t, float f1, float f2, float T, float amplitude) {
    if (t < 0.0f || t >= T) return 0.0f;
    float log_ratio = logf(f2 / f1);
    float phase = 2.0f * M_PI * (f1 * T / log_ratio) * (powf(f2 / f1, t / T) - 1.0f);
    return amplitude * sinf(phase);
}

// ============================================================
//  HÀM LFSR N-BIT — dùng runtime config từ LFSR_TABLE
//  Polynomial: x^tap_a + x^tap_b + 1
//  Gọi sau khi set g_lfsr_tap_a, g_lfsr_tap_b, g_lfsr_mask
// ============================================================
uint16_t lfsr_step(uint16_t state) {
    uint16_t bit = ((state >> (g_lfsr_tap_a - 1)) ^
                    (state >> (g_lfsr_tap_b - 1))) & 1;
    state = ((state << 1) | bit) & g_lfsr_mask;
    if (state == 0) state = 1;
    return state;
}

// Tìm và áp dụng cấu hình LFSR theo n_bits; trả về false nếu n_bits không hợp lệ
static bool apply_lfsr_config(uint8_t n_bits) {
    for (uint8_t i = 0; i < LFSR_N_CONFIGS; i++) {
        if (LFSR_TABLE[i].n_bits == n_bits) {
            g_lfsr_tap_a  = LFSR_TABLE[i].tap_a;
            g_lfsr_tap_b  = LFSR_TABLE[i].tap_b;
            g_lfsr_mask   = LFSR_TABLE[i].mask;
            g_lfsr_period = LFSR_TABLE[i].period;
            return true;
        }
    }
    return false;
}

// ============================================================
//  SINH GIÁ TRỊ PRMS THEO BÀI BÁO (Roman et al.)
//  PRMS = PRBS_sign × Uniform_random_amplitude
//
//  lfsrPRBS : bit 0 → dấu (+1 hoặc -1)
//  lfsrAmp  : normalize về [0,1] → nhân với amplitude
//  Kết quả  : phân bố đều trong [-amplitude, +amplitude]
// ============================================================
float prms_sample(uint16_t lfsrPRBS, uint16_t lfsrAmp, float amplitude) {
    float sign = (lfsrPRBS & 1) ? 1.0f : -1.0f;
    // Normalize theo period thực tế (không phải 32767 cứng)
    // → phân bố đều trong (0, 1] cho mọi n_bits
    float uni  = (float)lfsrAmp / (float)g_lfsr_period;
    return sign * uni * amplitude;   // ∈ [-A, +A]
}

// ============================================================
//  TASK ĐỌC CẢM BIẾN — ưu tiên cao, 20ms
// ============================================================
void TaskSensor(void *pvParameters) {
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        float alpha = 266.6f - readAlpha();
        alpha = constrain(alpha,0.0f, 50.0f);
        // float alpha = readAlpha();
        // Serial.println(alpha);
        xQueueOverwrite(qAlpha, &alpha);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_MS));
    }
}

// ============================================================
//  TASK ĐIỀU KHIỂN + LOG — 20ms
// ============================================================
void TaskSweep(void *pvParameters) {
    TickType_t xLastWake = xTaskGetTickCount();
    float alpha = 0.0f;

    for (;;) {
        xQueuePeek(qAlpha, &alpha, 0);

        unsigned long now_ms = millis();
        float pwm_out = 0.0f;

        switch (sweepState) {

            // ── IDLE: điều khiển thủ công từ GUI ────────────
            case STATE_IDLE:
                sendPWM(g_manualPWM);
                break;

            // ── BASELINE: 1s đứng yên trước khi phát tín hiệu
            case STATE_BASELINE:
            {
                pwm_out = 0.0f;
                sendPWM(pwm_out);
                send_data_packet(now_ms, pwm_out * 100.0f, alpha);
                if (now_ms - phaseStartMs >= 1000) {
                    phaseStartMs = now_ms;
                    sweepState   = g_nextState; // chuyển sang CHIRP hoặc PRMS

                    if (g_nextState == STATE_SWEEP) {
                        Serial.println("# CHIRP_START");
                    } else {
                        // Khởi tạo 2 LFSR độc lập
                        // LFSR_PRBS: dùng seed gốc
                        g_lfsrPRBS = prmsParams.seed;
                        // LFSR_Amp : XOR 0x55AA để đảm bảo độc lập thống kê
                        g_lfsrAmp  = (uint16_t)(prmsParams.seed ^ 0x55AA) & 0x7FFF;
                        if (g_lfsrAmp == 0) g_lfsrAmp = 1;
                        g_prmsLastTick = -1;
                        g_prmsOutput   = prms_sample(g_lfsrPRBS, g_lfsrAmp,
                                                     prmsParams.amplitude);
                        Serial.println("# PRMS_START");
                    }
                }
                break;
            }

            // ── SWEEP: logarithmic chirp ─────────────────────
            case STATE_SWEEP:
            {
                float t = (float)(now_ms - phaseStartMs) / 1000.0f;

                if (t >= params.duration) {
                    sweepState = STATE_DONE;
                    sendPWM(0.0f);
                    Serial.println("# CHIRP_END");
                    break;
                }

                float chirp_val = logChirp(t, params.f_start, params.f_end,
                                           params.duration, params.amplitude);
                pwm_out = params.offset + chirp_val;

                // Kiểm tra an toàn góc
                bool nearMin = (alpha < ALPHA_MIN_DEG + 1.0f) && (pwm_out < 0.0f);
                bool nearMax = (alpha > ALPHA_MAX_DEG - 1.0f) && (pwm_out > 0.0f);
                if (nearMin || nearMax) {
                    pwm_out = 0.0f;
                    Serial.printf("# CLAMP t=%.2f alpha=%.2f\n", t, alpha);
                }

                sendPWM(pwm_out);
                send_data_packet(now_ms, pwm_out, alpha);
                break;
            }

            // ── PRMS: Pseudo-Random Multi-level Sequence ─────
            case STATE_PRMS:
            {
                float t    = (float)(now_ms - phaseStartMs) / 1000.0f;
                // Đếm bước theo clock_ms — kết thúc sau đúng N = g_lfsr_period bước
                int   tick = (int)(t * 1000.0f / (float)prmsParams.clock_ms);

                if (tick >= (int)g_lfsr_period) {
                    sweepState = STATE_DONE;
                    sendPWM(0.0f);
                    Serial.println("# PRMS_END");
                    break;
                }

                // Khi sang tick mới → bước cả 2 LFSR
                // PRMS = PRBS_sign × Uniform_random_amplitude  (Roman et al.)
                if (tick != g_prmsLastTick) {
                    g_prmsLastTick = tick;
                    g_lfsrPRBS     = lfsr_step(g_lfsrPRBS);
                    g_lfsrAmp      = lfsr_step(g_lfsrAmp);
                    // normalize biên độ theo period (không phải 32767 cố định)
                    g_prmsOutput   = prms_sample(g_lfsrPRBS, g_lfsrAmp,
                                                 prmsParams.amplitude);
                }

                pwm_out = prmsParams.offset + g_prmsOutput;

                sendPWM(pwm_out);
                send_data_packet(now_ms, pwm_out, alpha);
                break;
            }

            // ── DONE: hoàn thành, in thông số rồi về IDLE ───
            case STATE_DONE:
                sendPWM(0.0f);
                Serial.println("# SWEEP_COMPLETE");
                if (g_nextState == STATE_SWEEP) {
                    Serial.printf("# [CHIRP] f=%.3f-%.3fHz  A=%.2f  off=%.2f  T=%.0fs\n",
                                   params.f_start, params.f_end,
                                   params.amplitude, params.offset, params.duration);
                } else {
                    Serial.printf("# [PRMS]  %d-bit  N=%d bước  clk=%dms  T=%.0fs  A=%.2f  off=%.2f  seed=%d\n",
                                   prmsParams.n_bits, g_lfsr_period, prmsParams.clock_ms,
                                   (float)g_lfsr_period * prmsParams.clock_ms / 1000.0f,
                                   prmsParams.amplitude, prmsParams.offset, prmsParams.seed);
                }
                sweepState = STATE_IDLE;
                break;
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(SAMPLE_MS));
    }
}

// ============================================================
//  TASK XỬ LÝ LỆNH SERIAL
// ============================================================
void TaskSerial(void *pvParameters) {
    Serial.setTimeout(50);
    float pwm_manual = 0.0f;
    for (;;) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            if (input.length() == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            char cmd = input.charAt(0);
            float dur;
            // ── LỆNH w: bắt đầu chirp sweep ─────────────────
            if (cmd == 'w') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: dang chay, gui 's' de dung truoc");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                float f1, f2, amp, off;
                int parsed = sscanf(input.c_str(), "w %f %f %f %f %f",
                                    &f1, &f2, &amp, &off, &dur);

                if (parsed != 5) {
                    Serial.println("# ERR: cu phap sai");
                    Serial.println("# Vi du: w 0.05 5.0 0.25 0.0 90");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (f1 <= 0.0f || f2 <= f1) {
                    Serial.println("# ERR: f_start phai > 0 va < f_end");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (amp <= 0.0f || amp > PWM_MAX_ABS) {
                    Serial.printf("# ERR: amplitude phai trong khoang (0, %.2f]\n", PWM_MAX_ABS);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (dur < 5.0f || dur > 300.0f) {
                    Serial.println("# ERR: duration phai trong khoang [5, 300] giay");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                float min_cycles = f1 * dur;
                if (min_cycles < 3.0f) {
                    Serial.printf("# WARN: f_start=%.3fHz chi co %.1f chu ky trong %.0fs\n",
                                   f1, min_cycles, dur);
                    Serial.println("# Nen tang duration hoac tang f_start");
                }

                params.f_start   = f1;
                params.f_end     = f2;
                params.amplitude = amp;
                params.offset    = off;
                params.duration  = dur;

                g_nextState  = STATE_SWEEP;
                phaseStartMs = millis();
                sweepState   = STATE_BASELINE;

                Serial.println("# ============================================");
                Serial.printf("# CHIRP SWEEP: f=%.4f->%.4fHz  A=%.3f  off=%.3f  T=%.0fs\n",
                               f1, f2, amp, off, dur);
                Serial.printf("# Sample rate: %d Hz  |  Baud: %d\n", 1000/SAMPLE_MS, SERIAL_BAUD);
                Serial.println("# t_ms,pwm_pct,alpha_deg");
                Serial.println("# ============================================");
                Serial.println("# BASELINE_START");
            }

            // ── LỆNH r: bắt đầu PRMS ────────────────────────
            // Cú pháp: r <amplitude> <offset> <clock_ms> <n_bits> <seed>
            // Ví dụ:   r 0.4 0.0 4000 7 1
            //   n_bits chọn theo thời gian: 7→127 bước(8.5min), 9→511(34min), 10→1023(68min)
            //   duration tự tính = N × clock_ms/1000 (đúng 1 chu kỳ PRBS)
            //   clock_ms = 1000 / (2.5 × BW_Hz)  →  BW=0.1Hz → 4000ms
            else if (cmd == 'r') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: dang chay, gui 's' de dung truoc");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                float amp, off;
                int   clk_ms, nb;
                unsigned int seed_val;
                int parsed = sscanf(input.c_str(), "r %f %f %d %d %u",
                                    &amp, &off, &clk_ms, &nb, &seed_val);

                if (parsed != 5) {
                    Serial.println("# ERR: cu phap sai");
                    Serial.println("# Vi du: r 0.4 0.0 4000 7 1");
                    Serial.println("#        amplitude offset clock_ms n_bits seed");
                    Serial.println("# n_bits: 7(127bước,8.5min) 9(511,34min) 10(1023,68min)");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (amp <= 0.0f || amp > PWM_MAX_ABS) {
                    Serial.printf("# ERR: amplitude phai trong khoang (0, %.2f]\n", PWM_MAX_ABS);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (clk_ms < (int)SAMPLE_MS) {
                    Serial.printf("# ERR: clock_ms phai >= %d ms\n", SAMPLE_MS);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (!apply_lfsr_config((uint8_t)nb)) {
                    Serial.println("# ERR: n_bits phai la 7, 9, 10, hoac 15");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (seed_val == 0 || seed_val > (unsigned int)g_lfsr_period) {
                    Serial.printf("# ERR: seed phai trong khoang [1, %d]\n", g_lfsr_period);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                prmsParams.amplitude = amp;
                prmsParams.offset    = off;
                prmsParams.clock_ms  = (uint16_t)clk_ms;
                prmsParams.n_bits    = (uint8_t)nb;
                prmsParams.seed      = (uint16_t)seed_val;

                // Seed LFSR_Amp = seed XOR 0x55AA (độc lập thống kê), masked theo period
                uint16_t amp_seed = (uint16_t)(seed_val ^ 0x55AA) & g_lfsr_mask;
                if (amp_seed == 0) amp_seed = 1;

                g_nextState  = STATE_PRMS;
                phaseStartMs = millis();
                sweepState   = STATE_BASELINE;

                Serial.println("# ============================================");
                Serial.printf("# PRMS (PRBS × Uniform Amp): A=%.3f  off=%.3f  clk=%dms  T=%.0fs  seed=%d\n",
                               amp, off, clk_ms, dur, seed_val);
                Serial.printf("# f_c=%.4fHz  f_upper=f_c/3=%.4fHz  f_lower=f_c/N\n",
                               1000.0f / clk_ms, 1000.0f / clk_ms / 3.0f);
                Serial.printf("# LFSR_PRBS seed=%d  LFSR_Amp seed=%d (auto)\n",
                               seed_val, amp_seed);
                Serial.printf("# Sample rate: %d Hz  |  Baud: %d\n",
                               1000 / SAMPLE_MS, SERIAL_BAUD);
                Serial.println("# t_ms,pwm_pct,alpha_deg");
                Serial.println("# ============================================");
                Serial.println("# BASELINE_START");
            }

            // ── LỆNH s: dừng khẩn cấp ────────────────────────
            else if (cmd == 's') {
                g_manualPWM = 0.0f;
                sendPWM(0.0f);
                sweepState = STATE_IDLE;
                Serial.println("# STOPPED");
            }

            // ── LỆNH ?: xem trạng thái ───────────────────────
            else if (cmd == '?') {
                float alpha = 0.0f;
                xQueuePeek(qAlpha, &alpha, 0);
                const char* stateNames[] = { "IDLE", "BASELINE", "CHIRP_SWEEP", "PRMS", "DONE" };
                Serial.printf("# State : %s\n", stateNames[(int)sweepState]);
                Serial.printf("# Alpha : %.3f deg\n", alpha);
                Serial.printf("# Uptime: %lu ms\n", millis());

                if (sweepState == STATE_SWEEP) {
                    float elapsed = (float)(millis() - phaseStartMs) / 1000.0f;
                    float f_now = params.f_start * powf(params.f_end / params.f_start,
                                                         elapsed / params.duration);
                    Serial.printf("# [CHIRP] Progress: %.1f%%  f_now=%.3fHz\n",
                                   elapsed / params.duration * 100.0f, f_now);
                } else if (sweepState == STATE_PRMS) {
                    float elapsed = (float)(millis() - phaseStartMs) / 1000.0f;
                    int tick = (int)(elapsed * 1000.0f / prmsParams.clock_ms);
                    Serial.printf("# [PRMS]  %d-bit  tick=%d/%d (%.1f%%)  pwm=%.3f\n",
                                   prmsParams.n_bits, tick, g_lfsr_period,
                                   (float)tick / g_lfsr_period * 100.0f, g_prmsOutput);
                }
            }

            // ── LỆNH d: đọc alpha ngay (debug) ───────────────
            else if (cmd == 'd') {
                float alpha = 0.0f;
                xQueuePeek(qAlpha, &alpha, 0);
                Serial.printf("# Alpha = %.4f deg\n", alpha);
            }

            // ── LỆNH p: test PWM 200ms rồi tự tắt ───────────
            else if (cmd == 'p') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: chi dung khi IDLE");
                } else {
                    float pwm_test = 0.0f;
                    if (sscanf(input.c_str(), "p %f", &pwm_test) == 1) {
                        pwm_test = constrain(pwm_test, -PWM_MAX_ABS, PWM_MAX_ABS);
                        sendPWM(pwm_test);
                        float alpha = 0.0f;
                        xQueuePeek(qAlpha, &alpha, 0);
                        Serial.printf("# PWM=%.3f  Alpha=%.4f\n", pwm_test, alpha);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        sendPWM(0.0f);
                    } else {
                        Serial.println("# Vi du: p 0.15  (test PWM 15%, tu tat sau 200ms)");
                    }
                }
            }

            // ── LỆNH m: PWM thủ công từ slider GUI ───────────
            else if (cmd == 'm') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: chi dung khi IDLE (dung sweep truoc)");
                } else {
                    if (sscanf(input.c_str(), "m %f", &pwm_manual) == 1) {
                        pwm_manual = constrain(pwm_manual, -PWM_MAX_ABS, PWM_MAX_ABS);
                        g_manualPWM = pwm_manual;
                        const char* dir = (pwm_manual >  0.01f) ? "THUAN" :
                                          (pwm_manual < -0.01f) ? "NGHICH" : "DUNG";
                        Serial.printf("# MANUAL PWM=%.3f (%s)\n", pwm_manual, dir);
                    } else {
                        Serial.println("# Vi du: m 0.30  (quay thuan 30%)");
                        Serial.println("# Vi du: m -0.30 (quay nghich 30%)");
                    }
                }
            }

            else {
                Serial.println("# Lenh: w(chirp) r(prms) s(stop) ?(status) d(alpha) p(pwm_test) m(manual_pwm)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    Serial.println("# ============================================");
    Serial.printf ("# SysID Logger v2.0  [Chirp + PRMS]  [ESP32PWM]\n");
    Serial.printf ("# Sample rate: %d Hz  |  Baud: %d\n", 1000/SAMPLE_MS, SERIAL_BAUD);
    Serial.println("# ============================================");

    // Khởi tạo I2C và ADS1115
    Wire.begin(PIN_SDA, PIN_SCL);
    delay(500);
    if (!ads.begin()) {
        Serial.println("# FATAL: Khong tim thay ADS1115!");
        while (1) vTaskDelay(pdMS_TO_TICKS(100));
    }
    ads.setGain(GAIN_ONE);
    
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, true);
    Serial.println("# ADS1115 OK");

    // Khởi tạo PWM
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    pwmIN1.attachPin(PIN_MOTOR_IN1, PWM_FREQ_HZ, PWM_BITS);
    pwmIN2.attachPin(PIN_MOTOR_IN2, PWM_FREQ_HZ, PWM_BITS);
    sendPWM(0.0f);
    Serial.printf("# Motor driver OK  [IN1=GPIO%d, IN2=GPIO%d, %dHz, %d-bit]\n",
                   PIN_MOTOR_IN1, PIN_MOTOR_IN2, PWM_FREQ_HZ, PWM_BITS);

    // Tạo queue mailbox cho alpha (size=1)
    qAlpha = xQueueCreate(1, sizeof(float));
    float initAlpha = readAlpha();
    xQueueOverwrite(qAlpha, &initAlpha);
    Serial.printf("# Alpha khoi dong: %.3f deg\n", initAlpha);

    if (initAlpha < ALPHA_MIN_DEG || initAlpha > ALPHA_MAX_DEG) {
        Serial.printf("# WARN: Alpha (%.2f) ngoai vung an toan [%.1f, %.1f]\n",
                       initAlpha, ALPHA_MIN_DEG, ALPHA_MAX_DEG);
    }
    delay(2000);

    // Tạo tasks
    xTaskCreate(TaskSensor, "Sensor", 2048, NULL, 4, NULL);
    xTaskCreate(TaskSweep,  "Sweep",  4096, NULL, 3, NULL);
    xTaskCreate(TaskSerial, "Serial", 4096, NULL, 1, NULL);

    Serial.println("# San sang. Gui '?' de xem trang thai.");
    Serial.println("# Vi du chirp: w 0.05 5.0 0.25 0.0 90");
    Serial.println("# Vi du PRMS:  r 0.5 0.0 100 60 3 1");
}

void loop() {
    vTaskDelete(NULL);
}
