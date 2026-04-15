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
 *  Giao tiếp Serial (115200 baud):
 *    "w <f_start> <f_end> <amplitude> <offset> <duration>"
 *                       — bắt đầu logarithmic chirp sweep
 *    "r <amplitude> <offset> <clock_ms> <duration> <num_levels> <seed>"
 *                       — bắt đầu PRMS (Pseudo-Random Multi-level Sequence)
 *    "s"          — dừng khẩn cấp
 *    "m <val>"    — PWM thủ công (-0.85 đến +0.85)
 *    "p <val>"    — test PWM 200ms rồi tự tắt
 *    "d"          — đọc alpha ngay
 *    "?"          — xem trạng thái
 *
 *  Output Serial (CSV):
 *    # comment lines bắt đầu bằng '#'
 *    t_ms, pwm_pct, alpha_deg
 *
 *  PRMS:
 *    Dùng LFSR 15-bit (polynomial x^15 + x^14 + 1) để sinh
 *    chuỗi giả ngẫu nhiên độ dài 32767 bước, sau đó ánh xạ
 *    sang num_levels mức đều nhau trong [-amplitude, +amplitude].
 *    Mỗi bước giữ nguyên giá trị trong clock_ms milli-giây.
 *    Ví dụ: 3 mức = {-A, 0, +A}; 5 mức = {-A, -A/2, 0, A/2, +A}.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <ESP32Servo.h>

// ============================================================
//  CẤU HÌNH PHẦN CỨNG — chỉnh sửa tại đây cho phù hợp
// ============================================================

// I2C cho ADS1115
#define PIN_SDA         25
#define PIN_SCL         26

// H-bridge: 2 kênh PWM độc lập
#define PIN_MOTOR_IN1   18
#define PIN_MOTOR_IN2   19

// Cài đặt PWM
#define PWM_FREQ_HZ     20000
#define PWM_BITS        10

// Chu kỳ lấy mẫu và điều khiển
#define SAMPLE_MS       2       // 500 Hz  (ADS1115 max = 860 SPS ✓)
#define SERIAL_BAUD     460800  // cần >= 230400 để không tràn buffer ở 500 Hz

// ============================================================
//  GIỚI HẠN AN TOÀN
// ============================================================
#define ALPHA_MIN_DEG   2.0f
#define ALPHA_MAX_DEG  42.0f
#define PWM_MAX_ABS     0.85f

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

// Tham số PRMS — được set qua lệnh 'r'
struct PRMSParams {
    float    amplitude  = 0.5f;
    float    offset     = 0.0f;
    uint16_t clock_ms   = 100;   // thời gian giữ mỗi bước (ms)
    float    duration   = 60.0f;
    int      num_levels = 3;     // số mức: 3 hoặc 5 thường dùng
    uint16_t seed       = 1;     // giá trị khởi đầu LFSR (1 – 32767)
};

volatile SweepState  sweepState   = STATE_IDLE;
volatile SweepState  g_nextState  = STATE_SWEEP; // trạng thái sau baseline

SweepParams          params;
PRMSParams           prmsParams;

unsigned long        phaseStartMs  = 0;

// PRMS runtime state
volatile uint16_t    g_lfsrState    = 1;
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
    int16_t adc0 = ads.getLastConversionResults();
    float voltage = ads.computeVolts(adc0);
    return 84.22851f * voltage;
}

// ============================================================
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
//  HÀM LFSR 15-BIT (polynomial x^15 + x^14 + 1)
//  Chu kỳ: 2^15 - 1 = 32 767 bước
// ============================================================
uint16_t lfsr_step(uint16_t state) {
    // Bit feedback = bit14 XOR bit13
    uint16_t bit = ((state >> 14) ^ (state >> 13)) & 1;
    state = ((state << 1) | bit) & 0x7FFF;
    if (state == 0) state = 1; // tránh trạng thái kẹt tại 0
    return state;
}

// ============================================================
//  ÁNH XẠ LFSR STATE → MỨC PWM
//  Chia đều num_levels mức trong [-amplitude, +amplitude]
//  idx=0 → -amplitude, idx=num_levels-1 → +amplitude
// ============================================================
float prmsLevel(uint16_t state, int num_levels, float amplitude) {
    int idx = (int)(state % (uint16_t)num_levels);
    float ratio = (num_levels > 1) ? (float)idx / (float)(num_levels - 1) : 0.5f;
    return amplitude * (2.0f * ratio - 1.0f);
}

// ============================================================
//  TASK ĐỌC CẢM BIẾN — ưu tiên cao, 20ms
// ============================================================
void TaskSensor(void *pvParameters) {
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;) {
        float alpha = 155.0f - readAlpha();
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
                Serial.printf("%lu,%.2f,%.2f\n", now_ms, pwm_out * 100.0f, alpha);
                if (now_ms - phaseStartMs >= 1000) {
                    phaseStartMs = now_ms;
                    sweepState   = g_nextState; // chuyển sang CHIRP hoặc PRMS

                    if (g_nextState == STATE_SWEEP) {
                        Serial.println("# CHIRP_START");
                    } else {
                        // Reset LFSR về seed ban đầu
                        g_lfsrState    = prmsParams.seed;
                        g_prmsLastTick = -1;
                        g_prmsOutput   = prmsLevel(g_lfsrState,
                                                    prmsParams.num_levels,
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
                Serial.printf("%lu,%.2f,%.2f\n", now_ms, pwm_out * 100.0f, alpha);
                break;
            }

            // ── PRMS: Pseudo-Random Multi-level Sequence ─────
            case STATE_PRMS:
            {
                float t = (float)(now_ms - phaseStartMs) / 1000.0f;

                if (t >= prmsParams.duration) {
                    sweepState = STATE_DONE;
                    sendPWM(0.0f);
                    Serial.println("# PRMS_END");
                    break;
                }

                // Xác định tick hiện tại dựa trên clock_ms
                int tick = (int)(t * 1000.0f / (float)prmsParams.clock_ms);

                // Khi sang tick mới → bước LFSR và cập nhật mức output
                if (tick != g_prmsLastTick) {
                    g_prmsLastTick = tick;
                    g_lfsrState    = lfsr_step(g_lfsrState);
                    g_prmsOutput   = prmsLevel(g_lfsrState,
                                               prmsParams.num_levels,
                                               prmsParams.amplitude);
                }

                pwm_out = prmsParams.offset + g_prmsOutput;

                // Kiểm tra an toàn góc
                bool nearMin = (alpha < ALPHA_MIN_DEG + 1.0f) && (pwm_out < 0.0f);
                bool nearMax = (alpha > ALPHA_MAX_DEG - 1.0f) && (pwm_out > 0.0f);
                if (nearMin || nearMax) {
                    pwm_out = 0.0f;
                    Serial.printf("# CLAMP t=%.2f alpha=%.2f\n", t, alpha);
                }

                sendPWM(pwm_out);
                Serial.printf("%lu,%.2f,%.2f\n", now_ms, pwm_out * 100.0f, alpha);
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
                    Serial.printf("# [PRMS]  clk=%dms  A=%.2f  off=%.2f  T=%.0fs  levels=%d  seed=%d\n",
                                   prmsParams.clock_ms, prmsParams.amplitude, prmsParams.offset,
                                   prmsParams.duration, prmsParams.num_levels, prmsParams.seed);
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

            // ── LỆNH w: bắt đầu chirp sweep ─────────────────
            if (cmd == 'w') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: dang chay, gui 's' de dung truoc");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                float f1, f2, amp, off, dur;
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
            // Cú pháp: r <amplitude> <offset> <clock_ms> <duration> <num_levels> <seed>
            // Ví dụ:   r 0.5 0.0 100 60 3 1
            else if (cmd == 'r') {
                if (sweepState != STATE_IDLE) {
                    Serial.println("# ERR: dang chay, gui 's' de dung truoc");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                float amp, off, dur;
                int clk_ms, nlevels;
                unsigned int seed_val;
                int parsed = sscanf(input.c_str(), "r %f %f %d %f %d %u",
                                    &amp, &off, &clk_ms, &dur, &nlevels, &seed_val);

                if (parsed != 6) {
                    Serial.println("# ERR: cu phap sai");
                    Serial.println("# Vi du: r 0.5 0.0 100 60 3 1");
                    Serial.println("#   amplitude offset clock_ms duration num_levels seed");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (amp <= 0.0f || amp > PWM_MAX_ABS) {
                    Serial.printf("# ERR: amplitude phai trong khoang (0, %.2f]\n", PWM_MAX_ABS);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (clk_ms < (int)SAMPLE_MS) {
                    Serial.printf("# ERR: clock_ms phai >= %d (chu ky lay mau)\n", SAMPLE_MS);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (dur < 5.0f || dur > 600.0f) {
                    Serial.println("# ERR: duration phai trong khoang [5, 600] giay");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (nlevels < 2 || nlevels > 9) {
                    Serial.println("# ERR: num_levels phai trong khoang [2, 9]");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }
                if (seed_val == 0 || seed_val > 32767) {
                    Serial.println("# ERR: seed phai trong khoang [1, 32767]");
                    vTaskDelay(pdMS_TO_TICKS(100));
                    continue;
                }

                prmsParams.amplitude  = amp;
                prmsParams.offset     = off;
                prmsParams.clock_ms   = (uint16_t)clk_ms;
                prmsParams.duration   = dur;
                prmsParams.num_levels = nlevels;
                prmsParams.seed       = (uint16_t)seed_val;

                g_nextState  = STATE_PRMS;
                phaseStartMs = millis();
                sweepState   = STATE_BASELINE;

                Serial.println("# ============================================");
                Serial.printf("# PRMS: A=%.3f  off=%.3f  clk=%dms  T=%.0fs  levels=%d  seed=%d\n",
                               amp, off, clk_ms, dur, nlevels, seed_val);
                Serial.printf("# Buoc chuyen doi: %.1f Hz  |  Sample rate: %d Hz  |  Baud: %d\n",
                               1000.0f / clk_ms, 1000 / SAMPLE_MS, SERIAL_BAUD);
                Serial.printf("# Chu ky LFSR: 32767 buoc = %.1f s\n",
                               32767.0f * clk_ms / 1000.0f);
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
                    Serial.printf("# [PRMS]  Progress: %.1f%%  tick=%d  output=%.3f\n",
                                   elapsed / prmsParams.duration * 100.0f, tick, g_prmsOutput);
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
    if (!ads.begin()) {
        Serial.println("# FATAL: Khong tim thay ADS1115!");
        while (1) vTaskDelay(pdMS_TO_TICKS(100));
    }
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
    ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, true);
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
