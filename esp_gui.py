import sys
import serial
import serial.tools.list_ports
import threading
import time
import csv
import os
from datetime import datetime
from collections import deque
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg

class SerialReader(QtCore.QObject):
    """Serial reader thread — emits signals to the UI"""
    data_received = QtCore.pyqtSignal(float, float, float) # t_ms, pwm, alpha
    message_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baud=460800):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = False
        self.ser = None

    def start(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.running = True
            while self.running:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue

                    if line.startswith('#'):
                        self.message_received.emit(line)
                    else:
                        parts = line.split(',')
                        if len(parts) == 3:
                            try:
                                t = float(parts[0])
                                p = float(parts[1])
                                a = float(parts[2])
                                self.data_received.emit(t, p, a)
                            except ValueError: pass
                else:
                    time.sleep(0.001)
        except Exception as e:
            self.message_received.emit(f"# ERROR: {str(e)}")
        finally:
            if self.ser: self.ser.close()

    def stop(self):
        self.running = False

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode('utf-8'))

class RealTimeGui(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 System ID — Chirp + PRMS  [500 Hz / 460800 baud]")
        self.resize(1150, 780)

        self.max_points = 2500   # 5 giây dữ liệu ở 500 Hz
        self.t_data    = deque(maxlen=self.max_points)
        self.pwm_data  = deque(maxlen=self.max_points)
        self.alpha_data = deque(maxlen=self.max_points)
        self.start_time = None

        self._pending: deque = deque()
        self._pending_lock = threading.Lock()

        self.csv_file = None
        self.csv_writer = None
        self.is_logging = False

        self.init_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(30)

        self.pwm_debounce_timer = QtCore.QTimer()
        self.pwm_debounce_timer.setSingleShot(True)
        self.pwm_debounce_timer.setInterval(50)
        self.pwm_debounce_timer.timeout.connect(self._do_send_pwm)

    def init_ui(self):
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout(central_widget)

        # ── CỘT TRÁI: ĐIỀU KHIỂN ──────────────────────────────
        control_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_panel, 1)

        # 1. Serial Connection
        conn_group = QtWidgets.QGroupBox("1. Serial Connection")
        conn_layout = QtWidgets.QFormLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.refresh_ports()
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        conn_layout.addRow("Port:", self.port_combo)
        conn_layout.addRow(self.btn_connect)
        conn_group.setLayout(conn_layout)
        control_panel.addWidget(conn_group)

        # 2. Data Logging
        save_group = QtWidgets.QGroupBox("2. Data Logging")
        save_layout = QtWidgets.QVBoxLayout()
        self.cb_save = QtWidgets.QCheckBox("Auto-save CSV during experiment")
        self.cb_save.setChecked(True)
        self.le_prefix = QtWidgets.QLineEdit("sweep_data")
        save_layout.addWidget(self.cb_save)
        save_layout.addWidget(QtWidgets.QLabel("Filename prefix:"))
        save_layout.addWidget(self.le_prefix)
        save_group.setLayout(save_layout)
        control_panel.addWidget(save_group)

        # 3. Signal Parameters (Chirp + PRMS)
        sig_group = QtWidgets.QGroupBox("3. Signal Parameters")
        sig_outer = QtWidgets.QVBoxLayout()

        # -- Selector loại tín hiệu --
        type_row = QtWidgets.QHBoxLayout()
        type_row.addWidget(QtWidgets.QLabel("Signal Type:"))
        self.sig_type_combo = QtWidgets.QComboBox()
        self.sig_type_combo.addItem("🔊  Chirp (Logarithmic Sweep)")
        self.sig_type_combo.addItem("🎲  PRMS (Pseudo-Random Multi-level)")
        self.sig_type_combo.currentIndexChanged.connect(self.on_signal_type_changed)
        type_row.addWidget(self.sig_type_combo)
        sig_outer.addLayout(type_row)

        # -- Divider --
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setStyleSheet("color: #555;")
        sig_outer.addWidget(line)

        # -- Stacked widget: tham số riêng mỗi loại tín hiệu --
        self.sig_stack = QtWidgets.QStackedWidget()

        # Page 0: Chirp params
        chirp_page = QtWidgets.QWidget()
        chirp_form = QtWidgets.QFormLayout(chirp_page)
        chirp_form.setContentsMargins(0, 0, 0, 0)
        self.f_start = QtWidgets.QLineEdit("0.05")
        self.f_end   = QtWidgets.QLineEdit("5.0")
        chirp_form.addRow("F Start (Hz):", self.f_start)
        chirp_form.addRow("F End (Hz):", self.f_end)
        self.sig_stack.addWidget(chirp_page)

        # Page 1: PRMS params
        prms_page = QtWidgets.QWidget()
        prms_form = QtWidgets.QFormLayout(prms_page)
        prms_form.setContentsMargins(0, 0, 0, 0)

        self.prms_clock_ms = QtWidgets.QLineEdit("4000")
        self.prms_clock_ms.setToolTip(
            "Thời gian giữ mỗi bước (ms). Phải >= 20ms (chu kỳ lấy mẫu).\n"
            "Theo bài báo: clock_ms = 1000 / (2.5 × f_bandwidth)\n"
            "  BW = 0.1 Hz → clock_ms = 4000 ms (4 s/bước)\n"
            "  BW = 1.0 Hz → clock_ms = 400 ms\n"
            "  BW = 5.0 Hz → clock_ms = 80 ms"
        )

        self.prms_levels = QtWidgets.QComboBox()
        for lv in ["2", "3", "5", "7", "9"]:
            self.prms_levels.addItem(lv)
        self.prms_levels.setCurrentIndex(1)  # mặc định 3 mức
        self.prms_levels.setToolTip(
            "Số mức biên độ:\n"
            "  3 mức → {-A, 0, +A}          (đơn giản, an toàn)\n"
            "  5 mức → {-A, -A/2, 0, +A/2, +A}  (gần giống bài báo)\n"
            "  7, 9 mức → kích thích phi tuyến tốt hơn\n"
            "Nhiều mức hơn = gần với PRBS×random_amp của bài báo."
        )

        self.prms_seed = QtWidgets.QLineEdit("1")
        self.prms_seed.setToolTip(
            "Giá trị khởi đầu LFSR [1–32767].\n"
            "Thay đổi seed → chuỗi khác, cùng tính chất thống kê.\n"
            "Chu kỳ LFSR 15-bit = 32767 bước."
        )

        # Thêm nhãn hướng dẫn thiết kế
        self.prms_hint = QtWidgets.QLabel(
            "<small><i>clock_ms = 1000 / (2.5 × BW_Hz) | "
            "duration ≥ 127 × clock_s</i></small>"
        )
        self.prms_hint.setStyleSheet("color: #888; padding: 2px;")

        prms_form.addRow("Clock Period (ms):", self.prms_clock_ms)
        prms_form.addRow("Levels:", self.prms_levels)
        prms_form.addRow("LFSR Seed:", self.prms_seed)
        prms_form.addRow(self.prms_hint)
        self.sig_stack.addWidget(prms_page)

        sig_outer.addWidget(self.sig_stack)

        # -- Divider --
        line2 = QtWidgets.QFrame()
        line2.setFrameShape(QtWidgets.QFrame.HLine)
        line2.setStyleSheet("color: #555;")
        sig_outer.addWidget(line2)

        # -- Tham số chung (amplitude, offset, duration) --
        common_form = QtWidgets.QFormLayout()
        common_form.setContentsMargins(0, 0, 0, 0)
        self.amplitude = QtWidgets.QLineEdit("0.5")
        self.offset    = QtWidgets.QLineEdit("0.0")
        self.duration  = QtWidgets.QLineEdit("508")
        common_form.addRow("Amplitude (0–0.85):", self.amplitude)
        common_form.addRow("Offset PWM:", self.offset)
        common_form.addRow("Duration (s):", self.duration)
        sig_outer.addLayout(common_form)

        # -- Nút START --
        self.btn_start = QtWidgets.QPushButton("▶  START SWEEP")
        self.btn_start.setStyleSheet(
            "background-color: #28a745; color: white; font-weight: bold; height: 40px;"
        )
        self.btn_start.clicked.connect(self.start_experiment)
        sig_outer.addWidget(self.btn_start)

        sig_group.setLayout(sig_outer)
        control_panel.addWidget(sig_group)

        # Nút dừng khẩn cấp
        self.btn_stop = QtWidgets.QPushButton("⛔  EMERGENCY STOP")
        self.btn_stop.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold; font-size: 16px; height: 50px;"
        )
        self.btn_stop.clicked.connect(self.emergency_stop)
        control_panel.addWidget(self.btn_stop)

        # 4. Manual PWM control
        manual_group = QtWidgets.QGroupBox("4. Manual PWM Control")
        manual_layout = QtWidgets.QVBoxLayout()

        dir_label_layout = QtWidgets.QHBoxLayout()
        lbl_fwd = QtWidgets.QLabel("◀ UP")
        lbl_fwd.setStyleSheet("color: #28a745; font-weight: bold;")
        lbl_rev = QtWidgets.QLabel("DOWN ▶")
        lbl_rev.setStyleSheet("color: #dc3545; font-weight: bold;")
        dir_label_layout.addWidget(lbl_fwd)
        dir_label_layout.addStretch()
        dir_label_layout.addWidget(lbl_rev)
        manual_layout.addLayout(dir_label_layout)

        self.pwm_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.pwm_slider.setMinimum(-100)
        self.pwm_slider.setMaximum(100)
        self.pwm_slider.setValue(0)
        self.pwm_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.pwm_slider.setTickInterval(25)
        self.pwm_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 8px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #28a745, stop:0.5 #6c757d, stop:1 #dc3545);
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 2px solid #555;
                width: 18px;
                height: 18px;
                margin: -5px 0;
                border-radius: 9px;
            }
        """)
        self.pwm_slider.valueChanged.connect(self.on_pwm_slider_changed)
        self.pwm_slider.sliderReleased.connect(self.on_pwm_slider_released)
        manual_layout.addWidget(self.pwm_slider)

        val_layout = QtWidgets.QHBoxLayout()
        self.lbl_pwm_val = QtWidgets.QLabel("PWM: 0%  (Stopped)")
        self.lbl_pwm_val.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_pwm_val.setStyleSheet("font-weight: bold; font-size: 13px;")
        val_layout.addWidget(self.lbl_pwm_val)
        manual_layout.addLayout(val_layout)

        self.btn_center = QtWidgets.QPushButton("⏹ Center (Stop motor)")
        self.btn_center.setStyleSheet(
            "background-color: #ffc107; color: black; font-weight: bold; height: 30px;"
        )
        self.btn_center.clicked.connect(self.reset_pwm_slider)
        manual_layout.addWidget(self.btn_center)

        manual_group.setLayout(manual_layout)
        control_panel.addWidget(manual_group)

        # Log console
        self.log_output = QtWidgets.QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet(
            "background-color: #1e1e1e; color: #d4d4d4; "
            "font-family: Consolas; font-size: 11px;"
        )
        control_panel.addWidget(self.log_output)

        # ── CỘT PHẢI: ĐỒ THỊ ─────────────────────────────────
        plot_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(plot_layout, 3)

        pg.setConfigOptions(antialias=False)

        self.pw_pwm = pg.PlotWidget(title="PWM Control (%)")
        self.pw_pwm.showGrid(x=True, y=True)
        self.pw_pwm.setYRange(-100, 100)
        self.curve_pwm = self.pw_pwm.plot(pen=pg.mkPen('r', width=1.5))
        plot_layout.addWidget(self.pw_pwm)

        self.pw_alpha = pg.PlotWidget(title="Angle Alpha (deg)")
        self.pw_alpha.showGrid(x=True, y=True)
        self.curve_alpha = self.pw_alpha.plot(pen=pg.mkPen('c', width=1.5))
        plot_layout.addWidget(self.pw_alpha)

    # ──────────────────────────────────────────────────────────
    #  Xử lý thay đổi loại tín hiệu
    # ──────────────────────────────────────────────────────────
    def on_signal_type_changed(self, idx):
        self.sig_stack.setCurrentIndex(idx)
        if idx == 0:
            self.btn_start.setText("▶  START SWEEP")
        else:
            self.btn_start.setText("▶  START PRMS")

    # ──────────────────────────────────────────────────────────
    #  Serial
    # ──────────────────────────────────────────────────────────
    def refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)

    def toggle_connection(self):
        if self.btn_connect.text() == "Connect":
            port = self.port_combo.currentText()
            if not port: return
            self.reader = SerialReader(port)
            self.reader_thread = threading.Thread(target=self.reader.start)
            self.reader.data_received.connect(self.on_data)
            self.reader.message_received.connect(self.on_message)
            self.reader_thread.start()
            self.btn_connect.setText("Disconnect")
        else:
            self.close_csv()
            self.reader.stop()
            self.btn_connect.setText("Connect")

    # ──────────────────────────────────────────────────────────
    #  Bắt đầu thí nghiệm (chirp hoặc PRMS)
    # ──────────────────────────────────────────────────────────
    def start_experiment(self):
        if not hasattr(self, 'reader'):
            QtWidgets.QMessageBox.warning(self, "Error", "Please connect to Serial first!")
            return

        # Mở file CSV nếu được chọn
        if self.cb_save.isChecked():
            self.close_csv()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            sig_tag = "chirp" if self.sig_type_combo.currentIndex() == 0 else "prms"
            filename = f"{self.le_prefix.text()}_{sig_tag}_{timestamp}.csv"
            try:
                self.csv_file = open(filename, mode='w', newline='')
                self.csv_writer = csv.writer(self.csv_file)
                self.log_output.append(f"<b>[FILE]</b> Saving to: {filename}")
            except Exception as e:
                self.log_output.append(f"# ERROR FILE: {e}")

        # Reset đồ thị
        with self._pending_lock:
            self._pending.clear()
        self.t_data.clear()
        self.pwm_data.clear()
        self.alpha_data.clear()
        self.start_time = None

        # Gửi lệnh tương ứng
        if self.sig_type_combo.currentIndex() == 0:
            # Chirp: w <f_start> <f_end> <amplitude> <offset> <duration>
            cmd = (f"w {self.f_start.text()} {self.f_end.text()} "
                   f"{self.amplitude.text()} {self.offset.text()} {self.duration.text()}")
        else:
            # PRMS: r <amplitude> <offset> <clock_ms> <duration> <num_levels> <seed>
            cmd = (f"r {self.amplitude.text()} {self.offset.text()} "
                   f"{self.prms_clock_ms.text()} {self.duration.text()} "
                   f"{self.prms_levels.currentText()} {self.prms_seed.text()}")

        self.reader.send(cmd)
        self.log_output.append(f"<b>[CMD]</b> Sent: <code>{cmd}</code>")

    # ──────────────────────────────────────────────────────────
    #  CSV
    # ──────────────────────────────────────────────────────────
    def close_csv(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.log_output.append("<b>[FILE]</b> Data file closed.")

    # ──────────────────────────────────────────────────────────
    #  Xử lý dữ liệu & tin nhắn từ ESP32
    # ──────────────────────────────────────────────────────────
    def on_data(self, t, p, a):
        if self.csv_writer:
            self.csv_writer.writerow([t, p, a])

        if self.start_time is None:
            self.start_time = t
        rel_t = (t - self.start_time) / 1000.0
        with self._pending_lock:
            self._pending.append((rel_t, p, a))

    def on_message(self, msg):
        # Tô màu các dòng quan trọng
        if "ERROR" in msg or "FATAL" in msg:
            self.log_output.append(f"<font color='#ff6b6b'><b>{msg}</b></font>")
        elif "WARN" in msg or "CLAMP" in msg:
            self.log_output.append(f"<font color='#ffa94d'>{msg}</font>")
        elif any(tag in msg for tag in ["CHIRP_START", "PRMS_START", "SWEEP_COMPLETE"]):
            self.log_output.append(f"<font color='#69db7c'><b>{msg}</b></font>")
        else:
            self.log_output.append(msg)
        self.log_output.moveCursor(QtGui.QTextCursor.End)

        if self.csv_file:
            self.csv_file.write(msg + "\n")

        # Đóng CSV khi thí nghiệm kết thúc
        if "# SWEEP_COMPLETE" in msg:
            self.close_csv()

    def update_plots(self):
        with self._pending_lock:
            batch = list(self._pending)
            self._pending.clear()

        for rel_t, p, a in batch:
            self.t_data.append(rel_t)
            self.pwm_data.append(p)
            self.alpha_data.append(a)

        if len(self.t_data) > 0:
            t_arr   = np.asarray(self.t_data,    dtype=np.float32)
            pwm_arr = np.asarray(self.pwm_data,  dtype=np.float32)
            alp_arr = np.asarray(self.alpha_data, dtype=np.float32)
            self.curve_pwm.setData(t_arr, pwm_arr)
            self.curve_alpha.setData(t_arr, alp_arr)

    # ──────────────────────────────────────────────────────────
    #  Manual PWM Slider
    # ──────────────────────────────────────────────────────────
    def on_pwm_slider_changed(self, value):
        if value < 0:
            self.lbl_pwm_val.setText(f"PWM: {abs(value)}%  ◀ Forward")
            self.lbl_pwm_val.setStyleSheet("font-weight: bold; font-size: 13px; color: #28a745;")
        elif value > 0:
            self.lbl_pwm_val.setText(f"PWM: {value}%  Reverse ▶")
            self.lbl_pwm_val.setStyleSheet("font-weight: bold; font-size: 13px; color: #dc3545;")
        else:
            self.lbl_pwm_val.setText("PWM: 0%  (Stopped)")
            self.lbl_pwm_val.setStyleSheet("font-weight: bold; font-size: 13px; color: #6c757d;")

        self.pwm_debounce_timer.start()

    def _do_send_pwm(self):
        if not hasattr(self, 'reader'):
            return
        value = self.pwm_slider.value()
        pwm_real = -value / 100.0
        self.reader.send(f"m {pwm_real:.3f}")
        self.log_output.append(f"<b>[MANUAL]</b> Sent PWM = {pwm_real:+.3f} ({-value:+d}%)")

    def on_pwm_slider_released(self):
        self.pwm_debounce_timer.stop()
        self._do_send_pwm()

    def reset_pwm_slider(self):
        self.pwm_slider.setValue(0)
        if hasattr(self, 'reader'):
            self.reader.send("m 0.000")
            self.log_output.append("<b>[MANUAL]</b> Reset to 0 — Motor stopped")

    def emergency_stop(self):
        if hasattr(self, 'reader'):
            self.reader.send("s")
            self.pwm_slider.setValue(0)
            self.log_output.append("<font color='red'><b>!!! EMERGENCY STOP SENT !!!</b></font>")

    def closeEvent(self, event):
        self.close_csv()
        if hasattr(self, 'reader'):
            self.reader.stop()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = RealTimeGui()
    gui.show()
    sys.exit(app.exec_())
