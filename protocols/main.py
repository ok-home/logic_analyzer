# main.py
import sys
import os
import json
import numpy as np
from PyQt5.QtWidgets import (QMainWindow, QApplication, QVBoxLayout, QHBoxLayout,
                             QWidget, QPushButton, QFileDialog, QMessageBox,
                             QMenuBar, QAction, QStatusBar, QDialog, QTextEdit,
                             QVBoxLayout as QVBoxLayout2, QTableWidget, QTableWidgetItem)
from PyQt5.QtCore import Qt

from config import load_la_config, save_la_config
from settings_dialog import SettingsDialog
from serial_worker import SerialWorker
from waveform_widget import WaveformWidget
from universal_decoder_dialog import UniversalDecoderDialog
import sigrokdecode_stub as srd

class PulseViewApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PulseView Clone - Logic Analyzer Viewer")
        self.resize(1200, 700)

        self.data = None
        self.samples = None
        self.time_axis = None
        self.config = {}
        self.serial_thread = None
        self.acquisition_in_progress = False

        self.waveform_widget = WaveformWidget()
        self.waveform_widget.vline.hide()

        self.waveform_widget.reference_marker_changed.connect(self.update_reference_marker)
        self.waveform_widget.mouse_moved_with_time.connect(self.on_waveform_mouse_move)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready. Left click to set reference marker. Right click to clear.")

        self.reset_btn = QPushButton("Reset View")
        self.reset_btn.clicked.connect(self.reset_view)
        self.start_btn = QPushButton("Start Acquisition")
        self.start_btn.clicked.connect(self.start_acquisition)
        self.decode_btn = QPushButton("Decode Protocol...")
        self.decode_btn.clicked.connect(self.decode_protocol)
        self.clear_ref_btn = QPushButton("Clear Reference")
        self.clear_ref_btn.clicked.connect(self.clear_reference)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.decode_btn)
        btn_layout.addWidget(self.clear_ref_btn)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout()
        layout.addWidget(self.waveform_widget)
        layout.addLayout(btn_layout)
        central.setLayout(layout)

        self.create_menu()
        self.load_la_config()

    def create_menu(self):
        menubar = self.menuBar()
        file_menu = menubar.addMenu('File')
        open_action = QAction('Open', self)
        open_action.triggered.connect(self.open_file)
        file_menu.addAction(open_action)
        save_action = QAction('Save', self)
        save_action.triggered.connect(self.save_file)
        file_menu.addAction(save_action)
        file_menu.addSeparator()
        exit_action = QAction('Exit', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        settings_menu = menubar.addMenu('Settings')
        settings_action = QAction('Configuration', self)
        settings_action.triggered.connect(self.show_settings_dialog)
        settings_menu.addAction(settings_action)

    def clear_reference(self):
        self.waveform_widget.clear_reference_marker()

    def update_reference_marker(self, time_sec):
        if time_sec is None:
            self.status_bar.showMessage("Reference marker cleared.")
        else:
            self.status_bar.showMessage(f"Reference marker at {self.format_time(time_sec)}")

    def on_waveform_mouse_move(self, current_time):
        if current_time is None:
            self.status_bar.showMessage("Move mouse over plot.")
        else:
            ref_time = self.waveform_widget.get_reference_time()
            if ref_time is None:
                self.status_bar.showMessage(f"Time: {self.format_time(current_time)}")
            else:
                delta = current_time - ref_time
                sign = '+' if delta >= 0 else '-'
                abs_delta = abs(delta)
                delta_str = self.format_time(abs_delta, sign=sign)
                freq = 1.0 / abs_delta if abs_delta > 0 else float('inf')
                freq_str = self.format_frequency(freq)
                self.status_bar.showMessage(f"Δt = {delta_str}   f = {freq_str}   (ref at {self.format_time(ref_time)})")

    def format_time(self, seconds, sign=''):
        if seconds is None or np.isnan(seconds):
            return "N/A"
        abs_sec = abs(seconds)
        if abs_sec < 1e-9:
            val = seconds * 1e9
            unit = "ns"
        elif abs_sec < 1e-6:
            val = seconds * 1e6
            unit = "µs"
        elif abs_sec < 1e-3:
            val = seconds * 1e6
            unit = "µs"
        elif abs_sec < 1:
            val = seconds * 1e3
            unit = "ms"
        else:
            val = seconds
            unit = "s"
        return f"{sign}{val:.3f} {unit}"

    def format_frequency(self, freq):
        if freq is None or np.isinf(freq):
            return "∞"
        if freq < 1e3:
            return f"{freq:.1f} Hz"
        elif freq < 1e6:
            return f"{freq/1e3:.1f} kHz"
        elif freq < 1e9:
            return f"{freq/1e6:.1f} MHz"
        else:
            return f"{freq/1e9:.1f} GHz"

    def load_la_config(self):
        self.config = load_la_config()

    def save_la_config(self, config=None):
        if config is not None:
            self.config = config
        save_la_config(self.config)

    def show_settings_dialog(self):
        dlg = SettingsDialog(self)
        dlg.set_config(self.config)
        if dlg.exec_():
            new_cfg = dlg.get_config()
            if new_cfg:
                self.config.update(new_cfg)
                self.save_la_config()
                self.status_bar.showMessage("Configuration saved. You can start acquisition.")

    def open_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select binary file", "", "Binary files (*.bin);;All files (*.*)")
        if path:
            try:
                with open(path, "rb") as f:
                    self.data = f.read()
                self.status_bar.showMessage(f"Loaded: {os.path.basename(path)} ({len(self.data)} bytes)")
                self.parse_binary_data()
                self.waveform_widget.plot_signals(self.samples, self.time_axis, self.config)
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Cannot load file:\n{e}")

    def save_file(self):
        if self.data is None:
            QMessageBox.warning(self, "Save", "No data to save.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save binary file", "", "Binary files (*.bin)")
        if path:
            with open(path, "wb") as f:
                f.write(self.data)
            self.status_bar.showMessage(f"Saved: {os.path.basename(path)}")

    def parse_binary_data(self):
        if self.data is None:
            return
        num_channels = self.config.get('num_channels', 16)   # из настроек (8 или 16)
        bytes_per_sample = (num_channels + 7) // 8           # 1 для 8, 2 для 16
        total_samples = len(self.data) // bytes_per_sample
        if total_samples == 0:
            QMessageBox.critical(self, "Error", "Not enough data for the selected channel count.")
            return

        # Обрезаем данные до целого числа сэмплов
        data = self.data[:total_samples * bytes_per_sample]

        # Преобразуем сырые байты в целые числа (uint32 для удобства)
        samples_int = np.zeros(total_samples, dtype=np.uint32)
        for i in range(total_samples):
            val = 0
            for b in range(bytes_per_sample):
                byte = data[i * bytes_per_sample + b]
                val |= (byte << (8 * b))
            samples_int[i] = val

        # Извлекаем отдельные каналы
        self.samples = np.zeros((num_channels, total_samples), dtype=np.uint8)
        for ch in range(num_channels):
            self.samples[ch, :] = (samples_int >> ch) & 1

        sample_rate = self.config.get('sample_rate', 1_000_000)
        self.time_axis = np.arange(total_samples) / sample_rate

    def start_acquisition(self):
        if self.acquisition_in_progress:
            QMessageBox.warning(self, "Acquisition", "Already in progress. Please wait.")
            return
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None

        port_name = self.config.get('port', '')
        if not port_name:
            QMessageBox.critical(self, "Error", "No COM port selected. Please configure in Settings.")
            return

        self.acquisition_in_progress = True
        self.start_btn.setEnabled(False)
        self.status_bar.showMessage(f"Starting acquisition on {port_name}...")

        self.serial_thread = SerialWorker(self.config)
        self.serial_thread.status_message.connect(self.status_bar.showMessage)
        self.serial_thread.error_occurred.connect(self.on_acquisition_error)
        self.serial_thread.data_received.connect(self.on_data_received)
        self.serial_thread.transfer_done.connect(self.on_transfer_done)
        self.serial_thread.config_received.connect(self.show_analyzer_config)
        self.serial_thread.start()

    def on_acquisition_error(self, msg):
        self.acquisition_in_progress = False
        self.start_btn.setEnabled(True)
        QMessageBox.critical(self, "Acquisition Error", msg)
        self.status_bar.showMessage("Acquisition failed.")

    def on_data_received(self, data):
        self.data = data
        self.parse_binary_data()
        self.waveform_widget.plot_signals(self.samples, self.time_axis, self.config)

    def on_transfer_done(self):
        self.acquisition_in_progress = False
        self.start_btn.setEnabled(True)
        self.status_bar.showMessage("Acquisition completed.")

    def show_analyzer_config(self, config_dict):
        dialog = QDialog(self)
        dialog.setWindowTitle("Analyzer Configuration")
        dialog.resize(500, 300)
        layout = QVBoxLayout2()
        text_edit = QTextEdit()
        if "text" in config_dict:
            text_edit.setPlainText(config_dict["text"])
        else:
            text_edit.setPlainText(json.dumps(config_dict, indent=2))
        layout.addWidget(text_edit)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(dialog.accept)
        layout.addWidget(close_btn)
        dialog.setLayout(layout)
        dialog.exec_()

    # ========== Универсальный декодер протоколов ==========
    def decode_protocol(self):
        if self.samples is None:
            QMessageBox.warning(self, "Decode", "No data loaded.")
            return
        dlg = UniversalDecoderDialog(self, config=self.config, save_config_callback=self.save_la_config)
        if not dlg.exec_():
            return
        settings = dlg.get_settings()
        info = settings['protocol_info']
        decoder = settings['decoder_class']()

        total_samples = self.samples.shape[1]
        words = []
        for i in range(total_samples):
            word = 0
            for ch in range(self.samples.shape[0]):
                if self.samples[ch, i]:
                    word |= (1 << ch)
            words.append(word)

        metadata = {
            'samplerate': self.config.get('sample_rate', 1_000_000),
            'channels': settings['channels'],
            'channel_bits': settings['channel_bits'],
            'num_channels': settings['num_channels'],
            'options': settings['options'],
        }

        try:
            results = srd.run_decoder(decoder, words, metadata=metadata)
        except Exception as e:
            QMessageBox.critical(self, "Decode Error", str(e))
            return

        if not results:
            QMessageBox.information(self, "Decode", "No data decoded.")
            return

        self.waveform_widget.add_annotations(results, decoder.output_protocols, info,
                                             self.config.get('sample_rate', 1_000_000))
        self.show_decoder_results(results, decoder.output_protocols, info)

    def show_decoder_results(self, results, out_map, info):
        win = QDialog(self)
        win.setWindowTitle("Decoder Results")
        win.resize(800, 500)
        layout = QVBoxLayout2()
        table = QTableWidget()
        table.setColumnCount(4)
        table.setHorizontalHeaderLabels(["Start", "End", "Type", "Data"])

        filtered = [(s, e, oid, data) for s, e, oid, data in results
                    if out_map.get(oid) in ('ann', 'python')]
        table.setRowCount(len(filtered))
        for i, (start, end, out_id, data) in enumerate(filtered):
            typ = out_map.get(out_id, 'unknown')
            table.setItem(i, 0, QTableWidgetItem(f"{start}"))
            table.setItem(i, 1, QTableWidgetItem(f"{end}"))
            if typ == 'ann' and isinstance(data, list) and len(data) >= 2:
                table.setItem(i, 2, QTableWidgetItem("ANN"))
                table.setItem(i, 3, QTableWidgetItem(data[1][0] if data[1] else ''))
            else:
                table.setItem(i, 2, QTableWidgetItem(typ.upper()))
                table.setItem(i, 3, QTableWidgetItem(str(data)))
        table.resizeColumnsToContents()
        layout.addWidget(table)
        win.setLayout(layout)
        win.exec_()

    def reset_view(self):
        if self.time_axis is not None:
            self.waveform_widget.graph_widget.setXRange(self.time_axis[0], self.time_axis[-1])

    def closeEvent(self, event):
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PulseViewApp()
    window.show()
    sys.exit(app.exec_())