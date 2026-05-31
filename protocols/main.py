# main.py
import sys
import os
import json
import numpy as np
from PyQt5.QtWidgets import (
    QMainWindow, QApplication, QVBoxLayout, QHBoxLayout,
    QWidget, QPushButton, QFileDialog, QMessageBox,
    QMenuBar, QAction, QStatusBar, QDialog, QTextEdit,
    QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt

from config import load_la_config, save_la_config, AnalyzerConfig
from settings_dialog import SettingsDialog
from serial_worker import SerialWorker
from waveform_widget import WaveformWidget
from decoder_manager_dialog import DecoderManagerDialog
import sigrokdecode_stub as srd
from protocol_scanner import scan_decoders


class PulseViewApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PulseView Clone - Logic Analyzer Viewer")
        self.resize(1200, 700)

        self.data: bytes = None
        self.samples: np.ndarray = None
        self.time_axis: np.ndarray = None
        self.config: AnalyzerConfig = None
        self.serial_thread = None
        self.acquisition_in_progress = False

        self.waveform_widget = WaveformWidget()
        self.waveform_widget.vline.hide()
        self.waveform_widget.reference_marker_changed.connect(self.update_reference_marker)
        self.waveform_widget.mouse_moved_with_time.connect(self.on_waveform_mouse_move)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready. Left click to set reference marker. Right click to clear.")

        self.reset_btn = QPushButton("Zoom to Fit")
        self.reset_btn.clicked.connect(self.reset_view)
        self.start_btn = QPushButton("Start Acquisition")
        self.start_btn.clicked.connect(self.start_acquisition)
        self.clear_ref_btn = QPushButton("Clear Reference")
        self.clear_ref_btn.clicked.connect(self.clear_reference)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.reset_btn)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.clear_ref_btn)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout()
        layout.addWidget(self.waveform_widget, stretch=1)
        layout.addLayout(btn_layout)
        central.setLayout(layout)

        self.create_menu()
        self.config = load_la_config()

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

        # Меню Decoders
        decoders_menu = menubar.addMenu('Decoders')
        manage_action = QAction('Manage Decoders...', self)
        manage_action.triggered.connect(self.open_decoder_manager)
        decoders_menu.addAction(manage_action)
        decode_all_action = QAction('Decode All', self)
        decode_all_action.triggered.connect(self._sync_decoder_plots)
        decoders_menu.addAction(decode_all_action)

    def open_decoder_manager(self):
        dlg = DecoderManagerDialog(self.config, self._save_current_config, self)
        dlg.decoders_changed.connect(self._sync_decoder_plots)
        if dlg.exec_():
            self._sync_decoder_plots()
            self.status_bar.showMessage("Decoders configuration updated.")

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

    def _save_current_config(self, config=None):
        cfg_to_save = config if config is not None else self.config
        save_la_config(cfg_to_save)
        if config is not None:
            self.config = config

    def show_settings_dialog(self):
        dlg = SettingsDialog(self)
        dlg.set_config(self.config)
        if dlg.exec_():
            new_cfg = dlg.get_config()
            if new_cfg:
                self.config = new_cfg
                self._save_current_config()
                self.status_bar.showMessage("Configuration saved. You can start acquisition.")

    def open_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select binary file", "", "Binary files (*.bin);;All files (*.*)")
        if path:
            try:
                with open(path, "rb") as f:
                    self.data = f.read()
                self.status_bar.showMessage(f"Loaded: {os.path.basename(path)} ({len(self.data)} bytes)")
                if not self.parse_binary_data():
                    QMessageBox.warning(self, "Warning", "No data to display.")
                    return
                self.waveform_widget.plot_signals(self.samples, self.time_axis, self.config)
                self._sync_decoder_plots()
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

    def parse_binary_data(self) -> bool:
        if self.data is None or len(self.data) == 0:
            return False
        num_channels = self.config.num_channels
        bytes_per_sample = (num_channels + 7) // 8
        total_samples = len(self.data) // bytes_per_sample
        if total_samples == 0:
            QMessageBox.critical(self, "Error", "Not enough data for the selected channel count.")
            return False

        valid_len = total_samples * bytes_per_sample
        raw = self.data[:valid_len]

        if bytes_per_sample == 1:
            arr = np.frombuffer(raw, dtype=np.uint8).astype(np.uint32)
        elif bytes_per_sample == 2:
            arr = np.frombuffer(raw, dtype=np.uint16).astype(np.uint32)
        else:
            arr = np.frombuffer(raw, dtype=np.uint32)

        bit_masks = np.array([1 << ch for ch in range(num_channels)], dtype=np.uint32)
        self.samples = ((arr[np.newaxis, :] & bit_masks[:, np.newaxis]) != 0).astype(np.uint8)
        self.time_axis = np.arange(total_samples) / self.config.sample_rate
        return True

    def start_acquisition(self):
        if self.acquisition_in_progress:
            QMessageBox.warning(self, "Acquisition", "Already in progress. Please wait.")
            return
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
            self.serial_thread = None

        port_name = self.config.port
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
        if self.parse_binary_data():
            self.waveform_widget.plot_signals(self.samples, self.time_axis, self.config)
            self._sync_decoder_plots()
        else:
            QMessageBox.warning(self, "Acquisition", "Received empty data.")

    def on_transfer_done(self):
        self.acquisition_in_progress = False
        self.start_btn.setEnabled(True)
        self.status_bar.showMessage("Acquisition completed.")

    def show_analyzer_config(self, config_dict):
        dialog = QDialog(self)
        dialog.setWindowTitle("Analyzer Configuration")
        dialog.resize(500, 300)
        layout = QVBoxLayout()
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

    # ========== Управление декодерами ==========
    def _sanitize_options(self, info, options):
        """Приводит значения опций к правильным типам согласно спецификации протокола."""
        clean = {}
        for opt in info.options:
            oid = opt['id']
            if oid not in options:
                continue
            val = options[oid]
            default = opt.get('default')
            values = opt.get('values')
            if values is not None:
                try:
                    if isinstance(values[0], int):
                        val = int(val)
                    elif isinstance(values[0], float):
                        val = float(val)
                except (ValueError, TypeError):
                    pass
            elif isinstance(default, (int, float)):
                try:
                    if isinstance(default, int):
                        val = int(val)
                    else:
                        val = float(val)
                except (ValueError, TypeError):
                    pass
            clean[oid] = val
        return clean

    def _sync_decoder_plots(self):
        """Синхронизирует список активных декодеров с виджетами аннотаций."""
        active_ids = {ad['id'] for ad in self.config.active_decoders}
        current_plots = set(self.waveform_widget.decoder_plots.keys())

        for inst_id in current_plots - active_ids:
            self.waveform_widget.remove_decoder_plot(inst_id)

        if self.samples is None or self.samples.shape[1] == 0:
            return

        protocols = scan_decoders()
        num_channels = self.samples.shape[0]
        total_samples = self.samples.shape[1]
        weights = 2 ** np.arange(num_channels, dtype=np.uint32)
        words = np.dot(self.samples.T, weights).tolist()

        for ad in self.config.active_decoders:
            proto_id = ad['proto_id']
            instance_id = ad['id']
            if proto_id not in protocols:
                continue
            info = protocols[proto_id]
            decoder = info.module.Decoder()

            channels = ad.get('channels', {})
            all_indices = list(range(len(info.all_channels)))
            all_bits = []
            for idx, ch in enumerate(info.all_channels):
                ch_id = ch['id']
                is_optional = ch in info.optional_channels
                if is_optional and not channels.get(ch_id + '_enabled', True):
                    all_bits.append(-1)
                else:
                    all_bits.append(channels.get(ch_id, idx))
            if all(b < 0 for b in all_bits):
                continue

            raw_options = ad.get('options', {})
            options = self._sanitize_options(info, raw_options)

            metadata = {
                'samplerate': self.config.sample_rate,
                'channels': all_indices,
                'channel_bits': all_bits,
                'num_channels': info.num_channels,
                'options': options,
            }
            try:
                results = srd.run_decoder(decoder, words, metadata=metadata)
            except Exception as e:
                QMessageBox.critical(self, "Decode Error", f"{info.name} ({instance_id[:8]}): {e}")
                continue

            self.waveform_widget.update_decoder_annotations(instance_id, results, decoder.output_protocols, info, self.config.sample_rate)

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