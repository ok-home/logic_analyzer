# settings_dialog.py
import sys
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QTableWidget, QTableWidgetItem, QCheckBox, QLineEdit,
    QPushButton, QGroupBox, QFormLayout, QTabWidget,
    QHeaderView, QDialogButtonBox, QWidget, QSpinBox, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer
import serial.tools.list_ports
from config import AnalyzerConfig

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.setModal(True)
        self.resize(700, 600)

        self._current_trig_gpio = None
        self._initial_config = None  # будет установлен в set_config

        main_layout = QVBoxLayout()
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        # --- Вкладка Channels ---
        channels_widget = QWidget()
        channels_layout = QVBoxLayout(channels_widget)

        top_layout = QHBoxLayout()
        top_layout.addWidget(QLabel("Number of channels:"))
        self.channel_count_combo = QComboBox()
        self.channel_count_combo.addItems(["8", "16"])
        self.channel_count_combo.currentTextChanged.connect(self.update_channel_table)
        top_layout.addWidget(self.channel_count_combo)
        top_layout.addStretch()
        channels_layout.addLayout(top_layout)

        self.table = QTableWidget(16, 3)
        self.table.setHorizontalHeaderLabels(["Show", "GPIO", "Channel"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setVerticalHeaderLabels([f"CH{i}" for i in range(16)])
        channels_layout.addWidget(self.table)

        btn_layout = QHBoxLayout()
        select_all_btn = QPushButton("Select All")
        select_all_btn.clicked.connect(lambda: self.set_all_checked(True))
        clear_all_btn = QPushButton("Clear All")
        clear_all_btn.clicked.connect(lambda: self.set_all_checked(False))
        btn_layout.addWidget(select_all_btn)
        btn_layout.addWidget(clear_all_btn)
        channels_layout.addLayout(btn_layout)

        basic_group = QGroupBox("Acquisition Settings")
        basic_form = QFormLayout(basic_group)
        self.sample_count_edit = QLineEdit("10000")
        basic_form.addRow("Number of samples:", self.sample_count_edit)
        self.rate_edit = QLineEdit("1000000")
        basic_form.addRow("Sampling rate (Hz):", self.rate_edit)
        self.ram_combo = QComboBox()
        self.ram_combo.addItems(["Internal RAM (0)", "PSRAM (1)"])
        basic_form.addRow("RAM type:", self.ram_combo)
        self.timeout_edit = QLineEdit("20")
        basic_form.addRow("Timeout (ms):", self.timeout_edit)
        channels_layout.addWidget(basic_group)

        tabs.addTab(channels_widget, "Channels")

        # --- Вкладка Trigger ---
        trigger_widget = QWidget()
        trigger_layout = QVBoxLayout(trigger_widget)
        self.trigger_enable = QCheckBox("Enable trigger")
        self.trigger_enable.stateChanged.connect(self.trigger_enable_changed)
        trigger_layout.addWidget(self.trigger_enable)

        trig_group = QGroupBox("Trigger settings")
        trig_form = QFormLayout(trig_group)
        self.trig_gpio = QComboBox()
        trig_form.addRow("GPIO:", self.trig_gpio)
        self.trig_edge = QComboBox()
        self.trig_edge.addItems(["posedge", "negedge", "both"])
        trig_form.addRow("Edge:", self.trig_edge)
        trigger_layout.addWidget(trig_group)

        tabs.addTab(trigger_widget, "Trigger")

        # --- Вкладка Connection ---
        conn_widget = QWidget()
        conn_layout = QFormLayout(conn_widget)
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo.addItems(available_ports if available_ports else ["COM3", "/dev/ttyUSB0"])
        conn_layout.addRow("COM Port:", self.port_combo)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"])
        conn_layout.addRow("Baud rate:", self.baud_combo)
        self.data_file_edit = QLineEdit("laRowBin.bin")
        conn_layout.addRow("Data file:", self.data_file_edit)
        tabs.addTab(conn_widget, "Connection")

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.validate_and_accept)
        buttons.rejected.connect(self.reject)
        main_layout.addWidget(buttons)

        self.setLayout(main_layout)

        # Таблица каналов: теперь используем SpinBox для GPIO
        self.checkboxes = []
        self.gpio_spinboxes = []
        for i in range(16):
            chk = QCheckBox()
            chk.setChecked(True)
            self.checkboxes.append(chk)
            self.table.setCellWidget(i, 0, chk)

            spin = QSpinBox()
            spin.setRange(-1, 39)   # допустимые номера GPIO, -1 = не используется
            spin.setValue(-1)
            spin.valueChanged.connect(self.on_gpio_changed)
            self.gpio_spinboxes.append(spin)
            self.table.setCellWidget(i, 1, spin)

            label = QLabel(f"CH{i}")
            self.table.setCellWidget(i, 2, label)

        self.current_num_channels = 16
        self.update_channel_table()

    def on_gpio_changed(self):
        # Мгновенно обновляем список триггерных GPIO
        self._update_trig_gpio_list()

    def _update_trig_gpio_list(self):
        gpio_list = []
        for i in range(self.current_num_channels):
            val = self.gpio_spinboxes[i].value()
            if val >= 0:
                gpio_list.append(str(val))
        self.trig_gpio.clear()
        self.trig_gpio.addItem("None")
        self.trig_gpio.addItems(gpio_list)
        if self._current_trig_gpio is not None:
            idx = self.trig_gpio.findText(self._current_trig_gpio)
            if idx >= 0:
                self.trig_gpio.setCurrentIndex(idx)

    def update_channel_table(self):
        num = int(self.channel_count_combo.currentText())
        self.current_num_channels = num
        for i in range(16):
            self.table.setRowHidden(i, i >= num)
        self._update_trig_gpio_list()

    def trigger_enable_changed(self, state):
        enabled = (state == Qt.Checked)
        self.trig_gpio.setEnabled(enabled)
        self.trig_edge.setEnabled(enabled)

    def set_all_checked(self, state):
        for i in range(self.current_num_channels):
            self.checkboxes[i].setChecked(state)

    def get_config(self) -> AnalyzerConfig:
        """Собрать и вернуть новый объект конфигурации (без сохранения в файл)."""
        num = int(self.channel_count_combo.currentText())
        show = []
        gpio_map = {}
        for i in range(num):
            if self.checkboxes[i].isChecked():
                show.append(i)
            gpio_map[i] = self.gpio_spinboxes[i].value()

        smp = int(self.sample_count_edit.text())
        clk = float(self.rate_edit.text())
        ram = self.ram_combo.currentIndex()
        tmo = float(self.timeout_edit.text()) if self.timeout_edit.text() else 20.0

        trig_en = self.trigger_enable.isChecked()
        trig_gpio_str = self.trig_gpio.currentText() if trig_en else "None"
        trig_gpio = None
        if trig_gpio_str != "None":
            try:
                trig_gpio = int(trig_gpio_str)
            except:
                trig_gpio = None
        trig_edge_str = self.trig_edge.currentText() if trig_en else "posedge"
        edge_num = {"posedge": 1, "negedge": 2, "both": 3}.get(trig_edge_str, 1)

        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())
        data_file = self.data_file_edit.text()

        # Берем decoder_settings из исходного конфига, чтобы не потерять другие протоколы
        decoder_settings = {}
        if self._initial_config is not None:
            decoder_settings = self._initial_config.decoder_settings.copy()

        config = AnalyzerConfig(
            num_channels=num,
            show_channels=show,
            sample_rate=clk,
            sample_count=smp,
            gpio=gpio_map,
            ram_type=ram,
            timeout_ms=tmo,
            trigger_enabled=trig_en,
            trigger_gpio=trig_gpio,
            trigger_edge_num=edge_num,
            trigger_edge=trig_edge_str,
            port=port,
            baudrate=baud,
            data_file=data_file,
            timeout=120.0,
            decoder_settings=decoder_settings
        )
        return config

    def set_config(self, cfg: AnalyzerConfig):
        self._initial_config = cfg  # сохраняем для последующего использования в get_config
        idx = self.channel_count_combo.findText(str(cfg.num_channels))
        if idx >= 0:
            self.channel_count_combo.setCurrentIndex(idx)
        self.update_channel_table()

        show_set = set(cfg.show_channels)
        for i in range(cfg.num_channels):
            self.checkboxes[i].setChecked(i in show_set)
            self.gpio_spinboxes[i].setValue(cfg.gpio.get(i, -1))

        self.sample_count_edit.setText(str(cfg.sample_count))
        self.rate_edit.setText(str(cfg.sample_rate))
        self.ram_combo.setCurrentIndex(cfg.ram_type)
        self.timeout_edit.setText(str(cfg.timeout_ms))

        self.trigger_enable.setChecked(cfg.trigger_enabled)
        if cfg.trigger_enabled:
            self._current_trig_gpio = str(cfg.trigger_gpio) if cfg.trigger_gpio is not None else "None"
            self._update_trig_gpio_list()
            idx = self.trig_gpio.findText(self._current_trig_gpio)
            if idx >= 0:
                self.trig_gpio.setCurrentIndex(idx)
            edge_idx = self.trig_edge.findText(cfg.trigger_edge)
            if edge_idx >= 0:
                self.trig_edge.setCurrentIndex(edge_idx)
        else:
            self._current_trig_gpio = None
            self._update_trig_gpio_list()

        self.port_combo.setCurrentText(cfg.port)
        baud_str = str(cfg.baudrate)
        idx = self.baud_combo.findText(baud_str)
        if idx >= 0:
            self.baud_combo.setCurrentIndex(idx)
        self.data_file_edit.setText(cfg.data_file)

    def validate_and_accept(self):
        # Создаём временный конфиг для проверки
        try:
            cfg = self.get_config()
            valid, errors = cfg.validate()
            if not valid:
                QMessageBox.warning(self, "Validation Error", "\n".join(errors))
                return
            self.accept()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Invalid configuration: {e}")