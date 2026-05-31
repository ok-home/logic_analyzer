# settings_dialog.py
import sys
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
                             QTableWidget, QTableWidgetItem, QCheckBox, QLineEdit,
                             QPushButton, QGroupBox, QFormLayout, QTabWidget,
                             QHeaderView, QDialogButtonBox, QWidget)
from PyQt5.QtCore import Qt, QTimer
import serial.tools.list_ports

class SettingsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.setModal(True)
        self.resize(700, 600)

        self._current_trig_gpio = None
        self.update_timer = QTimer()
        self.update_timer.setSingleShot(True)
        self.update_timer.timeout.connect(self.update_trigger_gpio_list)

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
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        main_layout.addWidget(buttons)

        self.setLayout(main_layout)

        # Таблица каналов
        self.checkboxes = []
        self.gpio_edits = []
        for i in range(16):
            chk = QCheckBox()
            chk.setChecked(True)
            self.checkboxes.append(chk)
            self.table.setCellWidget(i, 0, chk)

            gpio_edit = QLineEdit()
            gpio_edit.setPlaceholderText("GPIO number")
            gpio_edit.textChanged.connect(self.on_gpio_text_changed)
            self.gpio_edits.append(gpio_edit)
            self.table.setCellWidget(i, 1, gpio_edit)

            label = QLabel(f"CH{i}")
            self.table.setCellWidget(i, 2, label)

        self.current_num_channels = 16
        self.update_channel_table()

    def on_gpio_text_changed(self):
        self.update_timer.start(100)

    def update_trigger_gpio_list(self):
        gpio_list = []
        for i in range(self.current_num_channels):
            text = self.gpio_edits[i].text().strip()
            if text and text != "-1":
                gpio_list.append(text)
        self.trig_gpio.clear()
        self.trig_gpio.addItem("None")
        self.trig_gpio.addItems(gpio_list)
        if self._current_trig_gpio:
            idx = self.trig_gpio.findText(self._current_trig_gpio)
            if idx >= 0:
                self.trig_gpio.setCurrentIndex(idx)

    def update_channel_table(self):
        num = int(self.channel_count_combo.currentText())
        self.current_num_channels = num
        for i in range(16):
            self.table.setRowHidden(i, i >= num)
        self.update_trigger_gpio_list()

    def trigger_enable_changed(self, state):
        enabled = (state == Qt.Checked)
        self.trig_gpio.setEnabled(enabled)
        self.trig_edge.setEnabled(enabled)

    def set_all_checked(self, state):
        for i in range(self.current_num_channels):
            self.checkboxes[i].setChecked(state)

    def get_config(self):
        try:
            num = int(self.channel_count_combo.currentText())
            show = []
            gpio_map = {}
            for i in range(num):
                if self.checkboxes[i].isChecked():
                    show.append(i)
                gpio_text = self.gpio_edits[i].text().strip()
                gpio_map[i] = gpio_text if gpio_text else "-1"

            smp = int(self.sample_count_edit.text())
            clk = float(self.rate_edit.text())
            ram = self.ram_combo.currentIndex()
            tmo = float(self.timeout_edit.text()) if self.timeout_edit.text() else 20.0

            trig_en = self.trigger_enable.isChecked()
            trig_gpio = self.trig_gpio.currentText() if trig_en else None
            if trig_gpio == "None":
                trig_gpio = None
            trig_edge = self.trig_edge.currentText() if trig_en else None
            if trig_edge == "posedge":
                edge_num = 1
            elif trig_edge == "negedge":
                edge_num = 2
            else:
                edge_num = 3

            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            data_file = self.data_file_edit.text()

            return {
                'num_channels': num,
                'show_channels': show,
                'sample_rate': clk,
                'sample_count': smp,
                'gpio': gpio_map,
                'ram_type': ram,
                'timeout_ms': tmo,
                'trigger_enabled': trig_en,
                'trigger_gpio': trig_gpio,
                'trigger_edge': trig_edge,
                'trigger_edge_num': edge_num,
                'port': port,
                'baudrate': baud,
                'data_file': data_file,
                'timeout': 120,
            }
        except Exception as e:
            print("Config error:", e)
            return None

    def set_config(self, cfg):
        num = cfg.get('num_channels', 16)
        idx = self.channel_count_combo.findText(str(num))
        if idx >= 0:
            self.channel_count_combo.setCurrentIndex(idx)
        self.update_channel_table()

        show_set = set(cfg.get('show_channels', list(range(num))))
        for i in range(num):
            self.checkboxes[i].setChecked(i in show_set)
            gpio_val = cfg.get('gpio', {}).get(i, "-1")
            if gpio_val != "-1":
                self.gpio_edits[i].setText(str(gpio_val))
            else:
                self.gpio_edits[i].clear()

        self.sample_count_edit.setText(str(cfg.get('sample_count', 10000)))
        self.rate_edit.setText(str(cfg.get('sample_rate', 1000000)))
        self.ram_combo.setCurrentIndex(cfg.get('ram_type', 0))
        self.timeout_edit.setText(str(cfg.get('timeout_ms', 20)))

        self.trigger_enable.setChecked(cfg.get('trigger_enabled', False))
        if cfg.get('trigger_enabled'):
            self._current_trig_gpio = cfg.get('trigger_gpio')
            if self._current_trig_gpio:
                self.update_trigger_gpio_list()
                idx = self.trig_gpio.findText(self._current_trig_gpio)
                if idx >= 0:
                    self.trig_gpio.setCurrentIndex(idx)
            edge_num = cfg.get('trigger_edge_num', 1)
            if edge_num == 1:
                edge_str = "posedge"
            elif edge_num == 2:
                edge_str = "negedge"
            else:
                edge_str = "both"
            idx = self.trig_edge.findText(edge_str)
            if idx >= 0:
                self.trig_edge.setCurrentIndex(idx)
        else:
            self._current_trig_gpio = None
            self.update_trigger_gpio_list()

        self.port_combo.setCurrentText(cfg.get('port', ''))
        baud = str(cfg.get('baudrate', 115200))
        idx = self.baud_combo.findText(baud)
        if idx >= 0:
            self.baud_combo.setCurrentIndex(idx)
        self.data_file_edit.setText(cfg.get('data_file', 'laRowBin.bin'))

    def accept(self):
        self._current_trig_gpio = self.trig_gpio.currentText()
        if self._current_trig_gpio == "None":
            self._current_trig_gpio = None
        super().accept()