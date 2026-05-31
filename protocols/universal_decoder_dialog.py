# universal_decoder_dialog.py
import os
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QFormLayout, QComboBox, QSpinBox,
    QDoubleSpinBox, QCheckBox, QPushButton, QLabel, QWidget, QHBoxLayout,
    QDialogButtonBox, QLineEdit, QGroupBox, QMessageBox
)
from protocol_scanner import scan_decoders
from config import AnalyzerConfig, save_la_config

class UniversalDecoderDialog(QDialog):
    def __init__(self, parent=None, config: AnalyzerConfig = None, save_config_callback=None, instance_id=None):
        super().__init__(parent)
        self.setWindowTitle("Configure Protocol Decoder")
        self.setModal(True)
        self.resize(500, 450)

        self.config = config if config else AnalyzerConfig()
        self.save_config = save_config_callback if save_config_callback else (lambda cfg: save_la_config(cfg))
        self.instance_id = instance_id  # может быть None для старых вызовов, но сейчас всегда передаётся

        self.decoders = scan_decoders()
        self.current_info = None

        layout = QVBoxLayout()
        self.form_layout = QFormLayout()

        self.protocol_combo = QComboBox()
        self.protocol_combo.addItems(list(self.decoders.keys()))
        self.protocol_combo.currentTextChanged.connect(self.on_protocol_changed)
        self.form_layout.addRow("Protocol:", self.protocol_combo)

        self.enable_checkbox = QCheckBox("Enable decoder (show annotation window)")
        self.form_layout.addRow(self.enable_checkbox)

        self.channels_group = QGroupBox("Channels")
        self.channels_form = QFormLayout()
        self.channels_group.setLayout(self.channels_form)

        self.options_group = QGroupBox("Options")
        self.options_form = QFormLayout()
        self.options_group.setLayout(self.options_form)

        self.form_layout.addRow(self.channels_group)
        self.form_layout.addRow(self.options_group)

        layout.addLayout(self.form_layout)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.validate_and_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)

        self.channel_spinboxes = {}
        self.channel_checks = {}
        self.option_widgets = {}

        if self.decoders:
            self.on_protocol_changed(self.protocol_combo.currentText())

    def on_protocol_changed(self, proto_id):
        while self.channels_form.count():
            item = self.channels_form.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        while self.options_form.count():
            item = self.options_form.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        self.channel_spinboxes.clear()
        self.channel_checks.clear()
        self.option_widgets.clear()

        if proto_id not in self.decoders:
            return
        info = self.decoders[proto_id]
        self.current_info = info

        # Загружаем сохранённые настройки: если есть instance_id, ищем в active_decoders
        saved_channels = {}
        saved_options = {}
        saved_enabled = False
        if self.instance_id:
            for ad in self.config.active_decoders:
                if ad.get('id') == self.instance_id:
                    saved_channels = ad.get('channels', {})
                    saved_options = ad.get('options', {})
                    saved_enabled = True
                    break
        if not saved_channels:
            # Запасной вариант: последние настройки для протокола из decoder_settings
            saved = self.config.decoder_settings.get(proto_id, {})
            saved_channels = saved.get('channels', {})
            saved_options = saved.get('options', {})

        self.enable_checkbox.setChecked(saved_enabled)

        for idx, ch in enumerate(info.all_channels):
            row = QHBoxLayout()
            label = QLabel(f"{ch['name']} ({ch['desc']})")
            spin = QSpinBox()
            spin.setRange(0, 15)
            saved_bit = saved_channels.get(ch['id'], idx)
            spin.setValue(saved_bit)
            row.addWidget(label)
            row.addWidget(spin)
            self.channel_spinboxes[ch['id']] = spin

            if ch in info.optional_channels:
                check = QCheckBox("Use")
                saved_checked = saved_channels.get(ch['id'] + '_enabled', True)
                check.setChecked(saved_checked)
                check.toggled.connect(lambda checked, s=spin: s.setEnabled(checked))
                row.addWidget(check)
                self.channel_checks[ch['id']] = check
                if not saved_checked:
                    spin.setEnabled(False)

            w = QWidget()
            w.setLayout(row)
            self.channels_form.addRow(w)

        for opt in info.options:
            oid = opt['id']
            desc = opt.get('desc', oid)
            default = opt.get('default')
            values = opt.get('values')
            saved_val = saved_options.get(oid)

            if values is not None:
                combo = QComboBox()
                for v in values:
                    combo.addItem(str(v))
                if saved_val is not None:
                    idx_combo = combo.findText(str(saved_val))
                    if idx_combo >= 0:
                        combo.setCurrentIndex(idx_combo)
                elif default is not None:
                    idx_combo = combo.findText(str(default))
                    if idx_combo >= 0:
                        combo.setCurrentIndex(idx_combo)
                self.option_widgets[oid] = combo
                self.options_form.addRow(QLabel(desc + ":"), combo)
            elif isinstance(default, (int, float)):
                if isinstance(default, int):
                    spin = QSpinBox()
                    spin.setRange(-10**9, 10**9)
                    val = saved_val if saved_val is not None else default
                    spin.setValue(int(val))
                else:
                    spin = QDoubleSpinBox()
                    spin.setRange(-10**9, 10**9)
                    val = saved_val if saved_val is not None else default
                    spin.setValue(float(val))
                self.option_widgets[oid] = spin
                self.options_form.addRow(QLabel(desc + ":"), spin)
            else:
                edit = QLineEdit(str(saved_val) if saved_val is not None else str(default) if default else "")
                self.option_widgets[oid] = edit
                self.options_form.addRow(QLabel(desc + ":"), edit)

    def get_settings(self):
        info = self.current_info
        enabled_indices = []
        enabled_bits = []

        for idx, ch in enumerate(info.all_channels):
            ch_id = ch['id']
            is_optional = ch in info.optional_channels
            if is_optional and ch_id in self.channel_checks:
                if not self.channel_checks[ch_id].isChecked():
                    continue
            enabled_indices.append(idx)
            enabled_bits.append(self.channel_spinboxes[ch_id].value())

        options = {}
        for opt in info.options:
            oid = opt['id']
            w = self.option_widgets.get(oid)
            if w is None:
                continue
            if isinstance(w, QComboBox):
                val = w.currentText()
                try:
                    val = int(val)
                except ValueError:
                    try:
                        val = float(val)
                    except ValueError:
                        pass
                options[oid] = val
            elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
                options[oid] = w.value()
            elif isinstance(w, QLineEdit):
                options[oid] = w.text()

        return {
            'decoder_class': info.module.Decoder,
            'channels': enabled_indices,
            'channel_bits': enabled_bits,
            'num_channels': info.num_channels,
            'options': options,
            'protocol_info': info,
        }

    def validate_and_accept(self):
        bits_used = []
        for ch_id, spin in self.channel_spinboxes.items():
            if ch_id in self.channel_checks and not self.channel_checks[ch_id].isChecked():
                continue
            bits_used.append(spin.value())
        if len(set(bits_used)) != len(bits_used):
            QMessageBox.warning(self, "Channel Conflict", "GPIO bits must be unique for each channel.")
            return

        if self.current_info:
            proto_id = self.current_info.id
            channels_state = {}
            for ch in self.current_info.all_channels:
                ch_id = ch['id']
                spin = self.channel_spinboxes[ch_id]
                channels_state[ch_id] = spin.value()
                if ch in self.current_info.optional_channels:
                    check = self.channel_checks.get(ch_id)
                    if check:
                        channels_state[ch_id + '_enabled'] = check.isChecked()

            options_state = {}
            for opt in self.current_info.options:
                oid = opt['id']
                w = self.option_widgets.get(oid)
                if w is None:
                    continue
                if isinstance(w, QComboBox):
                    val = w.currentText()
                    try:
                        val = int(val)
                    except ValueError:
                        try:
                            val = float(val)
                        except ValueError:
                            pass
                    options_state[oid] = val
                elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
                    options_state[oid] = w.value()
                elif isinstance(w, QLineEdit):
                    options_state[oid] = w.text()

            new_decoder = {
                'id': self.instance_id,
                'proto_id': proto_id,
                'channels': channels_state,
                'options': options_state
            }
            # Обновляем active_decoders
            self.config.active_decoders = [ad for ad in self.config.active_decoders if ad.get('id') != self.instance_id]
            if self.enable_checkbox.isChecked():
                self.config.active_decoders.append(new_decoder)

            # Сохраняем как последние настройки протокола
            self.config.decoder_settings[proto_id] = {
                'channels': channels_state,
                'options': options_state
            }
            self.save_config(self.config)

        self.accept()