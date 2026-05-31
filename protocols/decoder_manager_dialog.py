# decoder_manager_dialog.py
import uuid
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
    QListWidget, QListWidgetItem, QMessageBox, QComboBox,
    QDialogButtonBox, QLabel, QSpinBox, QDoubleSpinBox, QLineEdit
)
from PyQt5.QtCore import pyqtSignal

from protocol_scanner import scan_decoders
from universal_decoder_dialog import UniversalDecoderDialog


class DecoderManagerDialog(QDialog):
    decoders_changed = pyqtSignal()

    def __init__(self, config, save_config_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Manage Protocol Decoders")
        self.setModal(True)
        self.resize(500, 400)
        self.config = config
        self.save_config = save_config_callback
        self.protocols = scan_decoders()

        self.active_decoders = config.active_decoders[:]

        layout = QVBoxLayout()

        self.list_widget = QListWidget()
        self._populate_list()
        layout.addWidget(self.list_widget)

        btn_layout = QHBoxLayout()
        add_btn = QPushButton("Add")
        add_btn.clicked.connect(self.add_decoder)
        edit_btn = QPushButton("Edit")
        edit_btn.clicked.connect(self.edit_decoder)
        remove_btn = QPushButton("Remove")
        remove_btn.clicked.connect(self.remove_decoder)
        btn_layout.addWidget(add_btn)
        btn_layout.addWidget(edit_btn)
        btn_layout.addWidget(remove_btn)
        layout.addLayout(btn_layout)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        self.setLayout(layout)

    def _populate_list(self):
        self.list_widget.clear()
        for i, dec_cfg in enumerate(self.active_decoders):
            proto_id = dec_cfg.get('proto_id', '')
            info = self.protocols.get(proto_id)
            name = info.name if info else proto_id
            inst_id = dec_cfg.get('id', '?')
            # Показываем сокращённый ID для различия
            short_id = inst_id[:8] if inst_id else '?'
            num_ch = len(dec_cfg.get('channels', {}))
            item = QListWidgetItem(f"{name} ({short_id})  channels: {num_ch}")
            item.setData(1, i)
            self.list_widget.addItem(item)

    def add_decoder(self):
        if not self.protocols:
            QMessageBox.warning(self, "No decoders", "No protocol decoders found.")
            return

        proto_id, ok = self._choose_protocol()
        if not ok:
            return

        info = self.protocols[proto_id]
        # Генерируем новый instance_id
        instance_id = str(uuid.uuid4())

        # Пытаемся загрузить сохранённые настройки для этого протокола (последние использованные)
        saved_cfg = self.config.decoder_settings.get(proto_id, {})
        saved_channels = saved_cfg.get('channels', {})
        saved_options = saved_cfg.get('options', {})

        if not saved_channels:
            saved_channels = {ch['id']: i for i, ch in enumerate(info.all_channels)}
            for ch in info.optional_channels:
                saved_channels[ch['id'] + '_enabled'] = True
        if not saved_options:
            saved_options = {opt['id']: opt.get('default', '') for opt in info.options}

        dlg = UniversalDecoderDialog(self, config=self.config, save_config_callback=None,
                                     instance_id=instance_id)
        dlg.setWindowTitle(f"Add {info.name} ({instance_id[:8]})")
        dlg.protocol_combo.setCurrentText(proto_id)
        dlg.enable_checkbox.setChecked(True)
        self._apply_settings_to_dialog(dlg, saved_channels, saved_options)

        if dlg.exec_():
            self._save_from_dialog(dlg, proto_id, instance_id)
            self._populate_list()
            self.decoders_changed.emit()

    def edit_decoder(self):
        current_item = self.list_widget.currentItem()
        if not current_item:
            QMessageBox.information(self, "No selection", "Select a decoder to edit.")
            return
        idx = current_item.data(1)
        dec_cfg = self.active_decoders[idx]
        proto_id = dec_cfg['proto_id']
        instance_id = dec_cfg.get('id')
        info = self.protocols.get(proto_id)
        if not info:
            return

        dlg = UniversalDecoderDialog(self, config=self.config, save_config_callback=None,
                                     instance_id=instance_id)
        dlg.setWindowTitle(f"Edit {info.name} ({instance_id[:8]})")
        dlg.protocol_combo.setCurrentText(proto_id)
        self._apply_settings_to_dialog(dlg, dec_cfg.get('channels', {}), dec_cfg.get('options', {}))
        dlg.enable_checkbox.setChecked(True)

        if dlg.exec_():
            self._save_from_dialog(dlg, proto_id, instance_id)
            self._populate_list()
            self.decoders_changed.emit()

    def remove_decoder(self):
        current_item = self.list_widget.currentItem()
        if not current_item:
            return
        idx = current_item.data(1)
        inst_id = self.active_decoders[idx]['id']
        del self.active_decoders[idx]
        self.config.active_decoders = self.active_decoders[:]
        if self.save_config:
            self.save_config(self.config)
        # Очищаем сохранённые настройки для этого instance? Лучше оставить, но можно и удалить.
        # Не будем удалять из decoder_settings, чтобы пользователь мог восстановить, но при новом
        # добавлении он получит последние настройки протокола (см. add_decoder).
        self._populate_list()
        self.decoders_changed.emit()

    def _apply_settings_to_dialog(self, dlg, channels, options):
        for ch_id, spin in dlg.channel_spinboxes.items():
            if ch_id in channels:
                spin.setValue(int(channels[ch_id]))
        for ch_id, check in dlg.channel_checks.items():
            key = ch_id + '_enabled'
            if key in channels:
                check.setChecked(bool(channels[key]))
                if ch_id in dlg.channel_spinboxes:
                    dlg.channel_spinboxes[ch_id].setEnabled(bool(channels[key]))
        for oid, w in dlg.option_widgets.items():
            if oid in options:
                val = options[oid]
                if isinstance(w, QComboBox):
                    idx = w.findText(str(val))
                    if idx >= 0:
                        w.setCurrentIndex(idx)
                elif isinstance(w, QSpinBox):
                    try:
                        w.setValue(int(val))
                    except (ValueError, TypeError):
                        pass
                elif isinstance(w, QDoubleSpinBox):
                    try:
                        w.setValue(float(val))
                    except (ValueError, TypeError):
                        pass
                elif isinstance(w, QLineEdit):
                    w.setText(str(val))

    def _save_from_dialog(self, dlg, proto_id, instance_id):
        info = dlg.current_info
        channels_state = {}
        for ch in info.all_channels:
            ch_id = ch['id']
            spin = dlg.channel_spinboxes[ch_id]
            channels_state[ch_id] = spin.value()
            if ch in info.optional_channels:
                check = dlg.channel_checks.get(ch_id)
                if check:
                    channels_state[ch_id + '_enabled'] = check.isChecked()
        options_state = {}
        for opt in info.options:
            oid = opt['id']
            w = dlg.option_widgets.get(oid)
            if w is None:
                continue
            if isinstance(w, QComboBox):
                options_state[oid] = w.currentText()
            elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
                options_state[oid] = w.value()
            elif isinstance(w, QLineEdit):
                options_state[oid] = w.text()

        new_decoder = {
            'id': instance_id,
            'proto_id': proto_id,
            'channels': channels_state,
            'options': options_state
        }
        # Удаляем старый экземпляр с таким же id (если редактировали)
        self.active_decoders = [ad for ad in self.active_decoders if ad.get('id') != instance_id]
        if dlg.enable_checkbox.isChecked():
            self.active_decoders.append(new_decoder)
        self.config.active_decoders = self.active_decoders[:]
        # Сохраняем последние настройки протокола в decoder_settings (по proto_id для удобства)
        self.config.decoder_settings[proto_id] = {
            'channels': channels_state,
            'options': options_state
        }
        if self.save_config:
            self.save_config(self.config)

    def _choose_protocol(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Select Protocol")
        layout = QVBoxLayout()
        combo = QComboBox()
        for pid, info in self.protocols.items():
            combo.addItem(f"{info.name} ({pid})", pid)
        layout.addWidget(QLabel("Choose protocol:"))
        layout.addWidget(combo)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        dialog.setLayout(layout)
        if dialog.exec_() == QDialog.Accepted:
            return combo.currentData(), True
        return None, False

    def accept(self):
        super().accept()