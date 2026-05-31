import serial
import time
import struct
from PyQt5.QtCore import QThread, pyqtSignal

class SerialWorker(QThread):
    status_message = pyqtSignal(str)
    error_occurred = pyqtSignal(str)
    data_received = pyqtSignal(bytes)
    transfer_done = pyqtSignal()
    config_received = pyqtSignal(dict)

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self._running = False
        self.ser = None

    def run(self):
        self._running = True
        port = self.config.get('port', 'COM3')
        baudrate = self.config.get('baudrate', 115200)
        timeout = self.config.get('timeout', 120)
        try:
            self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            self.status_message.emit(f"Connected to {port}")
        except Exception as e:
            self.error_occurred.emit(f"Serial error: {e}")
            return

        try:
            # Здесь должен быть протокол общения с анализатором
            # В данной заглушке просто читаем данные и эмулируем прием
            self.status_message.emit("Receiving data...")
            data = bytearray()
            while self._running:
                if self.ser.in_waiting:
                    chunk = self.ser.read(self.ser.in_waiting)
                    data.extend(chunk)
                else:
                    break
            if data:
                self.data_received.emit(bytes(data))
            self.transfer_done.emit()
        except Exception as e:
            self.error_occurred.emit(f"Acquisition error: {e}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()

    def stop(self):
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.wait()