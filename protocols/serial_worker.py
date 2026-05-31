# serial_worker.py
import json
import serial
from PyQt5.QtCore import QThread, pyqtSignal

class SerialWorker(QThread):
    data_received = pyqtSignal(bytes)
    status_message = pyqtSignal(str)
    transfer_done = pyqtSignal()
    error_occurred = pyqtSignal(str)
    config_received = pyqtSignal(dict)

    def __init__(self, config):
        super().__init__()
        self.config = config          # объект AnalyzerConfig
        self.serial_conn = None
        self.is_running = True
        self._is_stopped = False

    def run(self):
        port_name = self.config.port if self.config.port else ''
        baudrate = self.config.baudrate
        timeout = self.config.timeout

        try:
            self.serial_conn = serial.Serial(port_name, baudrate=baudrate, timeout=timeout)
            self.status_message.emit(f"Connected to {port_name} at {baudrate} baud.")
        except Exception as e:
            self.error_occurred.emit(f"Cannot open port {port_name}: {e}")
            return

        if not self.send_config():
            self.error_occurred.emit("Failed to send configuration.")
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            return

        self.read_loop()

        if self.is_running and self.serial_conn and self.serial_conn.is_open:
            self.get_available_cfg()

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def send_config(self):
        esp_keys = ['pin0','pin1','pin2','pin3','pin4','pin5','pin6','pin7',
                    'pin8','pin9','pin10','pin11','pin12','pin13','pin14','pin15',
                    'trg','edg','smp','clk','tmo','chn','ram']
        # Преобразуем объект AnalyzerConfig в словарь значений для отправки
        values = {}
        for i in range(16):
            values[f"pin{i}"] = str(self.config.gpio.get(i, -1))
        values["trg"] = str(self.config.trigger_gpio) if self.config.trigger_enabled else "-1"
        values["edg"] = str(self.config.trigger_edge_num)
        values["smp"] = str(self.config.sample_count)
        values["clk"] = str(int(self.config.sample_rate))
        values["tmo"] = str(self.config.timeout_ms)
        values["chn"] = str(self.config.num_channels)
        values["ram"] = str(self.config.ram_type)

        try:
            for key in esp_keys:
                msg = values.get(key, "-1")
                line = json.dumps({"key": key, "msg": msg}) + "\n"
                self.serial_conn.write(line.encode('utf-8'))
            self.serial_conn.write(b"endcfg\n")
            return True
        except Exception as e:
            self.error_occurred.emit(f"Error sending config: {e}")
            return False

    def read_loop(self):
        try:
            while self.is_running:
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if not line:
                        continue
                    print(f"DEBUG: received line: '{line}'")
                    if line.startswith("{") and not line.startswith("Start"):
                        try:
                            param = json.loads(line)
                            if "smp" in param:
                                self.config.sample_count = int(param["smp"])
                                self.status_message.emit(f"Device reports sample count: {param['smp']}")
                            if "clk" in param:
                                self.config.sample_rate = int(param["clk"])
                                self.status_message.emit(f"Device reports clock: {param['clk']}")
                            if "chn" in param:
                                self.config.num_channels = int(param["chn"])
                                self.status_message.emit(f"Device reports channels: {param['chn']}")
                        except json.JSONDecodeError:
                            pass
                    elif "Start samples transfer" in line:
                        self.read_samples_data()
                        break
                    elif "Start logic analyzer OK" in line:
                        self.status_message.emit("Logic analyzer started, waiting for data...")
                    elif "Samples transfer done" in line:
                        self.status_message.emit("Samples transfer done.")
                        self.transfer_done.emit()
                        break
                    elif "Start logic analyzer error" in line:
                        self.error_occurred.emit("Start logic analyzer error: invalid configuration")
                        break
                    elif "Error" in line:
                        self.error_occurred.emit(line)
                        break
        except Exception as e:
            self.error_occurred.emit(f"Read error: {e}")

    def read_samples_data(self):
        chn = self.config.num_channels
        smp = self.config.sample_count
        bytes_per_sample = 2 if chn == 16 else 1
        expected = smp * bytes_per_sample
        data = bytearray()
        while len(data) < expected and self.is_running:
            chunk = self.serial_conn.read(expected - len(data))
            if not chunk:
                break
            data.extend(chunk)
        if len(data) > 0:
            data_file = self.config.data_file
            with open(data_file, "wb") as f:
                f.write(data)
            self.status_message.emit(f"Received {len(data)} bytes, saved to {data_file}")
            self.data_received.emit(bytes(data))
        self.transfer_done.emit()

    def get_available_cfg(self):
        try:
            self.serial_conn.write(b"getcfg\n")
            self.status_message.emit("Requesting configuration from analyzer...")
            responses = []
            last_len = 0
            for _ in range(20):
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        responses.append(line)
                        print(f"CFG: {line}")
                        last_len = len(responses)
                else:
                    self.msleep(100)
                if len(responses) == last_len and last_len > 0:
                    self.msleep(500)
                    if self.serial_conn.in_waiting == 0:
                        break
            if responses:
                config_text = "\n".join(responses)
                self.config_received.emit({"text": config_text})
            else:
                self.status_message.emit("No configuration received from analyzer.")
        except Exception as e:
            self.error_occurred.emit(f"Error getting config: {e}")

    def stop(self):
        if self._is_stopped:
            return
        self._is_stopped = True
        self.is_running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.quit()
        self.wait()