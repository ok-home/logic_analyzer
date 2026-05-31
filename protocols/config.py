import os
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

@dataclass
class AnalyzerConfig:
    """Единая конфигурация логического анализатора."""
    data_file: str = "laRowBin.bin"
    port: str = ""
    baudrate: int = 115200
    timeout: float = 120.0

    num_channels: int = 16
    sample_count: int = 10000
    sample_rate: float = 1_000_000.0
    ram_type: int = 0
    timeout_ms: float = 20.0

    trigger_enabled: bool = False
    trigger_gpio: Optional[int] = None
    trigger_edge_num: int = 1
    trigger_edge: str = "posedge"

    gpio: Dict[int, int] = field(default_factory=lambda: {i: -1 for i in range(16)})
    show_channels: List[int] = field(default_factory=lambda: list(range(16)))

    # Список активных декодеров: каждый с уникальным id
    active_decoders: List[Dict] = field(default_factory=list)
    # Формат: {'id': str, 'proto_id': str, 'channels': dict, 'options': dict}

    decoder_settings: Dict = field(default_factory=dict)

    # ... константы и методы validate, _edge_num_to_str, _edge_str_to_num остаются без изменений
    MIN_SAMPLE_RATE = 1
    MAX_SAMPLE_RATE = 100_000_000
    MIN_SAMPLE_COUNT = 1
    MAX_SAMPLE_COUNT = 10_000_000
    MIN_CHANNELS = 1
    MAX_CHANNELS = 16

    def validate(self) -> Tuple[bool, List[str]]:
        errors = []
        if not (self.MIN_CHANNELS <= self.num_channels <= self.MAX_CHANNELS):
            errors.append(f"num_channels must be between {self.MIN_CHANNELS} and {self.MAX_CHANNELS}")
        if not (self.MIN_SAMPLE_RATE <= self.sample_rate <= self.MAX_SAMPLE_RATE):
            errors.append(f"sample_rate must be between {self.MIN_SAMPLE_RATE} and {self.MAX_SAMPLE_RATE}")
        if not (self.MIN_SAMPLE_COUNT <= self.sample_count <= self.MAX_SAMPLE_COUNT):
            errors.append(f"sample_count must be between {self.MIN_SAMPLE_COUNT} and {self.MAX_SAMPLE_COUNT}")
        if self.trigger_edge_num not in (1, 2, 3):
            errors.append("trigger_edge_num must be 1, 2, or 3")
        return len(errors) == 0, errors

    @staticmethod
    def _edge_num_to_str(num: int) -> str:
        return {1: "posedge", 2: "negedge", 3: "both"}.get(num, "posedge")

    @staticmethod
    def _edge_str_to_num(s: str) -> int:
        return {"posedge": 1, "negedge": 2, "both": 3}.get(s, 1)

    @classmethod
    def from_dict(cls, d: dict) -> 'AnalyzerConfig':
        def safe_int(v, default=0):
            try: return int(v)
            except (ValueError, TypeError): return default
        def safe_float(v, default=0.0):
            try: return float(v)
            except (ValueError, TypeError): return default

        gpio = {}
        for i in range(16):
            val = d.get('gpio', {}).get(i, -1)
            gpio[i] = safe_int(val, -1)

        trigger_enabled = d.get('trigger_enabled', False)
        trig_gpio = d.get('trigger_gpio')
        if trig_gpio is not None and str(trig_gpio).strip() not in ('', '-1', 'None'):
            trigger_gpio = safe_int(trig_gpio, -1)
            if trigger_gpio == -1:
                trigger_gpio = None
        else:
            trigger_gpio = None

        edge_num = safe_int(d.get('trigger_edge_num', 1), 1)
        if edge_num not in (1, 2, 3):
            edge_num = 1
        trigger_edge = cls._edge_num_to_str(edge_num)

        num_channels = safe_int(d.get('num_channels', 16), 16)
        show_channels = d.get('show_channels', list(range(num_channels)))

        sample_rate = safe_float(d.get('sample_rate', 1_000_000), 1_000_000)
        sample_count = safe_int(d.get('sample_count', 10000), 10000)

        active_decoders = d.get('active_decoders', [])
        # Убедимся, что у всех есть id
        for i, ad in enumerate(active_decoders):
            if 'id' not in ad:
                ad['id'] = f"inst_{i}"
        decoder_settings = d.get('decoder_settings', {})

        return cls(
            data_file=d.get('data_file', 'laRowBin.bin'),
            port=d.get('port', ''),
            baudrate=safe_int(d.get('baudrate', 115200), 115200),
            timeout=safe_float(d.get('timeout', 120.0), 120.0),
            num_channels=num_channels,
            sample_count=sample_count,
            sample_rate=sample_rate,
            ram_type=safe_int(d.get('ram_type', 0), 0),
            timeout_ms=safe_float(d.get('timeout_ms', 20.0), 20.0),
            trigger_enabled=trigger_enabled,
            trigger_gpio=trigger_gpio,
            trigger_edge_num=edge_num,
            trigger_edge=trigger_edge,
            gpio=gpio,
            show_channels=show_channels,
            active_decoders=active_decoders,
            decoder_settings=decoder_settings
        )

    def to_dict(self) -> dict:
        py_cfg = {
            "dataFile": self.data_file,
            "PortName": self.port,
            "PortSpeed": str(self.baudrate),
            "PortTimeout": str(self.timeout)
        }
        esp_cfg = {f"pin{i}": str(self.gpio.get(i, -1)) for i in range(16)}
        esp_cfg.update({
            "trg": str(self.trigger_gpio) if self.trigger_enabled and self.trigger_gpio is not None else "-1",
            "edg": str(self.trigger_edge_num),
            "smp": str(self.sample_count),
            "clk": str(int(self.sample_rate)),
            "tmo": str(self.timeout_ms),
            "chn": str(self.num_channels),
            "ram": str(self.ram_type)
        })
        full = {
            "PyCfg": py_cfg,
            "EspCfg": esp_cfg,
            "active_decoders": self.active_decoders,
            "decoder_settings": self.decoder_settings
        }
        return full


def load_la_config(cfg_file="la_cfg.json") -> AnalyzerConfig:
    default = AnalyzerConfig()
    if not os.path.exists(cfg_file):
        save_la_config(default, cfg_file)
        return default
    try:
        with open(cfg_file, "r") as f:
            raw = json.load(f)
    except (json.JSONDecodeError, Exception) as e:
        print(f"Error loading config: {e}, using defaults.")
        return default

    py = raw.get("PyCfg", {})
    esp = raw.get("EspCfg", {})
    merged = {
        'data_file': py.get('dataFile', default.data_file),
        'port': py.get('PortName', default.port),
        'baudrate': py.get('PortSpeed', str(default.baudrate)),
        'timeout': py.get('PortTimeout', str(default.timeout)),
        'num_channels': esp.get('chn', str(default.num_channels)),
        'sample_count': esp.get('smp', str(default.sample_count)),
        'sample_rate': esp.get('clk', str(default.sample_rate)),
        'ram_type': esp.get('ram', str(default.ram_type)),
        'timeout_ms': esp.get('tmo', str(default.timeout_ms)),
        'trigger_enabled': esp.get('trg', '-1') != '-1',
        'trigger_gpio': esp.get('trg'),
        'trigger_edge_num': esp.get('edg', '1'),
        'gpio': {i: esp.get(f"pin{i}", "-1") for i in range(16)},
        'show_channels': list(range(int(esp.get('chn', default.num_channels)))),
        'active_decoders': raw.get('active_decoders', []),
        'decoder_settings': raw.get('decoder_settings', {})
    }
    config = AnalyzerConfig.from_dict(merged)
    return config


def save_la_config(config: AnalyzerConfig, cfg_file="la_cfg.json"):
    full = config.to_dict()
    try:
        with open(cfg_file, "w") as f:
            json.dump(full, f, indent=1)
    except Exception as e:
        print(f"Error saving config: {e}")