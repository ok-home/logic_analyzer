import os
import json

def load_la_config(cfg_file="la_cfg.json"):
    default = {
        "PyCfg": {
            "dataFile": "laRowBin.bin",
            "PortName": "",
            "PortSpeed": "115200",
            "PortTimeout": "120"
        },
        "EspCfg": {
            "pin0": "-1", "pin1": "-1", "pin2": "-1", "pin3": "-1",
            "pin4": "-1", "pin5": "-1", "pin6": "-1", "pin7": "-1",
            "pin8": "-1", "pin9": "-1", "pin10": "-1", "pin11": "-1",
            "pin12": "-1", "pin13": "-1", "pin14": "-1", "pin15": "-1",
            "trg": "-1", "edg": "1", "smp": "10000", "clk": "1000000",
            "tmo": "20", "chn": "16", "ram": "0"
        }
    }
    if not os.path.exists(cfg_file):
        with open(cfg_file, "w") as f:
            json.dump(default, f, indent=1)
    try:
        with open(cfg_file, "r") as f:
            full_cfg = json.load(f)
    except:
        full_cfg = default
    py_cfg = full_cfg.get("PyCfg", {})
    esp_cfg = full_cfg.get("EspCfg", {})
    config = {
        'data_file': py_cfg.get("dataFile", "laRowBin.bin"),
        'port': py_cfg.get("PortName", ""),
        'baudrate': int(py_cfg.get("PortSpeed", 115200)),
        'timeout': float(py_cfg.get("PortTimeout", 120)),
        'num_channels': int(esp_cfg.get("chn", 16)),
        'sample_count': int(esp_cfg.get("smp", 10000)),
        'sample_rate': float(esp_cfg.get("clk", 1000000)),
        'ram_type': int(esp_cfg.get("ram", 0)),
        'timeout_ms': float(esp_cfg.get("tmo", 20)),
        'trigger_enabled': esp_cfg.get("trg") != "-1",
        'trigger_gpio': esp_cfg.get("trg") if esp_cfg.get("trg") != "-1" else None,
        'gpio': {i: esp_cfg.get(f"pin{i}", "-1") for i in range(16)},
        'show_channels': list(range(int(esp_cfg.get("chn", 16)))),
        # Новый ключ для хранения настроек декодеров
        'decoder_settings': full_cfg.get('decoder_settings', {})
    }
    edge_num = int(esp_cfg.get("edg", 1))
    if edge_num == 0:
        edge_num = 1
    elif edge_num == 1:
        edge_num = 2
    elif edge_num == 2:
        edge_num = 3
    config['trigger_edge_num'] = edge_num
    if edge_num == 1:
        config['trigger_edge'] = "posedge"
    elif edge_num == 2:
        config['trigger_edge'] = "negedge"
    else:
        config['trigger_edge'] = "both"
    return config

def save_la_config(config, cfg_file="la_cfg.json"):
    py_cfg = {
        "dataFile": config.get('data_file', 'laRowBin.bin'),
        "PortName": config.get('port', ''),
        "PortSpeed": str(config.get('baudrate', 115200)),
        "PortTimeout": str(config.get('timeout', 120))
    }
    esp_cfg = {f"pin{i}": config.get('gpio', {}).get(i, "-1") for i in range(16)}
    esp_cfg.update({
        "trg": config.get('trigger_gpio', "-1") if config.get('trigger_enabled') else "-1",
        "edg": str(config.get('trigger_edge_num', 1)),
        "smp": str(config.get('sample_count', 10000)),
        "clk": str(int(config.get('sample_rate', 1000000))),
        "tmo": str(config.get('timeout_ms', 20)),
        "chn": str(config.get('num_channels', 16)),
        "ram": str(config.get('ram_type', 0))
    })
    full = {
        "PyCfg": py_cfg,
        "EspCfg": esp_cfg,
        "decoder_settings": config.get('decoder_settings', {})
    }
    with open(cfg_file, "w") as f:
        json.dump(full, f, indent=1)