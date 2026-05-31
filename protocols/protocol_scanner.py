# protocol_scanner.py
import os
import sys
import importlib
from collections import OrderedDict

class ProtocolInfo:
    def __init__(self, mod_name, module):
        self.mod_name = mod_name
        self.module = module
        self.decoder_class = getattr(module, 'Decoder', None)
        if self.decoder_class is None:
            raise ValueError(f"Module {mod_name} does not contain Decoder class")

        self.id = getattr(self.decoder_class, 'id', mod_name)
        self.name = getattr(self.decoder_class, 'name', self.id)
        self.longname = getattr(self.decoder_class, 'longname', self.name)
        self.desc = getattr(self.decoder_class, 'desc', '')

        self.channels = getattr(self.decoder_class, 'channels', ())
        self.optional_channels = getattr(self.decoder_class, 'optional_channels', ())
        self.options = getattr(self.decoder_class, 'options', ())
        self.annotations = getattr(self.decoder_class, 'annotations', ())
        self.annotation_rows = getattr(self.decoder_class, 'annotation_rows', ())
        self.binary = getattr(self.decoder_class, 'binary', ())

    @property
    def all_channels(self):
        return list(self.channels) + list(self.optional_channels)

    @property
    def num_channels(self):
        return len(self.all_channels)


def scan_decoders(decoders_path='decoders'):
    protocols = OrderedDict()
    if not os.path.isdir(decoders_path):
        print(f"Folder {decoders_path} not found")
        return protocols

    if decoders_path not in sys.path:
        sys.path.insert(0, decoders_path)

    # Удаляем ранее загруженные модули из этой директории для возможности перезагрузки
    prefix = 'decoders.'
    to_remove = [m for m in sys.modules if m.startswith(prefix)]
    for m in to_remove:
        del sys.modules[m]

    entries = sorted(os.listdir(decoders_path))
    for entry in entries:
        entry_path = os.path.join(decoders_path, entry)
        if not os.path.isdir(entry_path) or entry.startswith('_') or entry.startswith('.'):
            continue
        pd_file = os.path.join(entry_path, 'pd.py')
        if not os.path.isfile(pd_file):
            continue
        try:
            module = importlib.import_module(f"{entry}.pd")
            info = ProtocolInfo(entry, module)
            protocols[info.id] = info
            print(f"Loaded protocol: {info.id} ({info.name})")
        except Exception as e:
            print(f"Error loading protocol {entry}: {e}")
    return protocols