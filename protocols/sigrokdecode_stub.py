# sigrokdecode_stub.py
import sys
from collections import deque

OUTPUT_ANN = 'ann'
OUTPUT_PYTHON = 'python'
OUTPUT_BINARY = 'binary'
OUTPUT_META = 'meta'
SRD_CONF_SAMPLERATE = 'samplerate'

class Decoder:
    def __init__(self):
        self.output_protocols = {}
        self.samplerate = None
        self.samples = []           # 16‑битные слова
        self.sample_index = 0
        self.samplenum = 0
        self.output_queue = deque()
        self.channels = []          # индексы каналов (0,1,…)
        self.channel_bits = []      # соответствующие номера битов
        self.options = {}
        self.matched = []

    def register(self, output_type, *args, **kwargs):
        if not hasattr(self, 'output_protocols'):
            self.output_protocols = {}
        output_id = len(self.output_protocols)
        self.output_protocols[output_id] = output_type
        return output_id

    def has_channel(self, channel_index):
        if not hasattr(self, 'channels'):
            self.channels = []
        return channel_index in self.channels

    def start(self):
        pass

    def metadata(self, key, value):
        pass

    def put(self, startsample, endsample, output_type, data):
        if not hasattr(self, 'output_queue'):
            self.output_queue = deque()
        self.output_queue.append((startsample, endsample, output_type, data))

    def _get_channel_values(self, index):
        word = self.samples[index]
        return {ch: (word >> self.channel_bits[ch]) & 1 for ch in self.channels}

    def _get_return_tuple(self, index):
        word = self.samples[index]
        result = []
        for ch in range(self.num_channels):
            if ch in self.channels:
                bit = self.channel_bits[ch]
                result.append((word >> bit) & 1)
            else:
                result.append(None)
        return tuple(result)

    def wait(self, conds=None):
        if not hasattr(self, 'samples'):
            self.samples = []
        if not hasattr(self, 'sample_index'):
            self.sample_index = 0
        if not hasattr(self, 'samplenum'):
            self.samplenum = 0

        if conds is None:
            if self.sample_index < len(self.samples):
                vals = self._get_return_tuple(self.sample_index)
                self.sample_index += 1
                self.samplenum = self.sample_index - 1
                return vals
            else:
                raise StopIteration()

        if isinstance(conds, dict):
            conds = [conds]

        self.matched = [False] * len(conds)
        start_samplenum = self.samplenum

        while self.sample_index < len(self.samples):
            curr_vals = self._get_channel_values(self.sample_index)
            prev_vals = self._get_channel_values(self.sample_index - 1) if self.sample_index > 0 else None

            any_matched = False
            for idx, cond in enumerate(conds):
                if 'skip' in cond:
                    continue
                matched_all = True
                for ch, edge in cond.items():
                    if ch not in curr_vals:
                        matched_all = False
                        break
                    if edge in ('h', 'l'):
                        target = 1 if edge == 'h' else 0
                        if curr_vals[ch] != target:
                            matched_all = False
                            break
                    elif prev_vals is not None and ch in prev_vals:
                        prev = prev_vals[ch]
                        curr = curr_vals[ch]
                        if edge == 'r' and (prev != 0 or curr != 1):
                            matched_all = False
                            break
                        elif edge == 'f' and (prev != 1 or curr != 0):
                            matched_all = False
                            break
                        elif edge == 'e' and prev == curr:
                            matched_all = False
                            break
                        elif edge == 's':
                            pass
                    else:
                        if edge != 's':
                            matched_all = False
                            break
                if matched_all:
                    self.matched[idx] = True
                    any_matched = True

            if any_matched:
                result = self._get_return_tuple(self.sample_index)
                self.sample_index += 1
                self.samplenum = self.sample_index - 1
                return result

            for idx, cond in enumerate(conds):
                if 'skip' in cond:
                    n = cond['skip']
                    if self.samplenum >= start_samplenum + n:
                        self.matched[idx] = True
                        result = self._get_return_tuple(self.sample_index)
                        self.sample_index += 1
                        self.samplenum = self.sample_index - 1
                        return result

            self.sample_index += 1
            self.samplenum = self.sample_index - 1

        raise StopIteration()


def run_decoder(decoder_instance, samples, metadata=None):
    for attr, default in [
        ('output_protocols', {}),
        ('output_queue', deque()),
        ('channels', []),
        ('channel_bits', []),
        ('options', {}),
        ('samplerate', None),
        ('samples', []),
        ('sample_index', 0),
        ('samplenum', 0),
    ]:
        if not hasattr(decoder_instance, attr):
            setattr(decoder_instance, attr, default)

    decoder_instance.samples = samples
    decoder_instance.sample_index = 0
    decoder_instance.samplenum = 0

    if metadata and 'channels' in metadata:
        decoder_instance.channels = metadata['channels']
    else:
        decoder_instance.channels = [0]

    if metadata and 'channel_bits' in metadata:
        decoder_instance.channel_bits = metadata['channel_bits']
    else:
        decoder_instance.channel_bits = metadata.get('channels', [0]) if metadata else [0]

    if metadata and 'num_channels' in metadata:
        decoder_instance.num_channels = metadata['num_channels']
    else:
        max_ch = max(decoder_instance.channels) if decoder_instance.channels else 0
        decoder_instance.num_channels = max(max_ch + 1, 2)

    if metadata and 'samplerate' in metadata:
        decoder_instance.samplerate = float(metadata['samplerate'])

    if hasattr(decoder_instance, 'options') and isinstance(decoder_instance.options, tuple):
        defaults = {opt['id']: opt['default'] for opt in decoder_instance.options}
        decoder_instance.options = defaults
    else:
        if not isinstance(decoder_instance.options, dict):
            decoder_instance.options = {}
    if metadata and 'options' in metadata:
        decoder_instance.options.update(metadata['options'])

    decoder_instance.start()

    if not hasattr(decoder_instance, 'bit_width') and decoder_instance.samplerate and 'baudrate' in decoder_instance.options:
        decoder_instance.bit_width = float(decoder_instance.samplerate) / float(decoder_instance.options['baudrate'])

    try:
        decoder_instance.decode()
    except StopIteration:
        pass

    return list(decoder_instance.output_queue)


sys.modules['sigrokdecode'] = sys.modules[__name__]