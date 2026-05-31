#!/usr/bin/env python3
"""
Генератор многосигнального бинарного файла для логического анализатора.
Каналы (16 бит, little-endian):
  D0 – UART RX (115200 8N1, "this is rx channel")
  D1 – UART TX (115200 8N1, "this is tx channel")
  D2 – I2C SDA (100 кГц, запись 0x12 0x34 0x56 на адрес 0x50)
  D3 – I2C SCL
  D4 – I2S SCK (1.536 МГц)
  D5 – I2S WS (48 кГц)
  D6 – I2S SD (16-бит стерео тестовый сигнал)
  D7 – SPI SCK (1 МГц, режим 0)
  D8 – SPI MOSI
  D9 – SPI MISO (высокий уровень)
  D10 – SPI SS (активный низкий)
  D11..D15 – 0
"""

import struct
import math

# ========================== UART ==========================
def make_uart_segments(message, baud, data_bits, parity, stop_bits, idle_high):
    IDLE = 1 if idle_high else 0
    START = 0 if idle_high else 1
    STOP = 1 if idle_high else 0
    bit_time = 1.0 / baud

    seg = [(IDLE, 0.0002)]
    for byte in message.encode('ascii'):
        seg.append((START, bit_time))
        for i in range(data_bits):
            bit = (byte >> i) & 1
            line = bit if idle_high else (1 - bit)
            seg.append((line, bit_time))
        if parity != 'N':
            ones = sum((byte >> i) & 1 for i in range(data_bits))
            if parity == 'E':
                p = 0 if (ones % 2 == 0) else 1
            else:  # 'O'
                p = 1 if (ones % 2 == 0) else 0
            line = p if idle_high else (1 - p)
            seg.append((line, bit_time))
        seg.append((STOP, stop_bits * bit_time))
    seg.append((IDLE, 0.0002))
    return seg

# ========================== I2C ==========================
def make_i2c_write_segments(dev_addr, data_bytes, i2c_freq, idle_high=True):
    IDLE = 1 if idle_high else 0
    T = 1.0 / i2c_freq
    T_half = T / 2.0

    sda_seg, scl_seg = [], []

    def set_lines(sda, scl, duration):
        sda_seg.append((sda, duration))
        scl_seg.append((scl, duration))

    set_lines(IDLE, IDLE, 0.0001)
    # Start
    set_lines(IDLE, IDLE, T_half)
    set_lines(IDLE, 1 if idle_high else 0, T_half)
    set_lines(0 if idle_high else 1, 1 if idle_high else 0, T_half)
    set_lines(0 if idle_high else 1, 0 if idle_high else 0, T_half)

    addr_byte = (dev_addr << 1) | 0
    for byte in [addr_byte] + data_bytes:
        for bit_pos in range(7, -1, -1):
            bit = (byte >> bit_pos) & 1
            sda_level = bit if idle_high else (1 - bit)
            set_lines(sda_level, 0 if idle_high else 1, T_half)
            set_lines(sda_level, 1 if idle_high else 0, T_half)
            set_lines(sda_level, 0 if idle_high else 1, T_half)
        # ACK
        ack = 0 if idle_high else 1
        set_lines(ack, 0 if idle_high else 1, T_half)
        set_lines(ack, 1 if idle_high else 0, T_half)
        set_lines(ack, 0 if idle_high else 1, T_half)

    # Stop
    set_lines(0 if idle_high else 1, 0 if idle_high else 1, T_half)
    set_lines(0 if idle_high else 1, 1 if idle_high else 0, T_half)
    set_lines(IDLE, 1 if idle_high else 0, T_half)
    set_lines(IDLE, 0 if idle_high else 1, T_half)
    set_lines(IDLE, IDLE, 0.0001)
    return sda_seg, scl_seg

# ========================== I2S ==========================
def make_i2s_segments(samples_left, samples_right, sample_rate, bit_depth, idle_high=True):
    IDLE = 1 if idle_high else 0
    sck_freq = 2 * bit_depth * sample_rate
    T_sck = 1.0 / sck_freq
    T_half = T_sck / 2.0

    sck_seg, ws_seg, sd_seg = [], [], []

    def append_triple(sck, ws, sd, dur):
        sck_seg.append((sck, dur))
        ws_seg.append((ws, dur))
        sd_seg.append((sd, dur))

    append_triple(IDLE, IDLE, IDLE, 0.0001)

    def transmit_word(sample, ws_level):
        # Один пустой такт (задержка данных в I2S)
        append_triple(0 if idle_high else 1, ws_level, IDLE, T_half)
        append_triple(1 if idle_high else 0, ws_level, IDLE, T_half)
        # 16 бит данных
        for i in range(bit_depth-1, -1, -1):
            bit = (sample >> i) & 1
            sd_val = bit if idle_high else (1 - bit)
            append_triple(0 if idle_high else 1, ws_level, sd_val, T_half)
            append_triple(1 if idle_high else 0, ws_level, sd_val, T_half)

    n = min(len(samples_left), len(samples_right))
    for i in range(n):
        ws_left = 0 if idle_high else 1
        transmit_word(samples_left[i], ws_left)
        ws_right = 1 if idle_high else 0
        transmit_word(samples_right[i], ws_right)

    append_triple(IDLE, IDLE, IDLE, 0.0001)
    return sck_seg, ws_seg, sd_seg

# ========================== SPI ==========================
def make_spi_segments(data_bytes, spi_freq, cpol=0, cpha=0, idle_high=True):
    """
    Генерирует сегменты для SCK, MOSI, MISO, SS.
    Режим 0: CPOL=0 (SCK idle low), CPHA=0 (данные выставляются по спаду SCK,
    захват по нарастающему фронту). SS активен низким.
    data_bytes – список байт для передачи.
    Возвращает (sck_seg, mosi_seg, miso_seg, ss_seg).
    """
    IDLE = 1 if idle_high else 0
    T = 1.0 / spi_freq
    T_half = T / 2.0

    sck_seg, mosi_seg, miso_seg, ss_seg = [], [], [], []

    def set_lines(sck, mosi, miso, ss, dur):
        sck_seg.append((sck, dur))
        mosi_seg.append((mosi, dur))
        miso_seg.append((miso, dur))
        ss_seg.append((ss, dur))

    # Начальное состояние: SCK = 0, MOSI = IDLE, MISO = IDLE (высокий), SS = IDLE (неактивный высокий)
    SCK_IDLE = 0 if idle_high else 1
    set_lines(SCK_IDLE, IDLE, IDLE, IDLE, 0.0001)

    # Активация SS (переход в низкий уровень)
    SS_ACTIVE = 0 if idle_high else 1
    set_lines(SCK_IDLE, IDLE, IDLE, SS_ACTIVE, T_half * 2)  # небольшая пауза перед передачей

    # Передача байтов подряд
    for byte in data_bytes:
        for bit_pos in range(7, -1, -1):  # MSB first
            bit = (byte >> bit_pos) & 1
            mosi_val = bit if idle_high else (1 - bit)
            # CPHA=0: данные выставляются по спаду SCK (когда SCK переходит из 1 в 0)
            # и остаются стабильными на всём периоде высокого уровня SCK.
            # Обычно: SCK = 0 в начале, выставляем данные, затем SCK ->1 (захват), затем SCK->0 (след. данные)
            # Но так как SCK изначально 0, нужно начать с данных, потом импульс.
            # Схема: Устанавливаем MOSI, потом SCK high, потом SCK low.
            # Первый бит выставляем сразу после активации SS, когда SCK низкий.
            set_lines(SCK_IDLE, mosi_val, IDLE, SS_ACTIVE, T_half)      # SCK low, данные на MOSI
            set_lines(1 if idle_high else 0, mosi_val, IDLE, SS_ACTIVE, T_half)  # SCK high (захват)
            set_lines(SCK_IDLE, mosi_val, IDLE, SS_ACTIVE, T_half)      # SCK low (подготовка к след. биту)

    # После всех байт: возвращаем SS в высокий уровень
    set_lines(SCK_IDLE, IDLE, IDLE, IDLE, T_half * 2)
    set_lines(SCK_IDLE, IDLE, IDLE, IDLE, 0.0001)

    return sck_seg, mosi_seg, miso_seg, ss_seg

# ========================== Дискретизация ==========================
def sample_segments(segments, dt, num_samples):
    samples = []
    seg_idx = 0
    time_in_seg = 0.0
    level = segments[0][0] if segments else 0
    for i in range(num_samples):
        t = i * dt
        while seg_idx < len(segments) and t >= time_in_seg + segments[seg_idx][1]:
            time_in_seg += segments[seg_idx][1]
            seg_idx += 1
            if seg_idx < len(segments):
                level = segments[seg_idx][0]
        samples.append(level)
    return samples

# ========================== Главная программа ==========================
if __name__ == '__main__':
    SAMPLE_RATE = 10_000_000   # 10 МГц
    dt = 1.0 / SAMPLE_RATE

    # UART
    BAUD = 115200
    MESSAGE_RX = "this is rx channel"
    MESSAGE_TX = "this is tx channel"
    seg_uart_rx = make_uart_segments(MESSAGE_RX, BAUD, 8, 'N', 1.0, idle_high=True)
    seg_uart_tx = make_uart_segments(MESSAGE_TX, BAUD, 8, 'N', 1.0, idle_high=True)
    time_uart_rx = sum(dur for _, dur in seg_uart_rx)
    time_uart_tx = sum(dur for _, dur in seg_uart_tx)

    # I2C
    I2C_FREQ = 100_000
    I2C_DEV_ADDR = 0x50
    I2C_DATA = [0x12, 0x34, 0x56]
    seg_sda, seg_scl = make_i2c_write_segments(I2C_DEV_ADDR, I2C_DATA, I2C_FREQ, idle_high=True)
    time_i2c = sum(dur for _, dur in seg_sda)

    # I2S
    AUDIO_SAMPLE_RATE = 48000
    BIT_DEPTH = 16
    left_samples = [int(32767 * math.sin(2 * math.pi * 440 * i / AUDIO_SAMPLE_RATE)) & 0xFFFF
                    for i in range(20)]
    right_samples = [0x5555] * len(left_samples)
    seg_sck, seg_ws, seg_sd = make_i2s_segments(left_samples, right_samples,
                                                 AUDIO_SAMPLE_RATE, BIT_DEPTH, idle_high=True)
    time_i2s = sum(dur for _, dur in seg_sck)

    # SPI
    SPI_FREQ = 1_000_000  # 1 МГц
    SPI_DATA = [0x9F, 0x00, 0x00]  # пример: чтение ID флеш-памяти
    seg_spi_sck, seg_spi_mosi, seg_spi_miso, seg_spi_ss = make_spi_segments(
        SPI_DATA, SPI_FREQ, cpol=0, cpha=0, idle_high=True)
    time_spi = sum(dur for _, dur in seg_spi_sck)

    # Общая длительность
    total_time = max(time_uart_rx, time_uart_tx, time_i2c, time_i2s, time_spi) + dt
    num_samples = int(total_time / dt) + 1

    # Дискретизация всех каналов
    rx_samp    = sample_segments(seg_uart_rx, dt, num_samples)
    tx_samp    = sample_segments(seg_uart_tx, dt, num_samples)
    sda_samp   = sample_segments(seg_sda, dt, num_samples)
    scl_samp   = sample_segments(seg_scl, dt, num_samples)
    sck_samp   = sample_segments(seg_sck, dt, num_samples)
    ws_samp    = sample_segments(seg_ws, dt, num_samples)
    sd_samp    = sample_segments(seg_sd, dt, num_samples)
    spi_sck_s  = sample_segments(seg_spi_sck, dt, num_samples)
    spi_mosi_s = sample_segments(seg_spi_mosi, dt, num_samples)
    spi_miso_s = sample_segments(seg_spi_miso, dt, num_samples)
    spi_ss_s   = sample_segments(seg_spi_ss, dt, num_samples)

    # Упаковка в 16-битные слова
    packed = b''
    for rx, tx, sda, scl, sck, ws, sd, ssck, mosi, miso, ss in zip(
        rx_samp, tx_samp, sda_samp, scl_samp, sck_samp, ws_samp, sd_samp,
        spi_sck_s, spi_mosi_s, spi_miso_s, spi_ss_s):
        word = (ss  << 10) | (miso << 9) | (mosi << 8) | (ssck << 7) | \
               (sd  << 6)  | (ws   << 5) | (sck  << 4) | (scl  << 3) | \
               (sda << 2)  | (tx   << 1) | rx
        packed += struct.pack('<H', word)

    filename = 'multi_protocol_signal.bin'
    with open(filename, 'wb') as f:
        f.write(packed)

    print(f"Файл: {filename}")
    print(f"Частота дискретизации: {SAMPLE_RATE/1e6} МГц")
    print(f"Количество выборок: {len(packed)//2}")
    print(f"Длительность: {len(packed)/2/SAMPLE_RATE:.6f} с\n")

    print("Распределение каналов:")
    print("  D0  – UART RX      (115200 8N1, \"this is rx channel\")")
    print("  D1  – UART TX      (115200 8N1, \"this is tx channel\")")
    print(f"  D2  – I2C SDA      ({I2C_FREQ/1000:.0f} кГц)")
    print(f"  D3  – I2C SCL")
    print(f"  D4  – I2S SCK      ({2*BIT_DEPTH*AUDIO_SAMPLE_RATE/1000:.1f} кГц)")
    print(f"  D5  – I2S WS       ({AUDIO_SAMPLE_RATE/1000:.1f} кГц)")
    print(f"  D6  – I2S SD       ({BIT_DEPTH}-бит стерео тест)")
    print(f"  D7  – SPI SCK      ({SPI_FREQ/1e6} МГц, режим 0)")
    print(f"  D8  – SPI MOSI     (передача 0x{' '.join(f'{b:02X}' for b in SPI_DATA)})")
    print(f"  D9  – SPI MISO     (высокий уровень)")
    print(f"  D10 – SPI SS       (активный низкий)")
    print("  D11..D15 – 0")

    print("\nИмпорт в PulseView:")
    print("  1. File → Import → Raw binary logic data")
    print("     - Bits: 16, Bit order: LSB first, Sample rate: 10 MHz")
    print("  2. Декодеры:")
    print("     - UART: каналы D0 (RX), D1 (TX), скорость 115200, 8N1")
    print("     - I2C:  каналы D2 (SDA), D3 (SCL)")
    print("     - I2S:  каналы D4 (SCK), D5 (WS), D6 (SD)")
    print("     - SPI:  каналы D7 (SCK), D8 (MOSI), D9 (MISO), D10 (SS), режим 0, 1 МГц")