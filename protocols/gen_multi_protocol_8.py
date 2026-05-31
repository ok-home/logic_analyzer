#!/usr/bin/env python3
"""
Генератор 8-битного бинарного файла для логического анализатора.
Каналы (один байт на сэмпл, LSB-first):
  D0 – UART RX  (115200 8N1, "this is rx channel")
  D1 – UART TX  (115200 8N1, "this is tx channel")
  D2 – I2C SDA  (100 кГц)
  D3 – I2C SCL
  D4 – SPI SCK  (1 МГц, режим 0)
  D5 – SPI MOSI
  D6 – SPI MISO
  D7 – SPI SS
"""

import struct

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
            p = 0 if (ones % 2 == 0) else 1 if parity == 'E' else (1 if (ones % 2 == 0) else 0)
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

# ========================== SPI ==========================
def make_spi_segments(data_bytes, spi_freq, cpol=0, cpha=0, idle_high=True):
    IDLE = 1 if idle_high else 0
    T = 1.0 / spi_freq
    T_half = T / 2.0
    SCK_IDLE = 0 if idle_high else 1

    sck_seg, mosi_seg, miso_seg, ss_seg = [], [], [], []

    def set_lines(sck, mosi, miso, ss, dur):
        sck_seg.append((sck, dur))
        mosi_seg.append((mosi, dur))
        miso_seg.append((miso, dur))
        ss_seg.append((ss, dur))

    # Начальное состояние: SCK = 0, остальные IDLE (высокий)
    set_lines(SCK_IDLE, IDLE, IDLE, IDLE, 0.0001)

    # Активация SS (низкий уровень)
    SS_ACTIVE = 0 if idle_high else 1
    set_lines(SCK_IDLE, IDLE, IDLE, SS_ACTIVE, T_half * 2)

    for byte in data_bytes:
        for bit_pos in range(7, -1, -1):
            bit = (byte >> bit_pos) & 1
            mosi_val = bit if idle_high else (1 - bit)
            # SCK low, данные на MOSI
            set_lines(SCK_IDLE, mosi_val, IDLE, SS_ACTIVE, T_half)
            # SCK high (захват)
            set_lines(1 if idle_high else 0, mosi_val, IDLE, SS_ACTIVE, T_half)
            # SCK low
            set_lines(SCK_IDLE, mosi_val, IDLE, SS_ACTIVE, T_half)

    # Снятие SS
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

    # SPI
    SPI_FREQ = 1_000_000
    SPI_DATA = [0x9F, 0x00, 0x00]
    seg_spi_sck, seg_spi_mosi, seg_spi_miso, seg_spi_ss = make_spi_segments(
        SPI_DATA, SPI_FREQ, cpol=0, cpha=0, idle_high=True)
    time_spi = sum(dur for _, dur in seg_spi_sck)

    # Общая длительность (максимальная)
    total_time = max(time_uart_rx, time_uart_tx, time_i2c, time_spi) + dt
    num_samples = int(total_time / dt) + 1

    # Дискретизация всех каналов
    rx_samp    = sample_segments(seg_uart_rx, dt, num_samples)
    tx_samp    = sample_segments(seg_uart_tx, dt, num_samples)
    sda_samp   = sample_segments(seg_sda, dt, num_samples)
    scl_samp   = sample_segments(seg_scl, dt, num_samples)
    spi_sck_s  = sample_segments(seg_spi_sck, dt, num_samples)
    spi_mosi_s = sample_segments(seg_spi_mosi, dt, num_samples)
    spi_miso_s = sample_segments(seg_spi_miso, dt, num_samples)
    spi_ss_s   = sample_segments(seg_spi_ss, dt, num_samples)

    # Упаковка в 8-битные слова (каждый бит соответствует одному каналу)
    packed = b''
    for rx, tx, sda, scl, ssck, mosi, miso, ss in zip(
        rx_samp, tx_samp, sda_samp, scl_samp,
        spi_sck_s, spi_mosi_s, spi_miso_s, spi_ss_s):
        word = (ss   << 7) | (miso << 6) | (mosi << 5) | (ssck << 4) | \
               (scl  << 3) | (sda  << 2) | (tx   << 1) | rx
        packed += struct.pack('B', word)   # 8-битная упаковка

    filename = 'multi_protocol_8bit.bin'
    with open(filename, 'wb') as f:
        f.write(packed)

    print(f"Файл: {filename}")
    print(f"Частота дискретизации: {SAMPLE_RATE/1e6:.0f} МГц")
    print(f"Количество выборок: {len(packed)}")
    print(f"Длительность: {len(packed)/SAMPLE_RATE:.6f} с\n")

    print("Распределение каналов (8 бит):")
    print("  D0 – UART RX      (115200 8N1, \"this is rx channel\")")
    print("  D1 – UART TX      (115200 8N1, \"this is tx channel\")")
    print(f"  D2 – I2C SDA      (100 кГц)")
    print(f"  D3 – I2C SCL")
    print(f"  D4 – SPI SCK      ({SPI_FREQ/1e6} МГц, режим 0)")
    print(f"  D5 – SPI MOSI     (передача 0x{' '.join(f'{b:02X}' for b in SPI_DATA)})")
    print(f"  D6 – SPI MISO     (высокий уровень)")
    print(f"  D7 – SPI SS       (активный низкий)")

    print("\nИмпорт в PulseView:")
    print("  1. File → Import → Raw binary logic data")
    print("     - Bits: 8, Bit order: LSB first, Sample rate: 10 MHz")
    print("  2. Декодеры:")
    print("     - UART: каналы D0 (RX), D1 (TX), скорость 115200, 8N1")
    print("     - I2C:  каналы D2 (SDA), D3 (SCL)")
    print("     - SPI:  каналы D4 (SCK), D5 (MOSI), D6 (MISO), D7 (SS)")
    print("             Режим 0 (CPOL=0, CPHA=0), частота 1 МГц")