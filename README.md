[Ru](/README-RU.md)

| Supported Targets |  
| ESP32 ESP32S3 ESP32C3 ESP32P4 | 
| ----------------- |

# Logic analyzer on ESP32 for self-diagnostics
 - No need to buy and connect an external logic analyzer
 - Connected as a component to your program (ESP IDF)
 - Displaying information on the WEB interface or Sigrok PulseView
 - Supported SOC -> ESP32, ESP32S3, ESP32C3, ESP32P4

![WebSocket](/la_ws.jpg)
![PulseView](/sigrok_esp.jpg)

## Main parameters
|                      | Channels | Max<br>Sample<br>Count(2) | Max<br>Sample<br>Rate | ESP<br>Module |        Free GPIO & Clock Source<br>Required        |
|:--------------------:|:--------:|:-------------------------:|:---------------------:|:-------------:|:--------------------------------------------------:|
|         ESP32        |    16    |          50 000           |         40 MHz        |   I2S0/I2S1   | NO                                                 |
|        ESP32S3       |  8<br>16 |     140 000<br>70 000     |    80 MHz<br>40 MHz   |    LCD_CAM    | One Free GPIO<br>One LEDC Channel for slow PCLK(3) |
| ESP32S3<br>PSRAM8(1) |  8<br>16 |   8 000 000<br>4 000 000  |    10 MHz<br>5 MHZ    |    LCD_CAM    | One Free GPIO<br>One LEDC Channel for slow PCLK(3) |
|       ESP32C3(4)     |    4     |          60 000           |         80 MHz        |    GPSPI2     | NO                                                 |
|       ESP32P4(5)     |  8<br>16 |16 000 000-32 000 000<br>8 000 000-16 000 000|80 MHz Ram<br>80 MHz Psram   |    LCD_CAM    | One Free GPIO for PCLK<br> One free GPIO for ETM Trigger(Opt)<br>One LEDC Channel for slow PCLK(3) |

1. I do not recommend using this mode unless necessary. PSRAM heavily loads the SPI bus, artifacts and delays in the operation of the main program are possible. Correct work with Cache requires ESP IDF ver 5.2 (in the current  master branch version)
2. The maximum number of samples depends on the amount of free memory when your program is running, the analyzer will limit the number of samples and return the maximum possible.
3. ESP32S3 -> requires one GPIO pin not connected anywhere for PCLK signal, for sample rate less than 1 MHz - you can allow 1 LEDC channel connection
4. ESP32C3 -> trigger does not use HiLevel interrupts, Delay from trigger to start of data is 1.3-1.5 µs. SUMP operates in 8-channel mode, the upper 4 channels are not used.
5. ESP32P4 -> the maximum number of samples depends on the size of the built-in PSRAM, free RAM memory. The trigger uses regular edge interrupts with a delay before the start of data of 1.3-1.5 µsec or ETM with a delay of 150 nsec. ETM mode requires one free GPIO

 - 1 capture trigger channel. The trigger is organized on interrupts along the fronts. In the ESP32, interrupts are processed for about 2 µS (rev0.1) - Accordingly, the delay from the trigger to the start of the data is about 2 µS. In current versions, the trigger has been moved to Hilevel interrupts ( level 5 ), the delay from the trigger to the beginning of the data has been reduced to 0.3 µS.
 - All mode settings are moved to Menuconfig. Channels, Trigger, Sample rate, use PSRAM is additionally configured in the WEB interface
 - The analyzer allows you to work on the measured device. We install the software on the patient as a standard external component of the ESP IDF, configure the GPIO for channels (checked - GPIO, I2C, I2S, SPI, LED PWM, IRQ_GPIO, UART, USB. I think that the rest will also work), shows both input and output signals of the patient. Trigger restrictions in this mode - you cannot assign a trigger to a pin (GPIO) that has an interrupt assigned to the patient software (the analyzer will reconfigure itself) - in the latest version (interrupt level 5) the restriction is partially removed, but the trigger will fire on those fronts (levels ) that are assigned to the patient software.
 - You can make the analyzer as a separate device, but I don't see much point. There are enough cheap analogs with similar characteristics on the market. The main advantage of self-diagnostics is that we connect the component to the project and see what happens there. It is clear that the patient's software can already use all the memory - then the volume of samples will greatly decrease - but we will still see at least levels and a small number of readings.
 ## Simple web interface
 - Completely wireless connection
 - Easy GPIO configuration per channels
 - Switching the number of channels 8/16 and the use of PSRAM
 - View samples (without protocol analyzer)
 - Saving data in .bin format, which can then be transferred to the Sigrok PulseView if necessary.
 ## Sigrok PulseView can be used as visualization and protocol analyzer
 - Open software
 - Lots of protocol analyzers
 - direct (SUMP) connection uses UART to receive data, transfer protocol "Openbench logic Sniffer & SUMP"
 - the default is UART0 ESP32 you can (or better you need to) use a different port if it is on your device. Can be configured in menuconfig.
 - you can use (recommend) saved data from WEB interface rawBin
 - I recommend to use direct connection to SUMP/Sigrok PulseView only if resources are scarce ( Rom/Ram ). The SUMP protocol is very small in terms of code size.
 ## Known bugs
 - when using SUMP/Sigrok UART0 - it is necessary to disable all ESP32 diagnostic output (LOG LEVEL - NONE ), this is not a bug but a significant limitation.
 - PulseView - to receive data, you need to press RUN 2 times with an interval of 1-2 seconds (I don’t know the reason)
 - PulseView - in trigger mode does not work with frames less than 1k (I don't know why)
 - PulseView - does not accept the maximum sample rate parameter - you can easily see frequencies of 50/100/200 megahertz.
 - PulseView - trigger prefetch does not work (set 0% ) - just didn't do it, and it's impossible in the current architecture.
 - PulseView - unable to set sample rate to 40-80 MHz
  # Connecting LogicAnalyzer to the project as a component
 - see documentation in ESP IDF - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#example-project
 - as an example - https://github.com/ok-home/logic_analyzer/tree/master/logic_analyzer_example
 # Connecting PulseView
 - Connect to device
 - Choose the driver - Openbench logic Sniffer & SUMP
 - Serial Port - Speed - 921600 (speed can be overridden, it works on my cable at this speed)
 - Scan for Device - ESP32 with 8/16 channels should appear
 - Further we read the manual on PulseView.
 - To read .bin files - just import rawBin, 8/16 channels, desired sample rate.
 # Connecting the Web interface ----- go to the page at http://xxx.xxx.xxx.xxx/la
 # Program interface
 ## Conditionally consists of 3 parts
 ### include/logic_analyzer_hal.h
   - Gets samples into ESP32 buffer
   - logic_analyzer_config_t - capture configuration
   - start_logic_analyzer(logic_analyzer_config_t *config) - capture start
   - void (*logic_analyzer_cb_t)(uint8_t *samle_buf, int samples, int sample_rate) - callback after data capture
 ### include/logic_analyzer_sump.h
   - work with PulseView
   - logic_analyzer_sump();
 ### include/logic_analyzer_ws_server.h
   - Start a web server with websocket support - check websocket support in menuconfig
   - logic_analyzer_ws_server()
 ### include/logic_analyzer_ws.h
   - if you already have websocket enabled server installed on your device - just register uri handler
   - logic_analyzer_register_uri_handlers(httpd_handle_t server);
 ## Example with test samples
 ### logic_analyzer_example
 - test_sample_init() - includes a simple 100 kHz generator, a burst of GPIO pulses with a high duty cycle, and a burst of GPIO pulses generated by interrupts. No additional connections, wiring, etc. are needed, it will already show the insides. If you wish, you can put 2 jumpers on the pins - it will duplicate the signals on the GPIO.
 - icing on the cake - we connect with any channel on TXD0 and watch our own uart0 (only web)
 ## Settings moved to menuconfig
 - General settings
   - ANALYZER_USE_HI_LEVEL_INTERRUPT - trigger via high level interrupts
   - ANALYZER_SEPARATE_MODE - to work as a separate device (own GPIO setting)
   - ANALYZER_USE_WS - use WEB interface
   - ANALYZER_USE_SUMP - connect directly to PulseView via COM port
   - ANALYZER_USE_PORT_ASSIGN - set default channels (Channels, trigger, frequency and number of samples, numbe of channels, use PSRAM)
 - Settings for ESP32
   - ANALYZER_USE_I2S_CHANNEL - used I2S channel ( 0/1 )
 - Settings for ESP32S3
   - ANALYZER_PCLK_PIN - GPIO number for PCLK transit
   - ANALYZER_USE_LEDC_TIMER_FOR_PCLK - enable LEDC channel for sample rate less than 1 MHz
 ## To use Hi-level interrupt, you need to set CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL = ESP_SYSTEM_CHECK_INT_LEVEL_4 in menuconfig

## Added CLI interface for stable operation with PulseView when RAM is limited
  - minimal RAM consumption
  - a simple script logic_analyzer_cli/logic_analyzer_cli.py to save samples to a RowBin file (you need to add the PySerial dependency)
  - script parameters are set in the la_cfg.json file (the file template is created automatically the first time the script is launched), GPIO,TRIGGER,TRBGGER EDGE,SAMPLES,SAMPLE RATE,NUMBER CHANNELS,RAM/PSRAM.
  - connection to the program code
    - logic_analyzer_cli/include/logic_analyzer_cli.h
    - set ANALYZER_USE_CLI and UART/USB_SERIAL_JTAG parameters in menuconfig
    - compile and download program code with connected logic_analyzer
    - connect to UART/USB
    - run logic_analyzer_cli.py with the parameters set in the file la_cfg.json
  - connection to PulseView
    - Import Raw binary logic data - a file created by the script
    - setting the number of channels and sample frequency during the first import (esp32c3 - set 8 channels, 4 lower ones are used), in the future, if these parameters do not change, just do Reload
  - for UART0 the same restrictions as for the SUMP interface
  - advantages over SUMP interface
    - look at the known PulseView bugs above - they are not here (except for UART0)
    - simple GPIO configuration for channels via la_cfg.json  

![cli interface output](/la_cli.jpg)

## Added support for ESP32P4
- Only CLI interface is supported
- Data reading in parallel mode 8/16 bit by LCD/CAM module, one GPIO is required for PCLK signal
- For sample rate less than 1 MHz, one LEDC channel is required
- You can select IRQ/ETM trigger mode, ETM mode requires one more additional free GPIO
- Maximum sampling rate does not depend on the number of channels and the type of memory for the buffer (Ram/Psram) and is 80 MHz (for PSRAM frequency = 200 MHz)
- The number of samples is limited only by the size of free memory (Ram/Psram), The maximum number of samples can be 32,000,000 for modules with a Psram volume of 32 Mbyte.

 ## Parts of the code used in the project
  - [esp32-cam](https://github.com/espressif/esp32-camera) for I2S DMA
  - [EUA/ESP32_LogicAnalyzer](https://github.com/EUA/ESP32_LogicAnalyzer) for SUMP

