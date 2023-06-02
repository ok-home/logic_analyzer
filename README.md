[Ru](/README-RU.md)

| Supported Targets |  
| ESP32             | 
| ----------------- |

# Logic analyzer on ESP32 for self-diagnostics

![WebSocket](/la_ws.jpg)
![PulseView](/sigrok_esp.jpg)

## Main parameters
  - 16 channels.
  - 40 megahertz. - maximum sample rate
  - 50000 - maximum number of samples per frame (capture buffer). The volume is limited by the maximum size of free DRAM.
  - 1 capture trigger channel. The trigger is organized on interrupts along the fronts. ESP32 interrupts are processed approximately 2 µs - Accordingly, the delay from the trigger to the beginning of the data, about 2 µs (rev0.1). In the latest version, the trigger has been moved to Hilevel interrupts ( level 5 ), the delay from the trigger to the beginning of the data has been reduced to 0.3 μS (rev1.1).
  - Uses the internal clock of samples, no need to set jumpers to supply sync pulses or use an external generator. Pins for sync pulses are not used.
  - The analyzer allows you to work on the measured device. We install the software on the patient, configure the GPIO for channels (checked - GPIO, I2C, LED PWM, IRQ_GPIO, I think that the rest will also work), shows both input and output signals of the patient. Trigger restrictions in this mode - you cannot assign a trigger to a pin (GPIO) that has an interrupt assigned to the patient software (the analyzer will reconfigure itself) - in the latest version (interrupt level 5) the restriction is partially removed, but the trigger will fire on those fronts (levels ) that are assigned to the patient software.
  - You can make the analyzer as a separate device, but I don't see much point. There are a sufficient number of cheap analogs with similar characteristics on the market. The main advantage of self-diagnostics is that we linked the software to the project and see what happens there. It is clear that the patient's software can already use the entire DRAM - then the volume of samples will greatly decrease - but we will still see at least the levels and a small number of samples.
   ## Simple web interface
   - Completely wireless connection
   - Easy GPIO configuration per channels
   - View samples (without protocol analyzer)
   - Saving data in .bin format, which can then be transferred, if necessary, to the same Sigrok PulseView (16 channels)
   - Connect to wifi - example_connect in menuconfig
  ## Sigrok PulseView can be used as visualization and protocol analyzer
   - Open software
   - Lots of protocol analyzers
   - direct (SUMP) connection uses UART to receive data, transfer protocol "Openbench logic Sniffer & SUMP"
   - the default is UART0 ESP32 you can (or better you need to) use a different port if it is on your device. Can be configured in menuconfig.
   - you can use (recommend) the saved data from the rawBin web interface
   - I recommend to use direct connection to SUMP/Sigrok PulseView only if resources are scarce ( Rom/Ram ). The SUMP protocol is very small in terms of code size.
   ## Known bugs
   - when using SUMP/Sigrok UART0 - it is necessary to disable all ESP32 diagnostic output (LOG LEVEL - NONE ), this is not a bug but a significant limitation.
   - PulseView - to receive data, you need to press RUN 2 times with an interval of 1-2 seconds (I don’t know the reason)
   - PulseView - in trigger mode does not work with frames less than 1k (I don't know why)
   - PulseView - does not accept the maximum sample rate parameter - you can easily see frequencies of 50/100/200 megahertz.
   - PulseView - trigger prefetch does not work (set 0% ) - just didn't do it, and it's impossible in the current architecture.
   - PulseView - unable to set sample rate to 40 mHz
   # Connecting PulseView
   - Connect to device
   - Choose the driver - Openbench logic Sniffer & SUMP
   - Serial Port - Speed - 921600 (speed can be overridden, it works on my cable at this speed)
   - Scan for Device - ESP32 with 16 channels should appear
   - Further we read the manual on PulseView.
   - To read .bin files - just import rawBin, 16 channels, desired sample rate.
   # Connecting the Web interface ----- go to the page at http://xxxxxxx/la
   # Program interface
   ## Conditionally consists of 3 parts
   ### include/logic_analyzer_hal.h
   - Gets samples into ESP32 buffer
   - logic_analyzer_config_t - capture configuration
   - start_logic_analyzer(logic_analyzer_config_t *config) - capture start
   - void (*logic_analyzer_cb_t)(uint16_t *samle_buf, int samples, int sample_rate) - callback after data capture
   ### include/logic_analyzer_sump.h
   - work with PulseView
   - logic_analyzer_sump();
   ### include/logic_analyzer_ws_server.h
   - Start a web server with websocket support
   - logic_analyzer_ws_server()
   ### include/logic_analyzer_ws.h
   - if you already have websocket enabled server installed on your device - just register uri handler
   - logic_analyzer_register_uri_handlers(httpd_handle_t server);
   ## Example with test samples
   ### logic_analyzer_example
   - test_sample_init() - includes a simple 100 kHz generator, a burst of GPIO pulses with a high duty cycle, and a burst of GPIO pulses generated by interrupts. No additional connections, wiring, etc. are needed, it will already show the insides. If you wish, you can put 2 jumpers on the pins (18-22,19-23) - it will duplicate the signals on the GPIO.
   - icing on the cake - we connect with any channel to GPIO1 (TXD0) and watch our own uart0 (only web)
   ## Mainly for SUMP/Sigrok - settings moved to menuconfig
   ## To use Hi-level interrupt, you need to set CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL = ESP_SYSTEM_CHECK_INT_LEVEL_4 in menuconfig

## Parts of the code used in the project
  - [esp32-cam](https://github.com/espressif/esp32-camera) for I2S DMA
  - [EUA/ESP32_LogicAnalyzer](https://github.com/EUA/ESP32_LogicAnalyzer) for SUMP

### The project was made for myself, so the wishes of adding and correcting only if desired and possible.
