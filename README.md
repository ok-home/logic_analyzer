[Ru](/README-RU.md)

| Supported Targets |  
| ESP32             | 
| ----------------- |

# Logic analyzer on ESP32

![PulseView](/sigrok_esp.jpg)
![WebSocket](/la_ws.jpg)

## Main parameters
  - 16 channels.
  - 40 megahertz. - maximum sample rate
  - 32764 - maximum number of samples per frame (capture buffer). The volume is limited by the maximum size of free DRAM.
  - 1 capture trigger channel. The trigger is organized on interrupts along the fronts. ESP32 interrupts are processed approximately 2 µs - Accordingly, the delay from the trigger to the beginning of the data, about 2 µs.
  - Uses the internal clock of samples, no need to set jumpers to supply sync pulses or use an external generator. Pins for sync pulses are not used.
  - The analyzer allows you to work on the measured device. We install the software on the patient, configure the GPIO for channels (checked - simple GPIO, I2C, LED PWM, I think that the rest will also work), shows both input and output signals of the patient. Trigger restrictions in this mode - you cannot assign a trigger to a pin (GPIO) that has an interrupt assigned to the patient software (the analyzer will reconfigure itself)
  - You can make the analyzer as a separate device, but I don't see much point. Ali has enough cheap analogues with similar characteristics. The main advantage of self-diagnostics is that we linked the software to the project and see what happens there. It is clear that the patient's software can already use the entire DRAM - then the volume of samples will greatly decrease - but we will still see at least the levels and a small number of samples.
  ## Sigrok PulseView is used as visualization
   - Open software
   - Lots of protocol analyzers
   - used UART to receive data, transfer protocol "Openbench logic Sniffer & SUMP"
   - the default is UART0 ESP32 you can (or rather you need to) use a different port if it is on your device.
  ## Added a simple web interface
   - Completely wireless connection
   - Easy GPIO configuration per channels
   - View samples (without interface analysis)
   - Saving data in .bin format, which can then be transferred, if necessary, to the same Sigrok PulseView (16 channels)
   - Connect to wifi - example_connect in menuconfig
   ## Known bugs
   - when using UART0 - it is necessary to disable all ESP32 diagnostic output (LOG LEVEL - NONE ), this is not a bug but a significant limitation.
   - PulseView - to receive data, you need to press RUN 2 times with an interval of 1-2 seconds (I don’t know the reason)
   - PulseView - in trigger mode does not work with frames less than 1k (I don't know why)
   - PulseView - does not accept the maximum sample rate parameter - you can easily see frequencies of 50/100/200 megahertz.
   - PulseView - trigger prefetch does not work (set 0% ) - just didn't do it, and it's impossible in the current architecture.
   # Connecting PulseView
   - Connect to device
   - Choose the driver - Openbench logic Sniffer & SUMP
   - Serial Port - Speed - 921600 (speed can be overridden, it works on my cable at this speed)
   - Scan for Device - ESP32 with 16 channels should appear
   - Further we read the manual on PulseView.
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
   - if a server is already installed on your device - just register uri handler
   - logic_analyzer_register_uri_handlers(httpd_handle_t server);
   ## Example with test samples
   ### logic_analyzer_example
   - test_sample_init() - turns on a simple 500 kHz generator, and a bunch of GPIO pulses with a large duty cycle. No additional connections, wiring, etc. are needed, it will already show the insides. If you wish, you can put 2 jumpers on the pins (18-22,19-23) - it will duplicate the signals on the GPIO.
   - icing on the cake - we connect with any channel to GPIO1 (TXD0) and watch our own uart0 (only web)
   ## Mainly for SUMP/Sigrok - settings moved to menuconfig

## Parts of the code used in the project
  - [esp32-cam](https://github.com/espressif/esp32-camera) for I2S DMA
  - [EUA/ESP32_LogicAnalyzer](https://github.com/EUA/ESP32_LogicAnalyzer) for SUMP

### The project was made for myself, so the wishes of adding and correcting only if desired and possible.