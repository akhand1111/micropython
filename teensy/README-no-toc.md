# Build Instructions for Teensy 3.1, 3.5 and 3.6

These instructions have been tested under Linux (Ubuntu 16.04) and may or may
not work on other platforms.

# Building
To build, you can either use the ARM toolchain from
[here](https://launchpad.net/gcc-arm-embedded)
(add to your PATH), or you can use the ARM toolchain included with
Arduino/Teensyduino (set ARDUINO environment variable to point to the
root of your arduino/teensyduino tree).

```bash
git clone https://github.com/micropython/micropython
cd micropython/teensy
make BOARD=TEENSY_3.1
```
BOARD can be one of TEENSY_3.1, TEENSY_3.5, or TEENSY_3.6. If no BOARD is
specified, then TEENSY_3.1 will be assumed.


# Flashing

The firmware will be in build-TEENSY_3.1/micropython.hex (replace
TEENSY_3.1 appropriately).

You can flash using
[teensy_loader_cli](https://github.com/PaulStoffregen/teensy_loader_cli)
or by using the flashing tools included with teensyduino by setting
ARDUINO to point to the arduino/teensyduino tree.
```bash
make BOARD=TEENSY_3.6 deploy
```

# Running Scripts

Scripts contained within the scripts directory will be compiled into the firmware
image. Place your .py (or pre-compiled .mpy) files in the scripts directory
and then you can import them as per usual.

The scripts directory includes a sample boot.py and main.py. The main.py will
flash the LED twice. For the Teensy 3.5 & 3.6 the scripts can also be located
on the sdcard, which will be located at the path /sd from MicroPython's
perspective. If there is a boot.py or main.py on the sdcard, then the builtin
ones will execute those instead.

On the Teensy 3.1, all scripts need to be compiled into the firmware.

# Accessing the REPL

Currently, the Python prompt is through the USB serial interface, i.e.

```bash
minicom -D /dev/ttyACM0
```

# TIPS

## Install 49-teensy.rules into /etc/udev/rules.d

If you install the [49-teensy.rules](https://www.pjrc.com/teensy/49-teensy.rules)
file into your ```/etc/udev/rules.d``` folder then you won't need to use sudo:
```bash
sudo cp ~/Downloads/49-teensy.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
```
Now /dev/ttyACM0 should be usable by all users.

## Create a GNUmakefile to hold your default settings.

Create a file call GNUmakefile (note the lowercase m) in the teensy folder
and have it set the path to your ARDUINO tree (if desired) and set which BOARD
you use by default. For example:
```make
$(info Executing GNUmakefile)

ARDUINO=${HOME}/arduino-1.6.11
$(info ARDUINO=${ARDUINO})

BOARD = TEENSY_3.6
$(info BOARD = $(BOARD))

include Makefile
```

GNUmakefile is not checked into the source code control system, so it will
retain your settings when updating your source tree. You can also add
additional Makefile customizations this way.

## Tips for OSX

Set the ARDUINO environment variable to the location where Arduino with TeensyDuino is installed.
```bash
export ARDUINO=~/Downloads/Arduino.app/Contents/Java/
```

Search /dev/ for USB port name, which will be cu.usbmodem followed by a few numbers. The name of the port maybe different depending on the version of OSX.
To access the Python prompt type: 

```bash
screen <devicename> 115200
```

## Use rshell to copy scripts onto the sdcard on the teensy

You can use a program called [rshell](https://github.com/dhylands/rshell) to
copy files from your host computer directly onto the teensy sdcard while the
sdcard is still in the teensy. You can also use the repl command to act as
a serial terminal emulator.

rshell should automatically detect the correct serial port to connect to the
teensy.

Make sure that you quit out of any other terminal emulators (like screen or minicom)
before trying to use rshell.

When running rshell, the contents of the sdcard will be found under /sd

# machine module

MicroPython is moving away from using the pyb module and will be using the
machine module in the future. So the teensy code now uses the machine module
as well. The API definitions for the machine interface are still fluid, so
this is my best guess as to what they'll look like. Expect things to change
a bit in the future.

## machine.ADC
adc_id must be 0 or 1.
- `adc = machine.ADC(adc_id, ...)` (see adc.init for ... args)
- `adc.init(bits=10, average=1, use_aref=True)`
- `channel = adc.channel(id=None, pin=None, diff=False)`
Must provide id (an ADC channel number) or a pin. For example, pin A14 maps to
ADC0 channel id 17.
- `val = channel.read()`
```python
from machine import ADC, Pin
adc = ADC(0)
channel = adc.channel(pin=Pin('A14'))
val = channel.read()
```
## machine.Pin
See [pyb.Pin](http://docs.micropython.org/en/latest/pyboard/library/pyb.Pin.html)
for documentation. Refer to the [teensy pinout](https://www.pjrc.com/teensy/pinout.html)
diagrams. Plain pin numbers are preceded by a 'D' (so pin 1 is called D1) and
the analog pins use the A prefix. The onboard LED uses a pin name of 'LED'. Use
the python command `dir(machine.Pin.board)` to see the names of the board pins
and use `dir(machine.Pin.cpu)` to see the names of the cpu pins.

## machine.RTC
- `rtc = machine.RTC()`
- `tuple = rtc.datetime()` - retrieves the current RTC date and time. The
returned tuple will have 8 elements: (year, month, day, week-day, hour, minutes, seconds, 0)
- `rtc.datetime((year, month, day, week-day, hour, minute, seconds, sub-seconds))` will
set the RTC. The sub-seconds field is ignored. year is expected to be the current year
(i.e. 2017). For month, 1=January, for week-day 0=Mon .. 6=Sun.
- `rtc.alarm(alarm_id, milliseconds)` Sets the alarm to expire the indicated
number of milliseconds in the future. Note that the RTC hardware only supports
seconds, so this function converts milliseconds to seconds by dividing by 1000.
alarm_id must be RTC.ALARM0.
- `time_left = rtc.alarm_left(alarm_id)` - Returns the number of milliseconds
before the alarm will expire.
- `rtc.alarm_cancel(alarm_id)`
- `rtc.irq(trigger=RTC.ALARM0, handler=alarm_callback_function)` - Sets the
callback function which will be called when the alarm expires.
- `rtc.calibration(tcr=None, cir=None, tcv=None, cic=None)` Sets the TCR, CIR,
TCV, and CIR RTC calibration registers (see the datasheet for details)
- `rtc.ALARM0` - alarm_id which can be used with these functions.
```python
from machine import RTC
rtc = RTC()
now = rtc.datetime()
print('Current date is {}-{:02}-{:02} at {:02}:{:02}:{:02}'.format(now[0], now[1], now[2], now[4], now[5], now[6]))
print('Setting RTC to Jan 15, 2017 at 12 noon')
rtc.datetime((2017, 1, 15, 6, 12, 0, 0, 0))

def my_rtc_handler(rtc):
    print('alarm expired')

rtc.irq(RTC.ALARM0, handler=my_rtc_handler)
rtc.alarm(2000) # Set an alarm for 2 seconds in the future
```
## machine.SD
The SD card is checked at bootup. If you plug the SD card in after bootup then
you should reset the board to get it to see the card.
- `sd = machine.SD()`
- `is_present = sd.present()` - determines if an sdcard is currently in the sdcard connector.
- `(card_bytes, block_size, card_type) = sd.info()`
- `powered = sd.power(state)` - turns the sdcard power on or off (state is a boolean)

## machine.Timer
- `timer = machine.Timer(id, ...)` (see timer.init for ... args)
- `timer.init(*, freq, prescaler, period, mode, callback)` - Initializes the
timer. Either freq or prescaler and
period must be specified. mode should be Timer.UP (counts from 0 to period-1) or
Timer.CENTER (counts from 0 to period-1 and back down to 0 again). callback is
as per Timer.callback.
- `timer.deinit()` - Stops the timer, which disables any timer and timer channel callbacks.
- `channel = timer.channel(mode, *, callback, pin, pulse_width, pulse_width_percent, compare, polarity)` -
Initializes a timer channel. mode can be one of Timer.PWM, Timer.PWM_INVERTED,
Timer.OC_TIMING, Timer.OC_ACTIVE, Timer.OC_INACTIVE, Timer.OC_TOGGLE, or Timer.IC.
callback should be as per channel.callback(). pin can be a pin object which will
be initialized to timer mode (rather than GPIO mode). pulse_width or pulse_width_percent
can be specified with the PWM modes. compare and polarity (Timer.HIGH or Timer.LOW)
can be specified with the OC modes. polarity (Timer.RISING, Timer.FALLING, Timer.BOTH)
can be specified with the IC mode.
- `timer.counter(val)` - sets the timer counter.
- `val = timer.counter()` - returns the timer counter.
- `freq = timer.source_freq()` - For the teensy, this always returns the value of F_BUS.
- `timer.prescaler(val)` - sets the timer prescaler value.
- `val = timer.prescaler()` - returns the timer prescaler value.
- `timer.period(val)` - sets the timer prescaler value.
- `val = timer.period()` - returns the timer prescaler value.
- `timer.callback(callback)` - sets the callback function which will be called
when the timer is triggered.
- `channel.callback()` - sets the callback function which will be called when
channel is triggered (i.e. an input is captured, an OC compare event occurs)
- `channel.pulse_width(width)` - Sets the pulse width when in PWM mode.
- `val = channel.pulse_width()` - Returns the pulse width when in PWM mode.
- `channel.pulse_width_percent(pct)` - Sets the pulse_width as a percentage of the
period when in PWM mode.
- `pct = channel.pulse_width_percent()` - Returns the pulse_width as a percentage of the
period when in PWM mode.
- `val = channel.capture()` - Returns the capture value associated with the channel when in IC mode.
- `channel.compare(val)` - Sets the compare mode to use when in OC mode.
- `val = channel.compare()` - Returns the compare mode to use when in OC mode.

On the Teensy 3.6, source_freq will be 60 MHz. Using a prescaler of 128 gives
a 468,750 Hz tick. With a period of 37499, the PWM period will be 12.5 Hz.
```python
timer = pyb.Timer(0, prescaler=128, period=37499, counter_mode=pyb.Timer.COUNTER_MODE_CENTER)
ch0 = t0.channel(0, pyb.Timer.PWM, pin=pyb.Pin.board.D22, pulse_width=(t0.period() + 1) // 4)
ch1 = t0.channel(1, pyb.Timer.PWM, pin=pyb.Pin.board.D23, pulse_width=(t0.period() + 1) // 2)
```

## machine.UART
- `uart = machine.UART(uart_id, ...)` - (see uart.init for ... args)
- `uart.init(baudrate=9600, bits=8, parity=None, stop=1, flow=None, timeout=1000, timeout_char=0, read_buf_len=64, pins)`
flow can be None, UART.RTS, UART.CTS, UART.RTS | UART.CTS. timeout specifies the number
of milliseconds to wait for the entire read or write to complete. timeout_char specifies
the number of milliseconds to wait between characters for a read/write. If pins=None then
no pin initialization is performed. Otherwise, pins should a tuple containing 2 or 4 entries.
If it only has 2 entries then it specifies (tx_pin, rx_pin), and if it has 4 entries
then it specifies (tx_pin, rx_pin, rts_pin, cts_pin). If no pins argument is provided
then the default pins (see the teensy pinout) will be used.
- `uart.deinit()` - disables the uart.
- `num_chars = uart.any()` - Returns the number of characters available for reading.
- `buf = uart.read([nbytes])` - Reads the specified number of bytes (or as many as possible if nbytes is not provided) before a timeout occurs.
- `line = uart.readline()` - Reads until a newline is encountered or a timeout occurs. Returns None if a timeout occurs.
- `num_bytes_read = uart.readinto(buf, [nbytes])` - Reads into an already allocated buffer. Will read nbytes if specified, or len(buf) if nbytes is not specified. Returns None if a timeout occurs.
- `num_bytes_written = uart.write(buf)` - Writes buf to the UART. Returns the number of bytes written or None if a timeout occurs.
- `uart.writchar(char)` - Writes a single character to the UART.
- `ch = uart.readchar()` - Reads a single character from the UART. Returns -1 if a timeout occurs.
- `uart.sendbreak()` - Sends a break consition on the uart.
- `uart.RTS` - Constant to use for enabling RTS flow control.
- `uart.CTS` - Constant to use for enabling CTS flow control.

## machine.USB_VCP
The USB_VCP class allows creation of an object representing the USB virtual
comm port. It can be used to read and write data over USB to the connected host.

- `vcp = machine.USB_VCP()` - Create a new USB_VCP object.
- `vcp.setinterrupt(char)` - Set the character which interrupts Running python code.
This is set to 3 (Control-C) by default. Set to -1 to disable the interrupt feature 
(useful for sending raw bytes over the usb-serial link).
- `vcp.isconnected()` - Return True if USB is connected as a serial device, False otherwise.
- `vcp.close()` - This method does nothing and exists so that the USB_VCP object can be used as a file.
- `num_chars = vcp.any()` - Returns the number of characters available for reading.
- `buf = vcp.send(data, *, timeout=5000)` - Sends data over the USB VCP. data can either
be an integer (byte value) or a buffer object.
- `buf = vcp.recv(data, *, timeout=5000)` - Receives data from the USB VCP. data can either
be an integer specifying the number of bytes to read, in which case, a buffer containing the
received bytes is returned, or data can be a buffer, in which case the number of
bytes received is returned.
- `buf = vcp.read([nbytes])` - Reads the specified number of bytes (or as many as possible if nbytes is not provided) before a timeout occurs.
- `line = vcp.readline()` - Reads until a newline is encountered or a timeout occurs. Returns None if a timeout occurs.
- `num_bytes_read = vcp.readinto(buf, [nbytes])` - Reads into an already allocated buffer. Will read nbytes if specified, or len(buf) if nbytes is not specified. Returns None if a timeout occurs.
- `num_bytes_written = vcp.write(buf)` - Writes buf to the UART. Returns the number of bytes written or None if a timeout occurs.

## machine misc functions
- `machine.info([dump_alloc_table])` - prints a bunch of information about the board.
- `machine.unique_id()` - Returns a string of 12 bytes (96 bits) which is the unique id for the MCU.
- `machine.reset()` - resets the teensy in a manner similar to asserting the Reset signal.
- `machine.bootloader()` - Activates the bootloader. 
- `machine.freq()` - Returns a tuple containing (F_CPU, F_BUS, F_MEM).
- `machine.rng()` - Return a 30-bit hardware generated random number. This is only available on Teensy 3.5 & 3.6.
- `machine.idle()` - Executes a WFI (wait-for-interupt) instruction.
- `machine.reset_cause()` - Returns one of machine.SOFT_RESET, machine.POWER_ON, machine.HARD_RESET, or machine.WDT_RESET.
- `state = machine.disable_irq()` - Disables interrupts and returns the previous irq state.
- `machine.enable_irq(state=True)` - Restores interrupts to the provided state.
- `machine.mem8[address] = val` - Writes an 8-bit value into memory at the indicated address.
- `val = machine.mem8[address]` - Reads an 8-bit value from the indicated memory address.
- `machine.mem16[address] = val` - Writes an 16-bit value into memory at the indicated address.
- `val = machine.mem16[address]` - Reads an 16-bit value from the indicated memory address.
- `machine.mem32[address] = val` - Writes an 32-bit value into memory at the indicated address.
- `val = machine.mem32[address]` - Reads an 32-bit value from the indicated memory address.

# Other modules
See [this documentation](http://docs.micropython.org/en/latest/pyboard/library/index.html#python-standard-libraries-and-micro-libraries)
for other modules included with MicroPython. The teensy port doesn't currently
implement the usocket or network modules.
