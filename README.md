# A-B Line Precision Planter Controller

ESP32-based GPS planter row control system for precision agriculture. Defines an A-B baseline, calculates the tractor's perpendicular distance to it, and fires solenoid pulses at configurable row intervals.

## How It Works

1. Drive to the start of your baseline and tap **Mark Point A** on the phone web portal
2. Drive to the end of the baseline and tap **Mark Point B**
3. As you drive subsequent rows, the ESP32 calculates your orthogonal distance from the A-B line
4. Each time the planter crosses a new interval lane, a pulse fires to the relay driving the planter actuator

The math uses a local flat-plane Mercator projection: GPS lat/lon are converted to meters relative to Point A, then a vector cross product yields the perpendicular distance to the line.

## Hardware

- **ESP32** development board
- **RTK GPS module** connected to Serial2 (RX2 = GPIO 16, TX2 = GPIO 17) at 115200 baud
- **Relay / optoisolated MOSFET** on GPIO 23 to drive planter solenoid
- **USB-to-TTL adapter** for bench testing (e.g. FTDI or CP2102)

### Power Protection Circuit

A tractor's 12V rail is electrically noisy. Wire this between the battery and the ESP32's 5V/VIN:

```
 12V BATTERY
     |
   [1A FUSE]
     |
   [1N4007 DIODE]        (reverse polarity protection)
     |
     +--- [470uF CAP]     (absorbs voltage dips)
     |
     +--- [TVS DIODE]     (clamps spikes >15V, e.g. 1.5KE15A)
     |
   [LM2596 / MP1584 BUCK CONVERTER]  ->  Stable 5V
     |
   ESP32 VIN + Relay VCC
```

Bill of Materials:

| # | Component | Purpose |
|---|-----------|---------|
| 1 | 1A inline fuse | Fire protection |
| 2 | 1N4007 diode | Reverse polarity blocking |
| 3 | 470uF electrolytic cap | Cranking voltage dip smoothing |
| 4 | TVS diode (1.5KE15A / 5KP15A) | Transient voltage suppression |
| 5 | DC-DC buck converter (LM2596 / MP1584) | 12V to 5V step-down |
| 6 | Optoisolated relay / MOSFET module | Isolates solenoid 12V from ESP32 logic |

## Project Structure

```
tractor_gps/
  tractor_gps_controller/
    tractor_gps_controller.ino   # Arduino sketch (main firmware)
  tools/
    gps_emulator.py              # Python bench-test GPS emulator & log monitor
  README.md
```

## Arduino Setup

### Dependencies

Install **TinyGPS++** via the Arduino Library Manager (Sketch > Include Library > Manage Libraries).

### Board Configuration

In Arduino IDE, select:

- Board: `ESP32 Dev Module`
- Upload Speed: `115200`

### Flashing

1. Open `tractor_gps_controller/tractor_gps_controller.ino`
2. Select the correct port (look for `/dev/cu.usbmodem*` in Arduino IDE)
3. Click Upload

### Configuration Defaults

| Parameter | Default | Description |
|-----------|---------|-------------|
| Interval | 0.75 m | Distance between row trigger points |
| Offset | 1.25 m | Antenna-to-planter physical offset |
| Pulse | 100 ms | Solenoid energize duration |
| Look-ahead | 0.05 s | Speed-based predictive delay |

All values are configurable via the web portal and saved to flash.

## WiFi Web Portal

On boot, the ESP32 creates a WiFi access point:

| Field | Value |
|-------|-------|
| SSID | `Tractor_Planter_GPS` |
| Password | `agri-precision` |

Connect your phone, open a browser to `192.168.4.1`, and you will see:

- **GPS fix status** and satellite count
- **Line A-B status** (configured or not)
- **Mark Point A / Mark Point B** buttons (require RTK Fixed fix)
- **Reset Field Lines** button (appears once A-B is set)
- Configuration form for interval, offset, pulse duration, and look-ahead

### JSON Status API

The ESP32 exposes a `/status` endpoint that returns all telemetry as JSON:

```bash
curl http://192.168.4.1/status
```

Example response:

```json
{
  "fix_type": "RTK FIXED",
  "fix_quality": 4,
  "satellites": 14,
  "latitude": -33.9998210,
  "longitude": 145.0013420,
  "speed_mps": 2.22,
  "line_ready": true,
  "line_a_lat": -34.0000000,
  "line_a_lon": 145.0000000,
  "line_b_lat": -34.0000000,
  "line_b_lon": 145.0020000,
  "interval_m": 0.75,
  "offset_m": 1.25,
  "pulse_ms": 100,
  "lookahead_s": 0.050,
  "pulse_count": 47,
  "pulse_active": false,
  "last_fix_age_ms": 120,
  "uptime_ms": 342500
}
```

## Bench Testing (Without a Tractor)

Use the Python emulator to simulate an RTK GPS receiver streaming NMEA data to the ESP32.

### Wiring

```
USB-to-TTL Adapter        ESP32
    TX  ---------------->  RX2 (GPIO 16)
    GND ---------------->  GND
```

Power the ESP32 separately via USB (also used for the serial debug monitor).

### Running the Emulator

First, find your serial ports:

```bash
ls /dev/cu.*
```

You will typically see:
- `/dev/cu.usbmodem*` -- ESP32 native USB (debug monitor output)
- `/dev/cu.usbserial*` -- USB-to-serial adapter (GPS input to ESP32 RX2)
- `/dev/cu.SLAB_USBtoUART` -- CP2102 adapter alternative name

```bash
pip install pyserial

# macOS (use your actual port names)
python tools/gps_emulator.py \
  --gps-port /dev/cu.usbserial-A10KH1TF \
  --debug-port /dev/cu.usbmodem-1101

# Linux (replace with your /dev/ttyUSB* ports)
python tools/gps_emulator.py \
  --gps-port /dev/ttyUSB0 \
  --debug-port /dev/ttyACM0

# Windows (replace with your COM* ports)
python tools/gps_emulator.py \
  --gps-port COM4 \
  --debug-port COM5
```

Options:

| Flag | Default | Description |
|------|---------|-------------|
| `--gps-port` | `/dev/cu.usbserial-0001` | Serial port to ESP32 RX2 |
| `--debug-port` | `/dev/cu.usbmodem-1101` | ESP32 debug USB port |
| `--baud` | `115200` | Baud rate (must match firmware) |
| `--lat` | `-34.000000` | Simulated start latitude |
| `--lon` | `145.000000` | Simulated start longitude |
| `--speed` | `8.0` | Simulated speed (km/h) |
| `--heading` | `90.0` | Driving heading (degrees, 90 = East) |

The emulator streams at 10 Hz with `fix quality = 4` (RTK Fixed). You should see pulse confirmations in the terminal:

```
[ESP32 LOG]: Target crossed! Fired Pulse Index: 1
[ESP32 LOG]: Target crossed! Fired Pulse Index: 2
```

Verify with a multimeter or LED on GPIO 23.

## Bench Test Procedure

1. Flash the ESP32 and power it on
2. Connect to the WiFi AP and open the web portal
3. Set an interval of e.g. 5.00 m for easy verification
4. Tap **Mark Point A** (anchors at the emulator's start position)
5. Connect the USB-to-TTL adapter and run the Python script
6. Watch the ESP32 logs for pulse events as the simulated tractor moves East

## Field Operation

1. Pull up to the edge of the field and wait for **RTK FIXED** status on the web panel
2. Tap **Mark Point A**
3. Drive to the end of the first pass and tap **Mark Point B**
4. Turn and drive subsequent rows -- pulses fire automatically at each interval crossing
5. To start a new field, tap **Reset Field Lines** and repeat from step 1

## License

MIT
