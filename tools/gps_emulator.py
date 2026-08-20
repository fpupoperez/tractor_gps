#!/usr/bin/env python3
"""
ESP32 Precision Planter GPS Emulator & Log Monitor

Bench-test tool that simultaneously:
1. Emulates an RTK GPS receiver by streaming GPGGA + GPVTG NMEA sentences
   down a serial port to the ESP32's GPS input (GPIO 16).
2. Monitors the ESP32's debug serial output for pulse confirmations and logs.

Hardware wiring:
  USB-to-TTL adapter TX  -> ESP32 RX2 (GPIO 16)
  USB-to-TTL adapter GND -> ESP32 GND

Requirements:
  pip install pyserial
"""

import time
import serial
import threading
import math
import argparse

# --- Default Configuration ---
# macOS serial ports use /dev/cu.* (not /dev/tty.*)
# Find your ports: ls /dev/cu.*
#   ESP32 native USB (debug monitor) -> /dev/cu.usbmodem*
#   USB-to-serial adapter (GPS input) -> /dev/cu.usbserial* or /dev/cu.SLAB_USBtoUART
DEFAULT_GPS_PORT = "/dev/cu.usbserial-0001"   # USB-to-TTL adapter -> ESP32 RX2 (GPIO 16)
DEFAULT_DEBUG_PORT = "/dev/cu.usbmodem-1101"  # ESP32 native USB (Serial monitor)
DEFAULT_BAUD = 115200
SHOW_LOGS = False


def read_esp32_logs(port, baud):
    """Listen to the ESP32 debug serial output and display log lines."""
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print(f"[LOG MONITOR] Connected to ESP32 debug on {port}. Monitoring...\n")
            while True:
                if ser.in_waiting:
                    line = ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        print(f"[ESP32 LOG]: {line}")
                time.sleep(0.01)
    except Exception as e:
        print(f"[LOG MONITOR ERROR] Could not monitor {port}: {e}")


def nmea_checksum(sentence):
    """Calculate the NMEA 0183 XOR checksum and return it as a hex string."""
    cksum = 0
    for char in sentence:
        cksum ^= ord(char)
    return f"{cksum:02X}"


def format_lat_nmea(decimal_deg):
    """Convert decimal latitude to NMEA DDMM.MMMMMM format."""
    direction = "S" if decimal_deg < 0 else "N"
    abs_deg = abs(decimal_deg)
    deg = int(abs_deg)
    minutes = (abs_deg - deg) * 60.0
    return f"{deg:02d}{minutes:08.5f},{direction}"


def format_lon_nmea(decimal_deg):
    """Convert decimal longitude to NMEA DDDMM.MMMMMM format."""
    direction = "W" if decimal_deg < 0 else "E"
    abs_deg = abs(decimal_deg)
    deg = int(abs_deg)
    minutes = (abs_deg - deg) * 60.0
    return f"{deg:03d}{minutes:08.5f},{direction}"


def generate_nmea_stream(port, baud, start_lat, start_lon, speed_kmh, heading):
    """
    Stream simulated RTK NMEA sentences to the ESP32.

    Sends $GPGGA (fix + coords), $GPRMC (date/validity + coords), and
    $GPGSA (fix type) so TinyGPS++ reports RTK Fixed status.
    Simulates the tractor driving in a straight line at constant speed.
    """
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print(f"[EMULATOR] Connected to ESP32 GPS input on {port}")
            print(f"[EMULATOR] Start: ({start_lat:.6f}, {start_lon:.6f})")
            print(f"[EMULATOR] Speed: {speed_kmh:.1f} km/h  Heading: {heading:.1f} deg")

            speed_mps = speed_kmh / 3.6
            heading_rad = math.radians(heading)

            # Direction components (north/east meters per second)
            north_mps = speed_mps * math.cos(heading_rad)
            east_mps = speed_mps * math.sin(heading_rad)

            lat_to_deg = 1.0 / 111132.95
            lon_to_deg = 1.0 / (math.cos(math.radians(start_lat)) * 111319.9)

            update_rate_hz = 10
            interval = 1.0 / update_rate_hz
            elapsed = 0.0
            start_time = time.time()

            print(f"[EMULATOR] Streaming {update_rate_hz} Hz NMEA sentences. Ctrl+C to stop.\n")

            while True:
                current_lat = start_lat + (north_mps * elapsed) * lat_to_deg
                current_lon = start_lon + (east_mps * elapsed) * lon_to_deg

                nmea_lat = format_lat_nmea(current_lat)
                nmea_lon = format_lon_nmea(current_lon)

                # UTC time from system clock
                utc = time.gmtime(start_time + elapsed)
                utc_str = time.strftime("%H%M%S.00", utc)

                # Date as DDMMYY
                date_str = time.strftime("%d%m%y", utc)

                # 1. $GPGGA - Fix quality 4 = RTK Fixed, 12 sats, HDOP 0.8
                gga_raw = (
                    f"GPGGA,{utc_str},{nmea_lat},{nmea_lon},4,12,0.8,100.0,M,0.0,M,1.0,0000"
                )
                gga_sentence = f"${gga_raw}*{nmea_checksum(gga_raw)}\r\n"

                # 2. $GPRMC - Recommended Minimum, A=valid, speed in knots + course
                knots = speed_kmh / 1.852
                rmc_raw = (
                    f"GPRMC,{utc_str},A,{nmea_lat},{nmea_lon},"
                    f"{knots:.1f},{heading:.1f},{date_str},,,A"
                )
                rmc_sentence = f"${rmc_raw}*{nmea_checksum(rmc_raw)}\r\n"

                # 3. $GPGSA - Active set, fix type 4=DGPS/RTK, 12 sats, HDOP 0.8
                gsa_raw = "GPGSA,A,4,1,2,3,4,5,6,7,8,9,10,11,12,0.8,0.4,0.6"
                gsa_sentence = f"${gsa_raw}*{nmea_checksum(gsa_raw)}\r\n"

                # 4. $GPVTG - Ground speed
                vtg_raw = (
                    f"GPVTG,{heading:.1f},T,,M,"
                    f"{knots:.1f},N,{speed_kmh:.1f},K,A"
                )
                vtg_sentence = f"${vtg_raw}*{nmea_checksum(vtg_raw)}\r\n"

                ser.write(gga_sentence.encode("ascii"))
                ser.write(rmc_sentence.encode("ascii"))
                ser.write(gsa_sentence.encode("ascii"))
                ser.write(vtg_sentence.encode("ascii"))

                elapsed += interval
                time.sleep(interval)

    except Exception as e:
        print(f"[EMULATOR ERROR] Could not send data on {port}: {e}")


def main():
    global SHOW_LOGS
    parser = argparse.ArgumentParser(
        description="ESP32 GPS emulator and log monitor for bench testing"
    )
    parser.add_argument(
        "--gps-port",
        default=DEFAULT_GPS_PORT,
        help="Serial port connected to ESP32 RX2 (GPIO 16)",
    )
    parser.add_argument(
        "--show-logs",
        default=SHOW_LOGS,
        help="Show ESP32 debug logs",
    )
    parser.add_argument(
        "--debug-port",
        default=DEFAULT_DEBUG_PORT,
        help="Serial port for ESP32 native USB debug output",
    )
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument(
        "--lat", type=float, default=-34.000000, help="Starting latitude"
    )
    parser.add_argument(
        "--lon", type=float, default=145.000000, help="Starting longitude"
    )
    parser.add_argument(
        "--speed", type=float, default=8.0, help="Simulated speed in km/h"
    )
    parser.add_argument(
        "--heading", type=float, default=90.0, help="Driving heading in degrees"
    )
    args = parser.parse_args()

    print("=" * 52)
    print("    ESP32 Precision Planter Hardware Tester")
    print("=" * 52)
    print()
    print(f"  GPS port : {args.gps_port}")
    print(f"  Show logs : {args.show_logs}")
    print(f"  Debug port: {args.debug_port}")
    print(f"  Baud rate : {args.baud}")
    print(f"  Start pos: ({args.lat:.6f}, {args.lon:.6f})")
    print(f"  Speed     : {args.speed} km/h")
    print(f"  Heading   : {args.heading} deg")
    print()

    SHOW_LOGS = args.show_logs

    if SHOW_LOGS:
        # Start log monitor in a background thread
        log_thread = threading.Thread(
            target=read_esp32_logs,
            args=(args.debug_port, args.baud),
            daemon=True,
        )
        log_thread.start()
    time.sleep(0.5)

    # Run the emulator (blocks until interrupted)
    try:
        generate_nmea_stream(
            args.gps_port,
            args.baud,
            args.lat,
            args.lon,
            args.speed,
            args.heading,
        )
    except KeyboardInterrupt:
        print("\n[EMULATOR] Stopped by user.")


if __name__ == "__main__":
    main()
