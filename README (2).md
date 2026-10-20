# Automatic Antenna Tuner

ESP32 firmware for an automatic antenna tuner: it measures SWR from a
forward/reflected RF detector and drives a stepper-motor-actuated variable
capacitor to find the lowest-SWR position, with a live web dashboard and a
USB serial command interface.

Team project: the RF hardware (detector, variable capacitor, stepper
motor) was built by a radio amateur; this repo covers the ESP32 firmware
and web interface, developed with AI assistance.

## What it does

- Reads forward and reflected RF levels through two ADC inputs and
  computes SWR.
- Drives a stepper motor (via a Pololu Tic T825 controller over I2C) to
  move a variable capacitor and search for the lowest-SWR position.
- Auto-tune runs a two-stage search — a coarse sweep across the full
  range, then a fine sweep around the best point found — and stops early
  once SWR drops below a configurable threshold.
- Serves a live web dashboard (SWR, forward power, auto-tune progress,
  manual jog controls) over WiFi, plus the same status/control over USB
  serial.

## Hardware

- ESP32 (LOLIN D32)
- RF forward/reflected detector feeding two ADC inputs
- Stepper motor + Pololu Tic T825 controller (I2C) driving a variable
  capacitor
- WiFi for the dashboard; USB serial as a fallback control path

## Design notes

- Runs as a FreeRTOS task so the tuning sweep doesn't block the web
  server; an I2C mutex protects the shared bus between the main loop and
  the tuning task.
- Software safety limits (`SANITY_RANGE`) stop the sweep from driving the
  capacitor past a safe range, even if the search parameters are
  misconfigured.
- Each measurement takes the median of three ADC reads to reject RF
  spikes.

## Setup

1. Open `AntennaTuner.ino` in the Arduino IDE.
2. Install the libraries listed at the top of the file (WiFiManager,
   AsyncTCP, ESPAsyncWebServer).
3. Adjust the constants in the configuration sections (pins, motor
   speeds, sweep range) to match your hardware.
4. Upload. On first boot it opens a `AntennaTuner-Setup` WiFi access
   point to configure your network; after that it's reachable at
   `http://esp32.local` or the IP printed on serial.

## Known limitations

- No authentication on the dashboard or control endpoints — intended for
  a trusted home network.
- SWR calibration (`PWR_CAL`) needs to be set against a known load for
  the power reading to be accurate.
