# T2CAN Bench Bridge

**A transparent two-port CAN bridge and exact frame logger for a LilyGO T-2CAN (ESP32-S3).**

It sits inline between two classic-CAN segments, passes every frame through
unchanged so both sides keep talking normally, and at the same time captures
every frame — sequence-numbered and timestamped — and streams it to a PC, where
a companion Python tool decodes it against a DBC into readable messages.

It was built to study a **Tesla charge-port ECU ↔ vehicle-harness** link on the
bench, but nothing about the bridge or the logger is model-specific: it works
with **any Tesla model** (Model 3, Y, S, X — the HV/charge buses run at
500 kbit/s classic CAN), and with **any 500 kbit/s classic-CAN pair** at all.
What you decode with is just a DBC file — swap it for the one that matches your
vehicle or subsystem.

---

## Features

- **Transparent bridge** — forwards every valid frame both ways unchanged (ID,
  standard/extended, data/RTR, DLC, payload).
- **Exact capture** — records every received frame with a 32-bit sequence number
  and a microsecond timestamp; gaps are always visible in the sequence.
- **Never blocks forwarding** — capture and USB output run around the bridging;
  USB stalls or an absent host can't stall the bus.
- **Auto-start & self-recovery** — starts ~0.3 s after power-up, no button;
  recovers from bus-off / transmit faults on its own.
- **DBC-driven decoding** — the PC tool turns raw frames into named signals,
  per-ID summaries, a change timeline, and a "reaction" view, for **any** DBC.
- **Model-agnostic** — default DBC is for the Tesla Model 3, but any DBC works.

---

## How it works

```
   Side B device   <──CAN──>  [ Port B: ESP32-S3 TWAI ]
   (e.g. charge port ECU)              T-2CAN ── USB ──> PC (t2can_log.py)
   Side A device   <──CAN──>  [ Port A: MCP2515 (SPI) ]
   (e.g. vehicle harness)
```

- **Port A = MCP2515** (SPI) and **Port B = ESP32-S3 TWAI** — two independent
  classic-CAN interfaces, **500 kbit/s** each.
- The two ports must be **two separate, correctly terminated CAN buses**. Never
  wire both ports to the same bus.
- It is a **store-and-forward** bridge: it adds a little latency, changes bus
  timing slightly, and has **no hardware bypass** — a reset or power loss briefly
  interrupts the link.
- Each received frame is also copied into a PSRAM ring (~170k frames) and
  streamed over USB as a text line. Capture runs in parallel with forwarding.

---

## Repository contents

| File | Description |
|------|-------------|
| `T2CAN_BenchBridge.ino` | Firmware for the LilyGO T-2CAN (ESP32-S3 + MCP2515). Bridges both ports and streams the capture over USB. |
| `t2can_log.py` | PC tool — capture the USB stream, verify completeness, and decode it against a DBC. |
| `Model3CAN.dbc` | Example DBC: Tesla Model 3 (community reverse-engineered, MIT — Josh Wardell). Replace or add your own for other vehicles. |
| `Model3CAN_LICENSE.txt` | License for the bundled DBC. |

---

## Safety

If either side is connected to a high-voltage battery system, treat it as
dangerous. Keep HV interlocks, isolation monitoring and fusing intact, follow
local regulations, and use a licensed electrician for any mains/dwelling tie-in.
This is a passive CAN tap and bridge — it does not make an HV system safe and is
not a substitute for proper battery protection.

---

## Installation

### A) Firmware (LilyGO T-2CAN / ESP32-S3)

**Requirements**
- [Arduino IDE](https://www.arduino.cc/en/software) 2.x.
- ESP32 board support (Espressif **arduino-esp32** core) via Boards Manager.
- Library **autowp-mcp2515 `1.3.1`** (Library Manager → "autowp mcp2515"). The
  code targets that exact version.
- Your board's **`pin_config.h`** in the sketch folder (the sketch `#include`s
  it and compile-time-asserts the pin map below).

**Expected pin map (H784 / T-2CAN)**

| Signal | GPIO | | Signal | GPIO |
|--------|------|-|--------|------|
| CAN_TX (TWAI) | 7 | | MCP2515_CS | 10 |
| CAN_RX (TWAI) | 6 | | MCP2515_RST | 9 |
| MCP2515_SCLK | 12 | | MCP2515_MISO | 13 |
| MCP2515_MOSI | 11 | | MCP2515_INT | 8 |

**Arduino IDE board settings**

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| USB CDC On Boot | Enabled |
| USB Mode | Hardware CDC and JTAG |
| Flash Size | 16MB |
| Flash Mode | QIO 80MHz |
| Partition Scheme | 16M Flash (3MB APP / 9.9MB FATFS) |
| PSRAM | OPI PSRAM *(required — the capture buffer lives in PSRAM)* |

**Flash**
1. Put `T2CAN_BenchBridge.ino` and your `pin_config.h` in one sketch folder.
2. Connect the board over USB, select its port, and **Upload**.
3. It begins bridging and capturing by itself ~0.3 s after power-up.

### B) PC tool

**Requirements**
- Python **3.8+**.
- `pip install pyserial` (needed only for live capture, not for decoding).
- Keep `t2can_log.py` and a DBC (e.g. `Model3CAN.dbc`) in the same folder.

That's it — it's a single script, no build step.

---

## Usage

> Close the Arduino IDE **Serial Monitor** before capturing — only one program
> can hold the COM port at a time.

**List serial ports (find the board)**
```bash
python t2can_log.py ports
```
Marks the port that is the ESP32-S3 (T-2CAN).

**Capture**
```bash
python t2can_log.py capture              # auto-detect the board
python t2can_log.py capture --port COM4  # or name it (e.g. /dev/ttyACM0)
```
Prints the output filename and a live frame count (`A=` / `B=` per-port split,
`missing=` counter). Do what you want to observe, then press **Ctrl+C**. It
decodes automatically on stop.

**Decode a saved log**
```bash
python t2can_log.py decode capture_20261013_1800.log
python t2can_log.py decode mylog.log --dbc MyVehicle.dbc
```

**Flags** (for `capture` and `decode`)

| Flag | Effect |
|------|--------|
| `--dbc FILE.dbc` | Add another DBC. First match per ID wins; the bundled `Model3CAN.dbc` is always loaded — pass your own to decode other vehicles. |
| `--show-analog` | Also list continuous measurements (voltages, currents, temps), hidden by default. |
| `--max-events N` | Max changes listed per ID in the events file (default 200). |

**Output files** (written next to the log)

| File | Contents |
|------|----------|
| `<name>_ids.csv` | One row per CAN ID: port, rate, period, DLC, decoded name + source, distinct payloads, first/last payload. |
| `<name>_events.txt` | Time-ordered changes of slowly-changing signals/bytes (states, requests, flags). |
| `<name>_reactions.txt` | Each Port-B state change with the Port-A changes in the preceding 0.5 s — where "command → reaction" shows up. |
| `<name>_decoded.csv` | Every captured frame, fully decoded. |

**On-board serial commands** (optional; serial at **115200**)

| Input | Effect |
|-------|--------|
| *(idle)* | One `#`-prefixed status line per second. |
| `d` or `?` | Print current status + last fault report; forwarding is not interrupted. |
| `c` | Drop the backlog and start a fresh capture (the PC tool sends this on start). |

**Capture line format**
```
F<seq8hex>,<time8hex>,<A|B>,<id hex>,<S|X>[R],<dlc>,<data hex>
# example:
F0000002A,0371B2C4,B,25D,S,8,0000000000000000
```
`seq` = 32-bit sequence (gaps = unrecorded frames) · `time` = micros(), unwrapped
by the decoder · `A`/`B` = port · `S`/`X` = standard/extended, trailing `R` =
remote · then DLC and payload.

---

## Using it with other vehicles / models

- The **bridge and logger are model-agnostic** — they move and record raw CAN
  frames and don't care what's on the bus.
- To **decode** another Tesla model or another manufacturer, pass that vehicle's
  **DBC** with `--dbc`. Undecoded IDs still appear with raw bytes and
  change-tracking, so the tool is useful even with no DBC.
- The firmware is fixed at **500 kbit/s** (Tesla's HV/charge buses). For a bus at
  a different bitrate, change the `CAN_500KBPS` bitrate constant in the sketch
  (both the MCP2515 `setBitrate` call and the TWAI timing config) and reflash.
- The pin map is specific to the **LilyGO T-2CAN (H784)**; other boards need
  their own `pin_config.h` and may need the pin asserts adjusted.

---

## Troubleshooting

- **"Access is denied" / port busy (Windows):** another program holds the port.
  Close the Serial Monitor / IDE and any serial app, unplug-replug, retry.
- **"cannot find the file specified" / wrong `COMx`:** that port doesn't exist —
  run `python t2can_log.py ports` and use the one marked as the T-2CAN. If it
  isn't listed, suspect a power-only USB cable or an unpowered board.
- **`missing=` climbing:** USB briefly couldn't keep up; frames were still
  forwarded but not recorded, and the sequence numbers mark exactly where. At
  ~2000 frames/s this is normally zero or a tiny handful.
- **Lots of `unknown` IDs:** your DBC doesn't define them — raw bytes and changed
  bytes are still shown. Add a better DBC with `--dbc`.
- **DBC not found:** keep the `.dbc` in the same folder as `t2can_log.py`, or
  pass its path with `--dbc`.

---

## Limitations

- Store-and-forward: latency, altered bus timing, **no** electrical bypass; a
  reset/restart briefly interrupts the link.
- Classic CAN only (no CAN FD).
- DBC names (for the bundled Model 3 file) are community reverse-engineering, not
  manufacturer documentation — treat as strong hints and confirm against
  observed behavior.
- Firmware validated in a software simulation of both controllers and the bus,
  not formally on-vehicle — verify on your hardware.

---

## Credits & references

- Tesla Model 3 DBC — Josh Wardell · https://github.com/joshwardell/model3dbc (MIT)
- MCP2515 Arduino library — autowp · https://github.com/autowp/arduino-mcp2515
- Charge-port controller references ·
  https://github.com/mdrobnak/cpcontrol-redacted ·
  https://github.com/damienmaguire/Tesla-Model-3-Charge-Port-Controller

## License

> Add your chosen license here (e.g. MIT). The bundled `Model3CAN.dbc` is MIT
> (Josh Wardell) — keep `Model3CAN_LICENSE.txt` with it.
