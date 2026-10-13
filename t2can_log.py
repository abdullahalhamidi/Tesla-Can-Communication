#!/usr/bin/env python3
"""
t2can_log.py - save and decode the exact CAN capture streamed by
T2CAN_BenchBridge AUTO v6 (LilyGO T-2CAN between a Tesla Model 3 charge
port ECU and the X096 vehicle connector).

  1) Capture (needs:  pip install pyserial)
       python t2can_log.py ports                          (shows which COM port is the board)
       python t2can_log.py capture                        (finds the board by itself)
       python t2can_log.py capture --port COM5            (Windows)
       python t2can_log.py capture --port /dev/ttyACM0    (Linux / macOS: /dev/cu.usbmodem...)
     Ctrl+C stops. The raw stream is saved exactly as received
     (capture_YYYYmmdd_HHMMSS.log) and then decoded automatically.
     Close the Arduino Serial Monitor first - only one program can own the port.

  2) Decode a saved capture (no extra packages needed)
       python t2can_log.py decode capture_20261012_101500.log
       python t2can_log.py decode capture.log --dbc tesla_model3.dbc   (add more DBC files)
     Also accepts the older logs that only contain fault-report traces.

Outputs (next to the log file):
  <name>_ids.csv      one row per CAN ID: port, rate, DLC, message name, where the name
                      comes from, how many distinct payloads, first/last payload
  <name>_events.txt   time line of every change of a slowly-changing signal or byte
                      (door open, cable present, latch, requests ...). Counters,
                      checksums and measurements that change all the time are listed
                      once in the summary instead of flooding the time line.
  <name>_reactions.txt every charge-port state change, with the vehicle-side changes
                      in the 0.5 s before it (where a command such as "open door" shows up)
  <name>_decoded.csv  every captured frame with its decoded signals

Message names / signals come from Model3CAN.dbc (Josh Wardell, MIT licence,
https://github.com/joshwardell/model3dbc), which must sit next to this script.
It was reverse engineered by the community on older Tesla firmware: treat
names as strong hints, not as Tesla documentation, and check the decoded
values against what you physically do (open the door, plug in, ...).
"""
import argparse
import collections
import csv
import datetime
import os
import re
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_DBC = os.path.join(HERE, "Model3CAN.dbc")
PORT_NAMES = {"A": "vehicle side (X096)", "B": "charge port ECU"}

# Frames a Model 3 charge port ECU transmits when it is powered on its own
# (from the public logs in github.com/damienmaguire/Tesla-Model-3-Charge-Port-Controller,
# "CAN Logs/US_cpecu_onown1.csv" and "EU_CCS_cpecu_onown1.csv"), plus 0x13D/0x43D/0x49D.
CP_ECU_IDS = {0x13D, 0x21D, 0x23D, 0x25D, 0x27D, 0x29D, 0x2BD, 0x31E, 0x33D, 0x37D,
              0x41D, 0x43D, 0x49D, 0x50E, 0x53E, 0x5FE, 0x73D, 0x75D, 0x77D, 0x7DD}

# Names (no signal definitions) used by other community DBC files for IDs that
# Model3CAN.dbc does not define. Shown as hints only.
NAME_HINTS = {
    0x1F9: "VCSEC_requests", 0x272: "VCLEFT_epbmDebug", 0x2B2: "BMS_chargerRequest",
    0x2D1: "VCFRONT_okToUseHighPower", 0x2E8: "EPBR_status", 0x31E: "CP_alertMatrix",
    0x53E: "CP_info", 0x76A: "HVP_vehicleDbg", 0x7AA: "HVP_debugMessage",
}


# ----------------------------------------------------------------------------
# Minimal DBC reader (messages, signals incl. multiplexing, value tables)
# ----------------------------------------------------------------------------
class Signal:
    def __init__(self, name, mux, start, length, little, signed, factor, offset, unit):
        self.name, self.mux, self.start, self.length = name, mux, start, length
        self.little, self.signed, self.factor, self.offset, self.unit = little, signed, factor, offset, unit
        self.is_mux_selector = mux == "M"
        self.mux_value = int(mux[1:]) if mux and mux.startswith("m") and mux[1:].isdigit() else None
        self.values = {}

    def raw(self, data):
        nbits = len(data) * 8
        if self.little:
            if self.start + self.length > nbits:
                return None
            v = (int.from_bytes(bytes(data), "little") >> self.start) & ((1 << self.length) - 1)
        else:  # Motorola: start bit is the MSB, sawtooth numbering
            v, pos = 0, self.start
            for _ in range(self.length):
                if pos < 0 or pos >= nbits:
                    return None
                v = (v << 1) | ((data[pos // 8] >> (pos % 8)) & 1)
                pos = pos + 15 if pos % 8 == 0 else pos - 1
        if self.signed and v & (1 << (self.length - 1)):
            v -= 1 << self.length
        return v

    def physical(self, raw):
        if raw in self.values:
            return self.values[raw]
        val = raw * self.factor + self.offset
        if isinstance(val, float) and val.is_integer() and float(self.factor).is_integer():
            val = int(val)
        elif isinstance(val, float):
            val = round(val, 6)
        return f"{val}{(' ' + self.unit) if self.unit else ''}"


class Message:
    def __init__(self, can_id, name, length, source):
        self.id, self.name, self.length, self.source = can_id, name, length, source
        self.signals = []

    def selector(self, data):
        sel = next((s for s in self.signals if s.is_mux_selector), None)
        return sel.raw(data) if sel else None

    def decode(self, data):
        """Returns list of (signal name, raw, text) for the signals present."""
        out = []
        sel = next((s for s in self.signals if s.is_mux_selector), None)
        selv = sel.raw(data) if sel else None
        for s in self.signals:
            if s.mux_value is not None and s.mux_value != selv:
                continue
            r = s.raw(data)
            if r is None:
                continue
            out.append((s.name, r, s.physical(r)))
        return out


def load_dbc(path, db):
    txt = open(path, encoding="utf-8", errors="replace").read()
    src = os.path.basename(path)
    cur = None
    re_bo = re.compile(r"^BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)")
    re_sg = re.compile(r"^\s*SG_\s+(\w+)\s*(M|m\d+M?|m\d+)?\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*"
                       r"\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)\s*\[[^\]]*\]\s*\"([^\"]*)\"")
    added = 0
    for line in txt.splitlines():
        m = re_bo.match(line)
        if m:
            cid = int(m.group(1)) & 0x1FFFFFFF
            ext = bool(int(m.group(1)) & 0x80000000)
            if (cid, ext) in db:      # first DBC loaded wins
                cur = None
                continue
            name = re.sub(r"^ID[0-9A-Fa-f]{2,3}(?=[A-Z])", "", m.group(2))
            cur = Message(cid, name, int(m.group(3)), src)
            db[(cid, ext)] = cur
            added += 1
            continue
        m = re_sg.match(line)
        if m and cur is not None:
            mux = m.group(2)
            if mux and mux.endswith("M") and mux != "M":
                mux = mux[:-1]  # extended multiplexing: treat as plain mN
            cur.signals.append(Signal(m.group(1), mux, int(m.group(3)), int(m.group(4)),
                                      m.group(5) == "1", m.group(6) == "-",
                                      float(m.group(7)), float(m.group(8)), m.group(9)))
            continue
        if not line.strip():
            cur = None
    for m in re.finditer(r"^VAL_\s+(\d+)\s+(\w+)\s+((?:-?\d+\s+\"[^\"]*\"\s*)+);", txt, re.M):
        key = (int(m.group(1)) & 0x1FFFFFFF, bool(int(m.group(1)) & 0x80000000))
        msg = db.get(key)
        if not msg or msg.source != src:
            continue
        table = {int(a): b for a, b in re.findall(r"(-?\d+)\s+\"([^\"]*)\"", m.group(3))}
        for s in msg.signals:
            if s.name == m.group(2):
                s.values = table
    return added


# ----------------------------------------------------------------------------
# Reading captures
# ----------------------------------------------------------------------------
Frame = collections.namedtuple("Frame", "seq t port id ext rtr dlc data")
RE_F = re.compile(r"^F([0-9A-F]{8}),([0-9A-F]{8}),([AB]),([0-9A-F]{3}|[0-9A-F]{8}),(S|X)(R?),(\d{1,2}),([0-9A-F]*)\s*$")
RE_OLD = re.compile(r"^t=(\d+) us (A_READ|B_RX_FRAME)\s+result=0 .*?ID=0x([0-9A-F]+) (STD|EXT) (DATA|RTR) DLC=(\d+) DATA=(.*)$")


def read_capture(path):
    frames, notes, malformed, markers = [], [], 0, []
    old_seen = set()
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\r\n")
            if line.startswith("F"):
                m = RE_F.match(line)
                if not m:
                    malformed += 1
                    continue
                dlc = int(m.group(7))
                rtr = m.group(6) == "R"
                hexd = m.group(8)
                if (rtr and hexd) or (not rtr and len(hexd) != 2 * min(dlc, 8)):
                    malformed += 1
                    continue
                frames.append(Frame(int(m.group(1), 16), int(m.group(2), 16), m.group(3),
                                    int(m.group(4), 16), m.group(5) == "X", rtr, dlc,
                                    bytes.fromhex(hexd)))
            elif line.startswith("C,"):
                markers.append(int(line[2:10], 16))
            elif line.startswith("#"):
                notes.append(line)
            else:
                m = RE_OLD.match(line.strip())
                if m:  # fault-report trace line from older firmware (no seq numbers)
                    key = (m.group(1), m.group(2), m.group(3), m.group(7))
                    if key in old_seen:
                        continue
                    old_seen.add(key)
                    data = b"" if m.group(5) == "RTR" else bytes.fromhex(m.group(7).replace(" ", "")) if m.group(7) != "<none>" else b""
                    frames.append(Frame(None, int(m.group(1)), "A" if m.group(2) == "A_READ" else "B",
                                        int(m.group(3), 16), m.group(4) == "EXT", m.group(5) == "RTR",
                                        int(m.group(6)), data))
    return frames, notes, malformed, markers


def unwrap_times(frames):
    """micros() wraps every 71.6 min; produce seconds from the first frame."""
    out, base, prev = [], 0, None
    for f in frames:
        if prev is not None and f.t < prev and prev - f.t > 0x80000000:
            base += 1 << 32
        prev = f.t
        out.append(base + f.t)
    t0 = out[0] if out else 0
    return [(x - t0) / 1e6 for x in out]


# ----------------------------------------------------------------------------
# Decode + reports
# ----------------------------------------------------------------------------
def describe(db, f):
    msg = db.get((f.id, f.ext))
    if msg:
        return msg.name, msg.source, msg
    if not f.ext and f.id in NAME_HINTS:
        return NAME_HINTS[f.id] + " (name only)", "community name hint", None
    return "unknown", "", None


def fmt_id(f):
    return f"{f.id:08X}" if f.ext else f"{f.id:03X}"


def decode(path, dbc_paths, port_names, max_events_per_key, show_analog=False):
    db = {}
    for p in dbc_paths:
        if os.path.exists(p):
            n = load_dbc(p, db)
            print(f"DBC {os.path.basename(p)}: {n} messages")
        else:
            print(f"DBC {p} not found - names will be missing")
    frames, notes, malformed, markers = read_capture(path)
    if not frames:
        print("No frames found in", path)
        return 1
    has_seq = frames[0].seq is not None
    dups = 0
    if has_seq:
        frames.sort(key=lambda f: f.seq)
        uniq = []
        for f in frames:                       # a line received twice counts once
            if uniq and uniq[-1].seq == f.seq:
                dups += 1
                continue
            uniq.append(f)
        frames = uniq
    else:
        frames.sort(key=lambda f: f.t)   # old traces: board time stamps, gaps between samples
    times = unwrap_times(frames)
    base = os.path.splitext(path)[0]

    # ---- integrity
    print(f"\n{len(frames)} frames read from {os.path.basename(path)}"
          + (f", {malformed} malformed lines ignored" if malformed else "")
          + (f", {dups} duplicate lines removed" if dups else ""))
    if has_seq:
        gaps, missing = [], 0
        for a, b in zip(frames, frames[1:]):
            if b.seq != a.seq + 1:
                if b.seq in markers or any(a.seq < mk <= b.seq for mk in markers):
                    continue  # deliberate restart of the capture ('c')
                gaps.append((a.seq, b.seq))
                missing += b.seq - a.seq - 1
        span = frames[-1].seq - frames[0].seq + 1
        print(f"sequence {frames[0].seq}..{frames[-1].seq}: "
              + ("COMPLETE - no frame missing" if not gaps else
                 f"{missing} frames NOT recorded in {len(gaps)} gap(s) (USB too slow / host not reading); "
                 f"forwarding was not affected"))
        print(f"duration {times[-1]:.3f} s, {span} frames received by the bridge in that span")
        caplost = [n for n in notes if "CAPLOST=" in n]
        if caplost:
            print("last status line:", caplost[-1])
    else:
        print("old-format log (fault-report traces): samples only, no sequence numbers or rates")

    # ---- per ID statistics
    per = collections.OrderedDict()
    for f, t in zip(frames, times):
        k = (f.port, f.id, f.ext)
        s = per.get(k)
        if s is None:
            s = per[k] = {"n": 0, "first_t": t, "last_t": t, "dlcs": collections.Counter(),
                          "payloads": collections.Counter(), "first": f, "last": f, "rtr": 0}
        s["n"] += 1
        s["last_t"] = t
        s["dlcs"][f.dlc] += 1
        s["payloads"][f.data] += 1
        s["last"] = f
        s["rtr"] += f.rtr

    # ---- which signals / bytes change "slowly" (event-like) vs continuously
    changes = collections.defaultdict(lambda: collections.Counter())   # key -> item -> count
    samples = collections.Counter()
    last_items = {}
    decoded_rows = []
    analog = set()   # (key, signal) with a physical unit: V, A, C, kW ...
    for f, t in zip(frames, times):
        name, src, msg = describe(db, f)
        # multiplexed messages: compare frames of the same multiplexer value only
        k = (f.port, f.id, f.ext, msg.selector(f.data) if (msg and not f.rtr) else None)
        items = {}
        if msg and not f.rtr:
            for sname, raw, text in msg.decode(f.data):
                items[sname] = text
            for sg in msg.signals:
                if sg.unit and not sg.values:
                    analog.add((k, sg.name))
        else:
            for i, b in enumerate(f.data):
                items[f"byte{i}"] = f"{b:02X}"
        prev = last_items.get(k)
        if prev is not None:
            samples[k] += 1
            for it, v in items.items():
                if it in prev and prev[it] != v:
                    changes[k][it] += 1
        last_items[k] = items
        decoded_rows.append((f, t, name, items, k))

    def fast(k, it):
        n = samples[k]
        return n >= 10 and changes[k][it] > 0.3 * n

    def hidden(k, it):
        return fast(k, it) or (not show_analog and (k, it) in analog)

    # ---- ids.csv + console table
    ids_path = base + "_ids.csv"
    with open(ids_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["port", "port_meaning", "id", "format", "count", "rate_hz", "period_ms", "dlc",
                    "name", "name_source", "known_cp_ecu_frame", "distinct_payloads",
                    "first_payload", "last_payload", "continuously_changing"])
        print(f"\n{'port':4} {'id':>8} {'count':>7} {'Hz':>7} {'dlc':>3}  {'name':34} {'payloads':>8}  last payload")
        for (port, cid, ext), s in sorted(per.items(), key=lambda kv: (kv[0][0], kv[0][1])):
            f = s["first"]
            dur = s["last_t"] - s["first_t"]
            rate = (s["n"] - 1) / dur if has_seq and dur > 0 else None
            name, src, msg = describe(db, f)
            fastitems = sorted({it for kk in changes if kk[:3] == (port, cid, ext)
                                for it in changes[kk] if fast(kk, it)})
            dlc = "/".join(str(d) for d in sorted(s["dlcs"]))
            w.writerow([port, port_names.get(port, ""), fmt_id(f), ("EXT" if ext else "STD") + ("+RTR" if s["rtr"] else ""),
                        s["n"], f"{rate:.2f}" if rate else "", f"{1000 / rate:.1f}" if rate else "", dlc,
                        name, src, "yes" if (not ext and cid in CP_ECU_IDS) else "",
                        len(s["payloads"]), s["first"].data.hex(" ").upper(), s["last"].data.hex(" ").upper(),
                        " ".join(fastitems)])
            print(f"{port:4} {fmt_id(f):>8} {s['n']:7d} {(f'{rate:7.1f}' if rate else '      -')} {dlc:>3}  "
                  f"{name[:34]:34} {len(s['payloads']):8d}  {s['last'].data.hex(' ').upper()}")

    # ---- events.txt: changes of slow signals, in time order
    ev_path = base + "_events.txt"
    count_per = collections.Counter()
    event_list = []

    def msg_known(f):
        return (f.id, f.ext) in db

    with open(ev_path, "w") as fh:
        fh.write(f"Changes of slowly-changing signals/bytes, {os.path.basename(path)}\n")
        fh.write("Ports: " + ", ".join(f"{p} = {n}" for p, n in port_names.items()) + "\n")
        fh.write("Signals that change in more than 30% of frames (counters, checksums, measurements)\n"
                 "are left out here; they are listed per ID in the _ids.csv file.\n"
                 + ("" if show_analog else "Measurements with a unit (V, A, C, kW...) are left out too (--show-analog shows them).\n")
                 + "\n")
        last = {}
        for f, t, name, items, k in decoded_rows:
            prev = last.get(k)
            last[k] = items
            if prev is None:
                shown = {it: v for it, v in items.items() if not fast(k, it)}
                fh.write(f"{t:12.6f}s  {f.port} {fmt_id(f):>8} {name}: FIRST  "
                         + ", ".join(f"{it}={v}" for it, v in shown.items()) + "\n")
                continue
            diff = [(it, prev.get(it), v) for it, v in items.items()
                    if it in prev and prev[it] != v and not hidden(k, it)]
            if not diff:
                continue
            event_list.append((t, f.port, fmt_id(f), name, msg_known(f), diff))
            count_per[k[:3]] += 1
            if count_per[k[:3]] > max_events_per_key:
                if count_per[k[:3]] == max_events_per_key + 1:
                    fh.write(f"{t:12.6f}s  {f.port} {fmt_id(f):>8} {name}: ... more changes not shown "
                             f"(use --max-events)\n")
                continue
            fh.write(f"{t:12.6f}s  {f.port} {fmt_id(f):>8} {name}: "
                     + ", ".join(f"{it} {a} -> {b}" for it, a, b in diff) + "\n")

    # ---- reactions: what changed on the vehicle side just before each
    # charge-port state change (this is where commands show up)
    re_path = base + "_reactions.txt"
    with open(re_path, "w") as fh:
        fh.write("Every state change of the charge port ECU (port B) and the vehicle-side (port A)\n"
                 "changes in the 0.5 s before it. A request that the ECU acts on appears here.\n"
                 "Only messages named in the DBC are listed on the vehicle side.\n\n")
        a_events = [e for e in event_list if e[1] == "A" and e[4]]
        j = 0
        for e in event_list:
            if e[1] != "B":
                continue
            fh.write(f"{e[0]:12.6f}s  B {e[2]:>8} {e[3]}: " + ", ".join(f"{it} {a} -> {b}" for it, a, b in e[5]) + "\n")
            for ae in a_events:
                if e[0] - 0.5 <= ae[0] <= e[0]:
                    fh.write(f"      {ae[0] - e[0]:+.3f}s  A {ae[2]:>8} {ae[3]}: "
                             + ", ".join(f"{it} {a} -> {b}" for it, a, b in ae[5]) + "\n")

    # ---- decoded.csv: every frame
    dec_path = base + "_decoded.csv"
    with open(dec_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["seq", "time_s", "port", "id", "format", "dlc", "data", "name", "signals"])
        for f, t, name, items, _k in decoded_rows:
            w.writerow(["" if f.seq is None else f.seq, f"{t:.6f}" if has_seq else "", f.port, fmt_id(f),
                        ("X" if f.ext else "S") + ("R" if f.rtr else ""), f.dlc, f.data.hex(" ").upper(), name,
                        "; ".join(f"{it}={v}" for it, v in items.items())])

    known = sum(1 for k in per if (k[1], k[2]) in db)
    print(f"\n{len(per)} distinct IDs, {known} named and decoded by the DBC")
    print("wrote", ids_path)
    print("wrote", ev_path)
    print("wrote", re_path)
    print("wrote", dec_path)
    return 0


# ----------------------------------------------------------------------------
# Capture from the board
# ----------------------------------------------------------------------------
def list_ports():
    from serial.tools import list_ports as lp
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found. Check the USB cable (must be a data cable) and that the board is powered.")
    for p in ports:
        tag = "  <-- ESP32-S3 (T-2CAN)" if p.vid == 0x303A else ""
        vidpid = f"VID:PID={p.vid:04X}:{p.pid:04X}" if p.vid is not None else ""
        print(f"  {p.device:12} {p.description} {vidpid}{tag}")
    return ports


def find_board():
    from serial.tools import list_ports as lp
    esp = [p.device for p in lp.comports() if p.vid == 0x303A]   # Espressif USB
    return esp[0] if len(esp) == 1 else None


def capture(port, out, seconds, args):
    try:
        import serial  # pyserial
    except ImportError:
        print("pyserial missing:  pip install pyserial")
        return 1
    if not port:
        port = find_board()
        if not port:
            print("Could not pick the board automatically. Serial ports on this PC:")
            list_ports()
            print("Run again with --port <one of the names above>.")
            return 1
        print("found T-2CAN on", port)
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except serial.SerialException as e:
        print(f"Could not open {port}: {e}")
        print("Serial ports on this PC:")
        list_ports()
        print("If the board is listed, close the Arduino IDE Serial Monitor / any other program using it and retry.")
        return 1
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"c")           # board drops its backlog and starts a fresh capture
    ser.flush()
    print(f"capturing from {port} into {out}   (Ctrl+C to stop)")
    n_frames, last_seq, gaps, missing = 0, None, 0, 0
    per_port = collections.Counter()
    started = time.time()
    last_print = started
    buf = b""
    try:
        with open(out, "wb") as fh:
            while True:
                chunk = ser.read(65536)
                if chunk:
                    fh.write(chunk)
                    buf += chunk
                    *lines, buf = buf.split(b"\n")
                    for line in lines:
                        if line.startswith(b"F") and len(line) > 20:
                            try:
                                seq = int(line[1:9], 16)
                            except ValueError:
                                continue
                            n_frames += 1
                            per_port[line[19:20].decode(errors="replace")] += 1
                            if last_seq is not None and seq != last_seq + 1:
                                gaps += 1
                                missing += max(0, seq - last_seq - 1)
                            last_seq = seq
                        elif line.startswith(b"C,"):
                            last_seq = None
                now = time.time()
                if now - last_print >= 2:
                    last_print = now
                    print(f"  {now - started:7.1f}s  frames={n_frames}  A={per_port['A']}  B={per_port['B']}  "
                          f"missing={missing}", flush=True)
                if seconds and now - started >= seconds:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print(f"saved {out}: {n_frames} frames, {missing} missing in {gaps} gap(s)")
    return decode(out, args.dbc, PORT_NAMES, args.max_events, args.show_analog)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="record from the T-2CAN over USB, then decode")
    c.add_argument("--port", help="serial port, e.g. COM5 (default: find the ESP32-S3 automatically)")
    sub.add_parser("ports", help="list serial ports and show which one is the T-2CAN")
    c.add_argument("--out", help="output log file (default capture_<date>_<time>.log)")
    c.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (default: Ctrl+C)")
    d = sub.add_parser("decode", help="decode a saved capture")
    d.add_argument("log")
    for p in (c, d):
        p.add_argument("--dbc", action="append", default=None,
                       help="DBC file(s); first one wins for an ID (default: Model3CAN.dbc next to this script)")
        p.add_argument("--max-events", type=int, default=200,
                       help="max changes listed per ID in _events.txt (default 200)")
        p.add_argument("--show-analog", action="store_true",
                       help="also list changes of measurements (voltages, currents, temperatures)")
    args = ap.parse_args()
    if args.cmd == "ports":
        try:
            list_ports()
        except ImportError:
            print("pyserial missing:  pip install pyserial")
        return 0
    if args.dbc is None:
        args.dbc = [DEFAULT_DBC]
    elif DEFAULT_DBC not in args.dbc:
        args.dbc = [DEFAULT_DBC] + args.dbc
    if args.cmd == "ports":
        try:
            list_ports()
        except ImportError:
            print("pyserial missing:  pip install pyserial")
        return 0
    if args.cmd == "capture":
        out = args.out or datetime.datetime.now().strftime("capture_%Y%m%d_%H%M%S.log")
        return capture(args.port, out, args.seconds, args)
    return decode(args.log, args.dbc, PORT_NAMES, args.max_events, args.show_analog)


if __name__ == "__main__":
    sys.exit(main())
