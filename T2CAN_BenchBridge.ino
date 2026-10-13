/*
 * T2CAN_BenchBridge AUTO v6 -- H784 / MCP2515 + ESP32-S3 TWAI
 * Keep your existing pin_config.h beside this file, unchanged.
 * Dependency: autowp-mcp2515 1.3.1 (tested against that exact version).
 *
 * WHAT IT DOES
 *   Two-port classic-CAN gateway, 500 kbit/s on both ports:
 *     Port A = MCP2515 (SPI)        Port B = ESP32-S3 TWAI
 *   Every valid frame received on one port is re-transmitted unchanged
 *   (same ID, standard/extended, data/RTR, DLC, payload) on the other.
 *   It starts by itself at power-up. No button, no Serial command.
 *   Put the Model 3 charge port ECU on one port and the X096 side on the
 *   other. The two ports must be two separate, correctly terminated
 *   CAN segments. Never wire both ports to the same bus.
 *
 * ADDED IN v6: EXACT FRAME CAPTURE
 *   Every frame received on either port is recorded (port, receive time in
 *   microseconds, ID, standard/extended, data/RTR, DLC, all data bytes) with
 *   a sequence number, kept in PSRAM (about 170,000 frames of backlog) and
 *   streamed over USB as one line per frame:
 *     F<seq 8 hex>,<time_us 8 hex>,<A|B>,<id hex>,<S|X>[R],<dlc>,<data hex>
 *   e.g. F0000002A,0371B2C4,B,25D,S,8,0000000000000000
 *   Port A = MCP2515 (X096 / vehicle side), port B = TWAI (charge port ECU),
 *   as seen in your logs. Nothing is decoded on the board. Use t2can_log.py
 *   on the PC to save the stream and name / decode every frame.
 *   Sequence numbers make any gap visible: if USB cannot keep up and the
 *   backlog fills, frames are still forwarded but not recorded, and the gap
 *   shows in the sequence numbers and in CAPLOST on the status line.
 *   Capture never delays forwarding. Send c to drop the backlog and start a
 *   fresh capture (t2can_log.py does this when it starts).
 *   Status lines start with '#', so they never look like frame lines.
 *
 * ADDED IN v5 (found by running this sketch and the real autowp library
 * against a simulated MCP2515 / TWAI / CAN bus)
 *   - The MCP2515 has two receive buffers. Under overload the OLDER frame can
 *     sit in RXB1 while a NEWER one lands in RXB0; always reading RXB0 first
 *     would deliver them out of order. v5 tracks which buffer is older.
 *   - A fault no longer holds up the restart. The report is rendered into
 *     RAM during the 50 ms cool-down and streamed to USB afterwards in small
 *     pieces while forwarding already runs again (restart ~0.1 s, was ~0.5 s
 *     with a USB host attached).
 *   - d / ? no longer stops the bridge. It prints the live status and the
 *     last fault report while forwarding continues.
 *
 * FIXED IN v4 (from a real bench log)
 *   MCP2515 transmit-submit used to be treated as a fault whenever the
 *   autowp library returned ERROR_FAILTX. On a busy bus that return code
 *   just means "arbitration was already lost once"; TXB0CTRL still has
 *   TXREQ set and the chip retries by itself. v4 checks TXB0CTRL and
 *   only faults if the frame really is not pending. This was the cause
 *   of the repeated restarts (A_TX_SUBMIT result=4, TXB0CTRL=0x28).
 *   Also: a sleeping side needs time to wake and start ACKing, so the
 *   delivery timeout is now 500 ms (was 100) and the restart cool-down
 *   is 50 ms (was 200).
 *
 * AUTONOMOUS OPERATION (since v3)
 *   - Auto-starts ~300 ms after power-up.
 *   - Faults no longer latch. On a genuine loss-of-service fault the
 *     bridge stops both controllers, saves a snapshot, renders a report
 *     into RAM, waits 50 ms, fully re-initialises both controllers and
 *     resumes forwarding. The report is sent to USB afterwards, and only
 *     if a host is listening.
 *   - While the TWAI driver is not installed (power-up, restart window) the
 *     TX pin is held recessive, so segment B is never held dominant.
 *   - Non-fatal conditions are counted, not fatal: bus errors, error
 *     passive, lost frames (RX overflow), full software queue, malformed
 *     frames.
 *   - Fatal (auto-restart): bus-off on either port, TWAI not running,
 *     MCP2515 SPI/receive failure, transmit not completed within 500 ms
 *     (no ACK / no peer / wrong bitrate), inconsistent controller status.
 *   - Serial output never blocks forwarding: bytes are only written when the
 *     USB TX buffer has room, a few at a time, and dropped if no host listens.
 *
 * LIMITS (unchanged and real)
 *   Not a transparent electrical connection: store-and-forward adds
 *   latency, changes arbitration/timing, and has NO hardware bypass.
 *   A reset, power loss or restart window interrupts communication.
 *   Polling MCP2515 (two hardware RX buffers), finite queues, no CAN FD,
 *   no classic DLC>8. Not vehicle-validated. No Supercharging unlock logic.
 *   If the peer segment has no powered node to ACK, forwarding into it
 *   cannot complete and the bridge will cycle through restarts until the
 *   peer is present.
 *
 * Arduino IDE: ESP32S3 Dev Module; USB CDC On Boot Enabled;
 * USB Mode Hardware CDC and JTAG; Flash 16MB; QIO 80MHz;
 * Partition 16M Flash (3MB APP/9.9MB FATFS); PSRAM OPI PSRAM.
 * Hardware assumptions: genuine H784 with 16MHz MCP2515 oscillator;
 * both CAN segments 500kbit/s. Confirm these on the actual hardware.
 *
 * Optional diagnostics (never required): Serial Monitor 115200.
 *   One status line per second. Faults print a report automatically.
 *   Send d or ? to print the status and the last fault report again;
 *   forwarding is not interrupted. With capture on, use t2can_log.py rather
 *   than the Serial Monitor (about 2,000 lines per second).
 */

#include <Arduino.h>
#include <SPI.h>
#include <cstring>
#include <cstdarg>
#include <cstdio>
#include "driver/twai.h"
#include "mcp2515.h"
#include "pin_config.h"

#if !defined(T_2Can) || defined(T_2Can_Fd)
#error "Select T_2Can only in your existing pin_config.h (H784, not FD)."
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select ESP32S3 Dev Module."
#endif

static_assert(CAN_TX == 7 && CAN_RX == 6, "Unexpected H784 TWAI pinmap");
static_assert(MCP2515_CS == 10 && MCP2515_RST == 9,
              "Unexpected H784 MCP2515 pinmap");
static_assert(MCP2515_SCLK == 12 && MCP2515_MISO == 13 &&
              MCP2515_MOSI == 11, "Unexpected H784 SPI pinmap");

constexpr uint32_t SPI_HZ = 10000000;
constexpr CAN_CLOCK OSCILLATOR = MCP_16MHZ;
constexpr size_t QUEUE_SIZE = 64;
constexpr uint32_t START_DELAY_MS = 300;        // power-up settle before auto-start
constexpr uint32_t RESTART_DELAY_MS = 50;       // cool-down after a fault
constexpr uint32_t MAX_TX_MS = 500;             // receive-to-delivered limit (a sleeping peer needs time to wake)
constexpr uint32_t MAX_QUEUE_AGE_MS = 600;      // queued frame age limit
constexpr uint32_t FULL_DUMP_MIN_GAP_MS = 5000; // rate limit for long reports
constexpr uint32_t STATUS_PERIOD_MS = 1000;
constexpr size_t REPORT_CAP = 32768;            // RAM for the last fault report
constexpr size_t LINE_CAP = 768;                // RAM for one short message
// true: a fault prints the full report over USB (if a host is listening),
// at most once per FULL_DUMP_MIN_GAP_MS. false: one short line only.
constexpr bool PRINT_FULL_REPORT_ON_FAULT = true;
// Exact frame capture (see header). false: forwarding and diagnostics only.
constexpr bool CAPTURE_ENABLED = true;
constexpr size_t CAPTURE_PSRAM_BYTES = 4u * 1024u * 1024u;  // ~170k frames
constexpr size_t CAPTURE_FALLBACK_RECORDS = 3000;           // if no PSRAM

MCP2515 canA(MCP2515_CS, SPI_HZ, &SPI);

enum class BridgeState { WAITING, RUNNING };
BridgeState state = BridgeState::WAITING;
bool twaiInstalled = false;
bool spiReady = false;
const char *lastReason = "Boot: auto-start pending";
const char *lastFault = "none";
uint32_t restartAt = 0;

struct QueuedFrame {
  can_frame frame;
  uint32_t receivedAt;
};

struct FrameQueue {
  QueuedFrame entries[QUEUE_SIZE] = {};
  size_t head = 0;
  size_t count = 0;

  bool push(const can_frame &f, uint32_t at) {
    if (count == QUEUE_SIZE) return false;
    entries[(head + count) % QUEUE_SIZE] = {f, at};
    ++count;
    return true;
  }
  QueuedFrame &front() { return entries[head]; }
  void pop() {
    head = (head + 1) % QUEUE_SIZE;
    --count;
  }
  void clear() {
    head = 0;
    count = 0;
  }
};

FrameQueue toA;
FrameQueue toB;
bool inFlightA = false;
bool inFlightB = false;
uint32_t flightAgeA = 0;
uint32_t flightAgeB = 0;
uint32_t rxA = 0, rxB = 0;
uint32_t queuedA = 0, queuedB = 0;
uint32_t doneA = 0, doneB = 0;
uint32_t overflowA = 0;                 // MCP2515 RX overflow events (>=1 frame each)
uint32_t lostB = 0;                     // TWAI frames lost (missed + overrun)
uint32_t droppedBad = 0, droppedFull = 0;
uint32_t arbLossA = 0;                  // MCP2515 submits that reported FAILTX but were pending (normal arbitration loss)
uint32_t faultCount = 0, startCount = 0;
uint32_t lastReport = 0;
uint32_t lastHealthCheck = 0;
uint32_t lastFullDump = 0;
bool everDumped = false;
bool rxb1Older = false;                 // MCP2515: RXB1 holds an older frame than RXB0
uint8_t lastEflg = 0;
uint32_t lastAlerts = 0;
uint32_t alertsRecorded = 0;
uint32_t baseMissed = 0, baseOverrun = 0;

// A bounded, memory-only event trace. Nothing prints in recordEvent().
constexpr size_t TRACE_CAPACITY = 256;
struct TraceEvent {
  uint32_t timeUs;
  const char *name;
  int32_t result;
  uint16_t depthA, depthB;
  bool hasFrame;
  can_frame frame;
};
TraceEvent traceBuffer[TRACE_CAPACITY] = {};
size_t traceNext = 0, traceUsed = 0;
uint32_t traceOverwritten = 0;
size_t highWaterA = 0, highWaterB = 0;
uint32_t maxLoopGapUs = 0, lastLoopUs = 0;
uint32_t allAlertsSeen = 0;

// Capture status BEFORE stop/reset alters it.
struct FaultSnapshot {
  bool captured = false;
  uint32_t beginUs = 0, endUs = 0;
  bool hadA = false, hadB = false;
  esp_err_t statusResult = ESP_FAIL;
  twai_status_info_t twai = {};
  bool haveMcp = false;
  uint8_t mcp[128] = {};
  int rxLogic = -1, mcpIntLogic = -1;
  esp_err_t stopResult = ESP_FAIL;
};
FaultSnapshot faultSnapshot;

// Output goes into RAM first (sinkBuf) and is streamed to USB by pumpSerial().
char reportBuf[REPORT_CAP];             // last fault report
size_t reportLen = 0, reportPos = 0;
char lineBuf[LINE_CAP];                 // one short message (fault line / status)
size_t lineLen = 0, linePos = 0;
char *sinkBuf = nullptr;
size_t sinkCap = 0, sinkLen = 0;
char statusBuf[256];                    // periodic status line
size_t statusLen = 0, statusPos = 0;

// Frame capture ring (PSRAM when available). Single-threaded: written in
// collectFrames(), read in pumpCapture(), both from loop().
struct CaptureRecord {
  uint32_t seq;
  uint32_t timeUs;
  uint32_t id;
  uint8_t info;      // bits 0-3 DLC, bit 4 extended, bit 5 RTR, bit 7 port B
  uint8_t data[8];
};
CaptureRecord *capRing = nullptr;
size_t capCapacity = 0, capHead = 0, capCount = 0;
bool capInPsram = false;
uint32_t capSeq = 0;          // sequence number of the next received frame
uint32_t capLost = 0;         // backlog full: forwarded but not recorded
uint32_t capSkipped = 0;      // no USB host, or dropped by 'c'
uint32_t capSent = 0;
char capPend[640];            // formatted whole lines, partly written
size_t capPendLen = 0, capPendPos = 0;
bool capRestartRequested = false;

// Explicit declarations keep Arduino's generated prototypes unambiguous.
void recordEvent(const char *name, int32_t result, const can_frame *frame);
void captureSnapshot();
void dumpDiagnostics();
void printFrame(const can_frame &frame);
void printStatus();
void decodeAlerts(uint32_t flags);
void sinkBegin(char *buffer, size_t capacity);
size_t sinkEnd();
void outWrite(const char *text, size_t length);
void outf(const char *fmt, ...);
bool serialIdle();
void pumpSerial();
void captureInit();
void captureFrame(bool portB, uint32_t id, bool ext, bool rtr, uint8_t dlc,
                  const uint8_t *data);
size_t formatCapture(const CaptureRecord &r, char *out);
void pumpCapture();
void mcpBitModify(uint8_t address, uint8_t mask, uint8_t value);
uint8_t mcpRead(uint8_t address);
void parkCanTx();
void quiesceHardware();
void faultStop(const char *reason);
void reportFault();
bool startBridge();
bool checkHealth();
bool validFrame(const can_frame &f);
twai_message_t toTwai(const can_frame &f);
can_frame fromTwai(const twai_message_t &m);
void collectFrames();
void collectCompletions();
void submitFrames();
void periodicReport();
void handleSerial();

void recordEvent(const char *name, int32_t result, const can_frame *frame) {
  TraceEvent &e = traceBuffer[traceNext];
  e.timeUs = micros();
  e.name = name;
  e.result = result;
  e.depthA = (uint16_t)toA.count;
  e.depthB = (uint16_t)toB.count;
  e.hasFrame = frame != nullptr;
  e.frame = {};
  if (frame) e.frame = *frame;
  traceNext = (traceNext + 1) % TRACE_CAPACITY;
  if (traceUsed < TRACE_CAPACITY) ++traceUsed;
  else ++traceOverwritten;
  if (toA.count > highWaterA) highWaterA = toA.count;
  if (toB.count > highWaterB) highWaterB = toB.count;
}

// ---------------------------------------------------------------------------
// Output. Text is first written into a RAM buffer (sinkBegin / outf /
// sinkEnd). pumpSerial() then streams it to USB a few bytes per loop pass,
// and only while the USB TX buffer has room. Nothing here waits for a host,
// so reports never delay forwarding or the restart.
// ---------------------------------------------------------------------------
void sinkBegin(char *buffer, size_t capacity) {
  sinkBuf = buffer;
  sinkCap = capacity;
  sinkLen = 0;
}

size_t sinkEnd() {
  const size_t n = sinkLen;
  sinkBuf = nullptr;
  return n;
}

void outWrite(const char *text, size_t length) {
  if (!sinkBuf) return;
  static const char marker[] = "\n[report truncated]\n";
  if (sinkLen + length + sizeof(marker) > sinkCap) {
    if (sinkLen + sizeof(marker) <= sinkCap) {
      memcpy(sinkBuf + sinkLen, marker, sizeof(marker) - 1);
      sinkLen += sizeof(marker) - 1;
    }
    sinkCap = sinkLen;   // full: every later write is ignored
    return;
  }
  memcpy(sinkBuf + sinkLen, text, length);
  sinkLen += length;
}

void outf(const char *fmt, ...) {
  if (!sinkBuf) return;
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  const int n = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  if (n <= 0) return;
  const size_t length = (size_t)n >= sizeof(buffer) ? sizeof(buffer) - 1
                                                      : (size_t)n;
  outWrite(buffer, length);
}

bool serialIdle() {
  return linePos >= lineLen && reportPos >= reportLen;
}

// Called every loop pass; never waits. Order: finish a started block of
// capture lines (so a frame line is never split), then fault line / report,
// then the status line, then new capture lines.
void pumpSerial() {
  if (!Serial) {            // no host listening: drop, never accumulate
    linePos = lineLen;
    reportPos = reportLen;
    statusPos = statusLen;
    capPendPos = capPendLen;
    pumpCapture();          // counts and discards the capture backlog
    return;
  }
  if (capPendPos < capPendLen) {
    pumpCapture();
    return;
  }
  const char *source = nullptr;
  size_t *pos = nullptr;
  size_t len = 0;
  if (linePos < lineLen) { source = lineBuf; pos = &linePos; len = lineLen; }
  else if (reportPos < reportLen) { source = reportBuf; pos = &reportPos; len = reportLen; }
  else if (statusPos < statusLen) { source = statusBuf; pos = &statusPos; len = statusLen; }
  if (!source) {
    pumpCapture();
    return;
  }
  const int space = Serial.availableForWrite();
  if (space <= 0) return;
  size_t n = len - *pos;
  if (n > (size_t)space) n = (size_t)space;
  *pos += Serial.write((const uint8_t *)source + *pos, n);
}

// ---------------------------------------------------------------------------
// Exact frame capture.
// ---------------------------------------------------------------------------
void captureInit() {
  if (!CAPTURE_ENABLED || capRing) return;
  if (psramFound()) {
    capRing = (CaptureRecord *)ps_malloc(CAPTURE_PSRAM_BYTES);
    if (capRing) {
      capCapacity = CAPTURE_PSRAM_BYTES / sizeof(CaptureRecord);
      capInPsram = true;
    }
  }
  if (!capRing) {
    capRing = (CaptureRecord *)malloc(CAPTURE_FALLBACK_RECORDS * sizeof(CaptureRecord));
    if (capRing) capCapacity = CAPTURE_FALLBACK_RECORDS;
  }
}

// Called for every frame read from either controller, before any filtering.
void captureFrame(bool portB, uint32_t id, bool ext, bool rtr, uint8_t dlc,
                  const uint8_t *data) {
  const uint32_t seq = capSeq++;
  if (!capCapacity) return;
  if (capCount == capCapacity) {     // backlog full: gap in seq shows it
    ++capLost;
    return;
  }
  CaptureRecord &r = capRing[(capHead + capCount) % capCapacity];
  r.seq = seq;
  r.timeUs = micros();
  r.id = id & (ext ? 0x1FFFFFFFu : 0x7FFu);
  r.info = (uint8_t)((dlc & 0x0F) | (ext ? 0x10 : 0) | (rtr ? 0x20 : 0) |
                     (portB ? 0x80 : 0));
  memset(r.data, 0, sizeof(r.data));
  if (!rtr) memcpy(r.data, data, dlc > 8 ? 8 : dlc);
  ++capCount;
}

static char *putHex(char *p, uint32_t v, unsigned digits) {
  static const char hex[] = "0123456789ABCDEF";
  for (int i = (int)digits - 1; i >= 0; --i) p[i] = hex[v & 0xF], v >>= 4;
  return p + digits;
}

// F<seq>,<time>,<port>,<id>,<S|X>[R],<dlc>,<data>\n  (at most 52 chars)
size_t formatCapture(const CaptureRecord &r, char *out) {
  char *p = out;
  const bool ext = r.info & 0x10, rtr = r.info & 0x20;
  const unsigned dlc = r.info & 0x0F;
  *p++ = 'F';
  p = putHex(p, r.seq, 8);
  *p++ = ',';
  p = putHex(p, r.timeUs, 8);
  *p++ = ',';
  *p++ = (r.info & 0x80) ? 'B' : 'A';
  *p++ = ',';
  p = putHex(p, r.id, ext ? 8 : 3);
  *p++ = ',';
  *p++ = ext ? 'X' : 'S';
  if (rtr) *p++ = 'R';
  *p++ = ',';
  if (dlc >= 10) *p++ = '1';
  *p++ = (char)('0' + dlc % 10);
  *p++ = ',';
  if (!rtr)
    for (unsigned i = 0; i < dlc && i < 8; ++i) p = putHex(p, r.data[i], 2);
  *p++ = '\n';
  return (size_t)(p - out);
}

void pumpCapture() {
  if (!capCapacity) return;
  if (!Serial) {                       // nobody to send to: discard
    capSkipped += (uint32_t)capCount;
    capHead = (capHead + capCount) % capCapacity;
    capCount = 0;
    return;
  }
  if (capPendPos >= capPendLen) {      // previous block fully sent
    capPendLen = capPendPos = 0;
    if (capRestartRequested) {         // 'c': fresh capture from here
      capRestartRequested = false;
      capSkipped += (uint32_t)capCount;
      capHead = (capHead + capCount) % capCapacity;
      capCount = 0;
      capPendLen = (size_t)snprintf(capPend, sizeof(capPend),
          "C,%08lX\n", (unsigned long)capSeq);
    }
    while (capCount && capPendLen + 64 <= sizeof(capPend)) {
      capPendLen += formatCapture(capRing[capHead], capPend + capPendLen);
      capHead = (capHead + 1) % capCapacity;
      --capCount;
      ++capSent;
    }
  }
  if (capPendPos >= capPendLen) return;
  const int space = Serial.availableForWrite();
  if (space <= 0) return;
  size_t n = capPendLen - capPendPos;
  if (n > (size_t)space) n = (size_t)space;
  capPendPos += Serial.write((const uint8_t *)capPend + capPendPos, n);
}

void printFrame(const can_frame &f) {
  const bool ext = f.can_id & CAN_EFF_FLAG;
  const bool rtr = f.can_id & CAN_RTR_FLAG;
  char text[96];
  int n = snprintf(text, sizeof(text), " ID=0x%08lX %s %s DLC=%u DATA=",
      (unsigned long)(f.can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK)),
      ext ? "EXT" : "STD", rtr ? "RTR" : "DATA", (unsigned)f.can_dlc);
  if (n < 0) return;
  if (rtr) {
    n += snprintf(text + n, sizeof(text) - (size_t)n, "<none>");
  } else {
    for (unsigned i = 0; i < f.can_dlc && i < 8 && (size_t)n < sizeof(text) - 4; ++i)
      n += snprintf(text + n, sizeof(text) - (size_t)n, "%02X%s",
          (unsigned)f.data[i], i + 1 == f.can_dlc ? "" : " ");
  }
  if ((size_t)n >= sizeof(text)) n = (int)sizeof(text) - 1;
  outWrite(text, (size_t)n);
}

void decodeAlerts(uint32_t flags) {
  outf("TWAI ALERT FLAGS 0x%08lX:", (unsigned long)flags);
  if (!flags) outf(" NONE");
  if (flags & TWAI_ALERT_TX_SUCCESS) outf(" TX_SUCCESS");
  if (flags & TWAI_ALERT_TX_FAILED) outf(" TX_FAILED");
  if (flags & TWAI_ALERT_BUS_ERROR) outf(" BUS_ERROR");
  if (flags & TWAI_ALERT_BUS_OFF) outf(" BUS_OFF");
  if (flags & TWAI_ALERT_ERR_PASS) outf(" ERROR_PASSIVE");
  if (flags & TWAI_ALERT_RX_QUEUE_FULL) outf(" RX_QUEUE_FULL");
  outf("\n");
}

// MCP2515 BIT MODIFY instruction (0x05). Same thread as the library, so no
// concurrent SPI use.
void mcpBitModify(uint8_t address, uint8_t mask, uint8_t value) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(MCP2515_CS, LOW);
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(MCP2515_CS, HIGH);
  SPI.endTransaction();
}

// MCP2515 READ instruction (0x03), one register. Same thread as the library.
uint8_t mcpRead(uint8_t address) {
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(MCP2515_CS, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  const uint8_t value = SPI.transfer(0x00);
  digitalWrite(MCP2515_CS, HIGH);
  SPI.endTransaction();
  return value;
}

void captureSnapshot() {
  faultSnapshot = FaultSnapshot();
  faultSnapshot.captured = true;
  faultSnapshot.beginUs = micros();
  faultSnapshot.hadA = inFlightA;
  faultSnapshot.hadB = inFlightB;
  if (twaiInstalled) {
    faultSnapshot.statusResult = twai_get_status_info(&faultSnapshot.twai);
    // One instantaneous MCU-side RX sample, NOT a bus-voltage measurement.
    faultSnapshot.rxLogic = digitalRead(CAN_RX);
  }
  if (spiReady) {
    faultSnapshot.haveMcp = true;
    faultSnapshot.mcpIntLogic = digitalRead(MCP2515_INT);
    // MCP2515 READ instruction 0x03, sequential address read 0x00..0x7F.
    SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(MCP2515_CS, LOW);
    SPI.transfer(0x03);
    SPI.transfer(0x00);
    for (unsigned i = 0; i < sizeof(faultSnapshot.mcp); ++i)
      faultSnapshot.mcp[i] = SPI.transfer(0x00);
    digitalWrite(MCP2515_CS, HIGH);
    SPI.endTransaction();
    lastEflg = faultSnapshot.mcp[0x2D];
  }
  faultSnapshot.endUs = micros();
}

void printStatus() {
  const char *name = state == BridgeState::RUNNING ? "RUNNING" : "WAITING";
  outf("%s | %s\n", name, lastReason);
  outf("A RX=%lu -> B QUEUED=%lu DONE=%lu pending=%u active=%u\n",
      (unsigned long)rxA, (unsigned long)queuedB, (unsigned long)doneB,
      (unsigned)toB.count, (unsigned)inFlightB);
  outf("B RX=%lu -> A QUEUED=%lu DONE=%lu pending=%u active=%u\n",
      (unsigned long)rxB, (unsigned long)queuedA, (unsigned long)doneA,
      (unsigned)toA.count, (unsigned)inFlightA);
  outf("Controller loss: A overflow events=%lu, B lost frames=%lu; dropped bad=%lu queue-full=%lu; A arbitration retries=%lu\n",
      (unsigned long)overflowA, (unsigned long)lostB,
      (unsigned long)droppedBad, (unsigned long)droppedFull,
      (unsigned long)arbLossA);
  outf("Starts=%lu faults=%lu; last MCP EFLG=0x%02X TWAI alerts=0x%08lX\n",
      (unsigned long)startCount, (unsigned long)faultCount,
      (unsigned)lastEflg, (unsigned long)lastAlerts);
  outf("Last fault: %s\n", lastFault);
  outf("Capture: %s, %lu records; sent=%lu queued=%lu lost(backlog full)=%lu skipped(no host / c)=%lu next seq=%lu\n",
      !CAPTURE_ENABLED ? "off" : (capInPsram ? "PSRAM" : "internal RAM"),
      (unsigned long)capCapacity, (unsigned long)capSent,
      (unsigned long)capCount, (unsigned long)capLost,
      (unsigned long)capSkipped, (unsigned long)capSeq);
}

// Renders into the current sink (see sinkBegin). Never touches USB itself.
void dumpDiagnostics() {
  outf("\n========== T2CAN AUTO v6 REPORT BEGIN ==========\n");
  printStatus();
  outf("Config: A=MCP2515 500k/16MHz; B=TWAI 500k NORMAL; SPI=%lu\n",
      (unsigned long)SPI_HZ);
  outf("GPIO: TX=%d RX=%d CS=%d SCLK=%d MISO=%d MOSI=%d RST=%d INT=%d\n",
      (int)CAN_TX, (int)CAN_RX, (int)MCP2515_CS, (int)MCP2515_SCLK,
      (int)MCP2515_MISO, (int)MCP2515_MOSI, (int)MCP2515_RST,
      (int)MCP2515_INT);
  outf("Queue high-water A=%u B=%u; max RUN loop gap=%lu us\n",
      (unsigned)highWaterA, (unsigned)highWaterB, (unsigned long)maxLoopGapUs);
  decodeAlerts(lastAlerts);
  outf("All alerts seen since boot: ");
  decodeAlerts(allAlertsSeen);
  if (faultSnapshot.captured) {
    outf("Snapshot begin=%lu us end=%lu us; outstanding BEFORE stop A=%u B=%u\n",
        (unsigned long)faultSnapshot.beginUs, (unsigned long)faultSnapshot.endUs,
        (unsigned)faultSnapshot.hadA, (unsigned)faultSnapshot.hadB);
    outf("TWAI status API result=%d; stop API result=%d\n",
        (int)faultSnapshot.statusResult, (int)faultSnapshot.stopResult);
    if (faultSnapshot.statusResult == ESP_OK) {
      const twai_status_info_t &t = faultSnapshot.twai;
      outf("B BEFORE STOP: state=%d TEC=%lu REC=%lu bus_errors=%lu arb_lost=%lu\n",
          (int)t.state, (unsigned long)t.tx_error_counter,
          (unsigned long)t.rx_error_counter, (unsigned long)t.bus_error_count,
          (unsigned long)t.arb_lost_count);
      outf("B buffers: tx=%lu rx=%lu; failed=%lu missed=%lu overrun=%lu\n",
          (unsigned long)t.msgs_to_tx, (unsigned long)t.msgs_to_rx,
          (unsigned long)t.tx_failed_count, (unsigned long)t.rx_missed_count,
          (unsigned long)t.rx_overrun_count);
      outf("B MCU RX instantaneous logic=%d (NOT CAN-H/L voltage)\n",
          faultSnapshot.rxLogic);
    }
    if (faultSnapshot.haveMcp) {
      const uint8_t *r = faultSnapshot.mcp;
      outf("A BEFORE RESET: CANSTAT=%02X CANCTRL=%02X TEC=%u REC=%u\n",
          (unsigned)r[0x0E], (unsigned)r[0x0F], (unsigned)r[0x1C],
          (unsigned)r[0x1D]);
      outf("A timing CNF1=%02X CNF2=%02X CNF3=%02X; CANINTE=%02X CANINTF=%02X EFLG=%02X\n",
          (unsigned)r[0x2A], (unsigned)r[0x29], (unsigned)r[0x28],
          (unsigned)r[0x2B], (unsigned)r[0x2C], (unsigned)r[0x2D]);
      outf("A TXB0CTRL=%02X TXB1CTRL=%02X TXB2CTRL=%02X RXB0CTRL=%02X RXB1CTRL=%02X INT=%d\n",
          (unsigned)r[0x30], (unsigned)r[0x40], (unsigned)r[0x50],
          (unsigned)r[0x60], (unsigned)r[0x70], faultSnapshot.mcpIntLogic);
      outf("A raw registers, captured before reset (hex address: bytes):\n");
      for (unsigned i = 0; i < 128; ++i) {
        if ((i % 16) == 0) outf("%02X:", i);
        outf(" %02X", (unsigned)r[i]);
        if ((i % 16) == 15) outf("\n");
      }
    }
  }
  outf("Pending software queues, oldest first (max 16 shown each):\n");
  for (unsigned direction = 0; direction < 2; ++direction) {
    FrameQueue &q = direction == 0 ? toA : toB;
    for (size_t i = 0; i < q.count && i < 16; ++i) {
      QueuedFrame &e = q.entries[(q.head + i) % QUEUE_SIZE];
      outf("TO_%c[%u] rx_ms=%lu", direction == 0 ? 'A' : 'B',
          (unsigned)i, (unsigned long)e.receivedAt);
      printFrame(e.frame);
      outf("\n");
    }
  }
  outf("TRACE events=%u overwritten=%lu (timestamps are micros(), wrap ~71.6min)\n",
      (unsigned)traceUsed, (unsigned long)traceOverwritten);
  const size_t first = (traceNext + TRACE_CAPACITY - traceUsed) % TRACE_CAPACITY;
  for (size_t i = 0; i < traceUsed; ++i) {
    const TraceEvent &e = traceBuffer[(first + i) % TRACE_CAPACITY];
    outf("t=%lu us %-22s result=%ld QA=%u QB=%u",
        (unsigned long)e.timeUs, e.name, (long)e.result,
        (unsigned)e.depthA, (unsigned)e.depthB);
    if (e.hasFrame) printFrame(e.frame);
    outf("\n");
  }
  outf("BUS_ERROR alone cannot distinguish ACK vs bit/stuff/CRC/form.\n");
  outf("DONE means controller completion, not ECU application acceptance.\n");
  outf("========== T2CAN AUTO v6 REPORT END ==========\n\n");
}

bool validFrame(const can_frame &f) {
  if (f.can_dlc > 8 || (f.can_id & CAN_ERR_FLAG)) return false;
  const uint32_t mask = (f.can_id & CAN_EFF_FLAG) ?
                        CAN_EFF_MASK : CAN_SFF_MASK;
  return (f.can_id & ~(mask | CAN_EFF_FLAG | CAN_RTR_FLAG)) == 0;
}

twai_message_t toTwai(const can_frame &f) {
  twai_message_t m = {};
  m.extd = (f.can_id & CAN_EFF_FLAG) != 0;
  m.rtr = (f.can_id & CAN_RTR_FLAG) != 0;
  m.identifier = (uint32_t)(f.can_id & (m.extd ? CAN_EFF_MASK : CAN_SFF_MASK));
  m.data_length_code = f.can_dlc;
  if (!m.rtr) memcpy(m.data, f.data, f.can_dlc);
  return m;
}

can_frame fromTwai(const twai_message_t &m) {
  can_frame f = {};
  f.can_id = m.identifier;
  if (m.extd) f.can_id |= CAN_EFF_FLAG;
  if (m.rtr) f.can_id |= CAN_RTR_FLAG;
  f.can_dlc = m.data_length_code;
  if (!m.rtr) memcpy(f.data, m.data, m.data_length_code);
  return f;
}

// Keep the TWAI transmit pin recessive (high) whenever the TWAI driver is
// not installed, so a restart can never leave segment B held dominant.
// twai_driver_install() takes the pin over again by itself.
void parkCanTx() {
  pinMode(CAN_TX, OUTPUT);
  digitalWrite(CAN_TX, HIGH);
}

// Silence both controllers. MCP2515 is held in hardware reset (its TXCAN
// output is recessive); TWAI is stopped and uninstalled so that the next
// start can install it again cleanly.
void quiesceHardware() {
  digitalWrite(MCP2515_RST, LOW);
  if (twaiInstalled) {
    twai_status_info_t s = {};
    const bool haveStatus = twai_get_status_info(&s) == ESP_OK;
    if (haveStatus && s.state == TWAI_STATE_RUNNING)
      faultSnapshot.stopResult = twai_stop();
    if (twai_driver_uninstall() == ESP_OK) {
      twaiInstalled = false;
    } else {
      // e.g. still BUS_OFF: begin recovery; next restart attempt retries.
      if (haveStatus && s.state == TWAI_STATE_BUS_OFF)
        twai_initiate_recovery();
      recordEvent("B_UNINSTALL_PENDING", 0, nullptr);
    }
  }
  if (!twaiInstalled) parkCanTx();
}

void reportFault() {
  // A previous report is still being sent to a listening host: keep it
  // intact; the status line shows the new fault count.
  if (Serial && reportPos < reportLen) return;
  sinkBegin(lineBuf, sizeof(lineBuf));
  outf("\n!! FAULT #%lu: %s -> automatic restart in %lu ms\n",
      (unsigned long)faultCount, lastReason, (unsigned long)RESTART_DELAY_MS);
  lineLen = sinkEnd();
  linePos = 0;
  if (PRINT_FULL_REPORT_ON_FAULT &&
      (!everDumped || millis() - lastFullDump >= FULL_DUMP_MIN_GAP_MS)) {
    everDumped = true;
    lastFullDump = millis();
    // Rendered now (a few ms, inside the restart cool-down), sent later.
    sinkBegin(reportBuf, sizeof(reportBuf));
    dumpDiagnostics();
    reportLen = sinkEnd();
    reportPos = 0;
  }
}

// Latch-free fault handling: capture, silence, schedule an automatic restart.
void faultStop(const char *reason) {
  recordEvent("FAULT", 0, nullptr);
  captureSnapshot();
  quiesceHardware();
  state = BridgeState::WAITING;
  lastReason = reason;
  lastFault = reason;
  inFlightA = false;
  inFlightB = false;
  ++faultCount;
  restartAt = millis() + RESTART_DELAY_MS;
  reportFault();
}

bool startBridge() {
  if (state == BridgeState::RUNNING) return true;
  recordEvent("START_REQUEST", 0, nullptr);
  quiesceHardware();
  if (twaiInstalled) {
    faultStop("TWAI driver not released yet (bus-off recovery in progress)");
    return false;
  }
  toA.clear();
  toB.clear();
  inFlightA = false;
  inFlightB = false;
  baseMissed = 0;
  baseOverrun = 0;
  lastAlerts = 0;
  alertsRecorded = 0;
  lastEflg = 0;
  rxb1Older = false;
  ++startCount;

  if (!spiReady) {
    SPI.begin(MCP2515_SCLK, MCP2515_MISO, MCP2515_MOSI, MCP2515_CS);
    spiReady = true;
  }
  digitalWrite(MCP2515_RST, LOW);
  delay(5);
  digitalWrite(MCP2515_RST, HIGH);
  delay(startCount <= 1 ? 100 : 20); // oscillator settle; long only at power-up
  MCP2515::ERROR aResult = canA.reset();
  recordEvent("A_RESET", aResult, nullptr);
  if (aResult != MCP2515::ERROR_OK) {
    faultStop("MCP2515 reset failed");
    return false;
  }
  aResult = canA.setBitrate(CAN_500KBPS, OSCILLATOR);
  recordEvent("A_SET_BITRATE", aResult, nullptr);
  if (aResult != MCP2515::ERROR_OK) {
    faultStop("MCP2515 bitrate setup failed");
    return false;
  }

  // autowp reset sets zero masks and accepts standard + extended frames.
  // MCP2515 remains in configuration mode until both sides are prepared.
  canA.clearInterrupts();

  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
      (gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  g.rx_queue_len = 64;
  // One software-owned in-flight frame; no hidden TWAI transmit backlog.
  g.tx_queue_len = 0;
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
      TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL |
      TWAI_ALERT_BUS_ERROR;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t bResult = twai_driver_install(&g, &timing, &filter);
  recordEvent("B_INSTALL", bResult, nullptr);
  if (bResult != ESP_OK) {
    faultStop("TWAI install failed");
    return false;
  }
  twaiInstalled = true;
  bResult = twai_start();
  recordEvent("B_START", bResult, nullptr);
  if (bResult != ESP_OK) {
    faultStop("TWAI start failed");
    return false;
  }
  aResult = canA.setNormalMode();
  recordEvent("A_NORMAL", aResult, nullptr);
  if (aResult != MCP2515::ERROR_OK) {
    faultStop("MCP2515 normal mode failed");
    return false;
  }
  lastReason = "Forwarding classic CAN, 500 kbit/s, both directions";
  state = BridgeState::RUNNING;
  lastLoopUs = micros();
  lastHealthCheck = millis();
  lastReport = millis();
  recordEvent("RUNNING", 0, nullptr);
  return true;
}

bool checkHealth() {
  lastEflg = canA.getErrorFlags();
  if (lastEflg & 0x20) { // TXBO: MCP2515 bus-off
    faultStop("A bus-off (MCP2515)");
    return false;
  }
  if (lastEflg & 0xC0) { // RX1OVR / RX0OVR: a frame was lost; count, clear
    ++overflowA;
    recordEvent("A_RX_OVERFLOW", lastEflg, nullptr);
    mcpBitModify(0x2D, 0xC0, 0x00);
  }
  twai_status_info_t s = {};
  if (twai_get_status_info(&s) != ESP_OK) {
    faultStop("TWAI status read failed");
    return false;
  }
  if (s.state != TWAI_STATE_RUNNING) {
    faultStop("B not running (bus-off / recovering / stopped)");
    return false;
  }
  if (s.tx_failed_count) {
    faultStop("B transmit failed");
    return false;
  }
  const uint32_t lost = (s.rx_missed_count - baseMissed) +
                        (s.rx_overrun_count - baseOverrun);
  if (lost) {
    lostB += lost;
    baseMissed = s.rx_missed_count;
    baseOverrun = s.rx_overrun_count;
    recordEvent("B_RX_LOST", (int32_t)lost, nullptr);
  }
  return true;
}

void collectCompletions() {
  // Only TXB0 is used, so queued frames cannot overtake each other across
  // MCP2515's three transmit buffers. Hardware RX ordering is separate.
  if (inFlightA && (canA.getInterrupts() & MCP2515::CANINTF_TX0IF)) {
    // READ STATUS bit 2 is TXB0.TXREQ. Completion must have cleared it.
    if (canA.getStatus() & 0x04) {
      faultStop("Inconsistent MCP2515 transmit status");
      return;
    }
    canA.clearTXInterrupts();
    inFlightA = false;
    ++doneA;
    recordEvent("A_TX_COMPLETE", 0, nullptr);
  }

  uint32_t alerts = 0;
  const esp_err_t err = twai_read_alerts(&alerts, 0);
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT) {
    faultStop("TWAI alert read failed");
    return;
  }
  if (err == ESP_OK) {
    lastAlerts = alerts;
    allAlertsSeen |= alerts;
    // Trace only new alert kinds, or fatal ones, so a bus-error storm cannot
    // flood the bounded trace.
    if ((alerts & ~alertsRecorded) ||
        (alerts & (TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED)))
      recordEvent("B_ALERT", (int32_t)alerts, nullptr);
    alertsRecorded |= alerts;
    if (alerts & TWAI_ALERT_BUS_OFF) {
      faultStop("B BUS_OFF");
      return;
    }
    if (alerts & TWAI_ALERT_TX_FAILED) {
      faultStop("B TX_FAILED");
      return;
    }
    // BUS_ERROR / ERROR_PASSIVE / RX_QUEUE_FULL are non-fatal: visible in
    // the status counters; a stuck transmit is caught by the timeout below.
    if (alerts & TWAI_ALERT_TX_SUCCESS) {
      if (!inFlightB) {
        faultStop("Unexpected TWAI completion");
        return;
      }
      inFlightB = false;
      ++doneB;
      recordEvent("B_TX_COMPLETE", 0, nullptr);
    }
  }
  const uint32_t now = millis();
  if (inFlightA && now - flightAgeA >= MAX_TX_MS) {
    faultStop("A transmit not completed in time (no ACK / peer absent / bitrate)");
    return;
  }
  if (inFlightB && now - flightAgeB >= MAX_TX_MS) {
    faultStop("B transmit not completed in time (no ACK / peer absent / bitrate)");
    return;
  }
}

void collectFrames() {
  // Small bounded batches service both directions fairly.
  // MCP2515 has two receive buffers. A frame goes to RXB0 if it is free,
  // otherwise (rollover) to RXB1. When both hold a frame, RXB1 is the older
  // one if it was filled while RXB0 was busy and RXB0 has been read since.
  // Reading RXB0 first in that case would reorder frames, so track it.
  uint8_t status = canA.getStatus();
  for (unsigned i = 0; i < 2; ++i) {
    const bool has0 = (status & 0x01) != 0;
    const bool has1 = (status & 0x02) != 0;
    if (!has0 && !has1) break;
    const MCP2515::RXBn buffer = (has0 && has1) ? (rxb1Older ? MCP2515::RXB1 : MCP2515::RXB0)
                                                : (has0 ? MCP2515::RXB0 : MCP2515::RXB1);
    can_frame f = {};
    const MCP2515::ERROR e = canA.readMessage(buffer, &f);
    recordEvent("A_READ", e, e == MCP2515::ERROR_OK ? &f : nullptr);
    if (e != MCP2515::ERROR_OK) {
      // The library leaves the RX buffer flagged on a bad frame; only a
      // controller restart clears that reliably.
      faultStop("MCP2515 receive failed (SPI error or DLC>8)");
      return;
    }
    // Status AFTER the read: if RXB1 is filled now, it is older than any
    // frame that will land in the (just freed) RXB0.
    status = canA.getStatus();
    rxb1Older = (buffer == MCP2515::RXB0) && (status & 0x02);
    ++rxA;
    captureFrame(false, f.can_id & CAN_EFF_MASK, (f.can_id & CAN_EFF_FLAG) != 0,
                 (f.can_id & CAN_RTR_FLAG) != 0, f.can_dlc, f.data);
    if (!validFrame(f)) {
      ++droppedBad;
      recordEvent("A_DROP_BAD", 0, &f);
      continue;
    }
    if (!toB.push(f, millis())) {
      ++droppedFull;
      recordEvent("A_DROP_QFULL", 0, &f);
      continue;
    }
    recordEvent("ENQUEUE_TO_B", 0, &f);
  }
  for (unsigned i = 0; i < 2; ++i) {
    twai_message_t m = {};
    const esp_err_t e = twai_receive(&m, 0);
    if (e == ESP_ERR_TIMEOUT) break;
    recordEvent("B_READ_RESULT", e, nullptr);
    if (e != ESP_OK) {
      faultStop("TWAI receive failed");
      return;
    }
    ++rxB;
    captureFrame(true, m.identifier, m.extd, m.rtr, m.data_length_code, m.data);
    if (m.data_length_code > 8 ||
        m.identifier > (m.extd ? CAN_EFF_MASK : CAN_SFF_MASK)) {
      ++droppedBad;
      recordEvent("B_DROP_BAD", 0, nullptr);
      continue;
    }
    const can_frame f = fromTwai(m);
    recordEvent("B_RX_FRAME", 0, &f);
    if (!toA.push(f, millis())) {
      ++droppedFull;
      recordEvent("B_DROP_QFULL", 0, &f);
      continue;
    }
    recordEvent("ENQUEUE_TO_A", 0, &f);
  }
}

void submitFrames() {
  const uint32_t now = millis();
  if ((toA.count && now - toA.front().receivedAt >= MAX_QUEUE_AGE_MS) ||
      (toB.count && now - toB.front().receivedAt >= MAX_QUEUE_AGE_MS)) {
    faultStop("Queued frame exceeded age limit (egress side stuck)");
    return;
  }
  if (!inFlightA && toA.count) {
    if (canA.getStatus() & 0x04) {
      faultStop("MCP2515 TXB0 unexpectedly busy");
      return;
    }
    canA.clearTXInterrupts();
    const MCP2515::ERROR result = canA.sendMessage(MCP2515::TXB0, &toA.front().frame);
    recordEvent("A_TX_SUBMIT", result, &toA.front().frame);
    bool accepted = (result == MCP2515::ERROR_OK);
    if (!accepted && result == MCP2515::ERROR_FAILTX) {
      // autowp reads TXB0CTRL right after setting TXREQ and returns FAILTX
      // if MLOA (lost arbitration) / TXERR / ABTF is flagged. Losing
      // arbitration is NORMAL on a busy bus: the frame is still queued in
      // the chip (TXREQ=1) and is retried automatically. Only a frame
      // that is neither pending nor already completed is a real failure.
      const uint8_t ctrl = mcpRead(0x30);                       // TXB0CTRL
      const bool pending = (ctrl & 0x08) != 0;                  // TXREQ
      const bool done = (canA.getInterrupts() & MCP2515::CANINTF_TX0IF) != 0;
      recordEvent("A_TX_FAILTX_CHECK", ctrl, nullptr);
      if (pending || done) {
        accepted = true;
        ++arbLossA;
      }
    }
    if (!accepted) {
      faultStop("MCP2515 transmit submission failed (frame not pending)");
      return;
    }
    flightAgeA = toA.front().receivedAt;
    toA.pop();
    inFlightA = true;
    ++queuedA;
  }
  if (!inFlightB && toB.count) {
    const twai_message_t m = toTwai(toB.front().frame);
    const esp_err_t result = twai_transmit(&m, 0);
    recordEvent("B_TX_SUBMIT", result, &toB.front().frame);
    if (result != ESP_OK) {
      faultStop("TWAI transmit submission failed");
      return;
    }
    flightAgeB = toB.front().receivedAt;
    toB.pop();
    inFlightB = true;
    ++queuedB;
  }
}

// One short status line per second, only if the USB TX buffer has room.
void periodicReport() {
  const uint32_t now = millis();
  if (now - lastReport < STATUS_PERIOD_MS) return;
  lastReport = now;
  if (statusPos < statusLen || !Serial) return;   // previous line not out yet
  int n;
  if (state == BridgeState::RUNNING) {
    n = snprintf(statusBuf, sizeof(statusBuf),
        "#RUN A_RX=%lu B_DONE=%lu B_RX=%lu A_DONE=%lu Q=%u/%u OVR_A=%lu LOST_B=%lu DROP=%lu/%lu ARB_A=%lu FAULTS=%lu CAP=%lu CAPQ=%lu CAPLOST=%lu\n",
        (unsigned long)rxA, (unsigned long)doneB,
        (unsigned long)rxB, (unsigned long)doneA,
        (unsigned)toB.count, (unsigned)toA.count,
        (unsigned long)overflowA, (unsigned long)lostB,
        (unsigned long)droppedBad, (unsigned long)droppedFull,
        (unsigned long)arbLossA, (unsigned long)faultCount,
        (unsigned long)capSent, (unsigned long)capCount,
        (unsigned long)capLost);
  } else {
    n = snprintf(statusBuf, sizeof(statusBuf), "#WAIT restart in %ld ms | %s | faults=%lu\n",
        (long)(int32_t)(restartAt - now), lastReason,
        (unsigned long)faultCount);
  }
  if (n <= 0) return;
  if (n >= (int)sizeof(statusBuf)) n = (int)sizeof(statusBuf) - 1;
  statusLen = (size_t)n;
  statusPos = 0;
}

// Optional diagnostics only. No command is needed for operation, and no
// command ever stops forwarding.
void handleSerial() {
  if (!Serial.available()) return;
  const int command = Serial.read();
  if (command == 'c' || command == 'C') {   // start a fresh capture
    capRestartRequested = true;
    return;
  }
  if (command != 'd' && command != 'D' && command != '?') return;
  if (!serialIdle()) return;   // still sending the previous output
  sinkBegin(lineBuf, sizeof(lineBuf));
  outf("\n---- status (forwarding continues) ----\n");
  printStatus();
  outf(reportLen ? "---- last fault report follows ----\n"
                 : "---- no fault recorded since boot ----\n");
  lineLen = sinkEnd();
  linePos = 0;
  reportPos = 0;   // send the stored fault report again (nothing if none)
}

void setup() {
  pinMode(MCP2515_INT, INPUT_PULLUP);
  pinMode(MCP2515_RST, OUTPUT);
  digitalWrite(MCP2515_RST, LOW); // MCP2515 held in reset until auto-start
  parkCanTx();                    // B side stays recessive until TWAI starts
  Serial.begin(115200);
  captureInit();
  // No dependency on a USB host connecting at boot.
  recordEvent("BOOT", 0, nullptr);
  state = BridgeState::WAITING;
  restartAt = millis() + START_DELAY_MS;
  lastReport = millis();
}

void loop() {
  // One serial byte in, one short chunk out per pass. Serial can never
  // starve CAN.
  handleSerial();
  pumpSerial();

  if (state != BridgeState::RUNNING) {
    // Waiting for auto-start (boot) or for the restart cool-down.
    if ((int32_t)(millis() - restartAt) >= 0) {
      startBridge();
      return;
    }
    periodicReport();
    delay(1);
    return;
  }

  const uint32_t loopUs = micros();
  const uint32_t gapUs = loopUs - lastLoopUs;
  if (gapUs > maxLoopGapUs) maxLoopGapUs = gapUs;
  lastLoopUs = loopUs;
  collectCompletions();
  if (state != BridgeState::RUNNING) return;
  collectFrames();
  if (state != BridgeState::RUNNING) return;
  submitFrames();
  if (state != BridgeState::RUNNING) return;
  if (millis() - lastHealthCheck >= 1) {
    lastHealthCheck = millis();
    if (!checkHealth()) return;
  }
  periodicReport();
  // Yield to same-priority work; this is not a throughput guarantee.
  yield();
}
