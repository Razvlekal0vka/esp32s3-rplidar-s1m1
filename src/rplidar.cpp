#include "rplidar.h"

namespace {
// Синхробайты команд и дескриптора ответа / command + answer descriptor sync.
constexpr uint8_t kCmdSync = 0xA5;
constexpr uint8_t kAnsSync1 = 0xA5;
constexpr uint8_t kAnsSync2 = 0x5A;

constexpr uint8_t kCmdStop = 0x25;
constexpr uint8_t kCmdReset = 0x40;
constexpr uint8_t kCmdScan = 0x20;
constexpr uint8_t kCmdExpressScan = 0x82;
constexpr uint8_t kCmdGetInfo = 0x50;
constexpr uint8_t kCmdGetHealth = 0x52;
constexpr uint8_t kCmdGetLidarConf = 0x84;
constexpr uint8_t kCmdMotorSpeed = 0xA8;

constexpr uint8_t kAnsInfo = 0x04;
constexpr uint8_t kAnsHealth = 0x06;
constexpr uint8_t kAnsConf = 0x20;
constexpr uint8_t kAnsScan = 0x81;
constexpr uint8_t kAnsDense = 0x85;

// GET_LIDAR_CONF type ids (SLAMTEC).
constexpr uint32_t kConfTypical = 0x7C;
constexpr uint32_t kConfAnsType = 0x75;
constexpr uint32_t kConfName = 0x7F;

uint8_t xorBytes(const uint8_t* data, size_t n) {
  uint8_t cs = 0;
  for (size_t i = 0; i < n; ++i) {
    cs ^= data[i];
  }
  return cs;
}

float wrapDeg(float deg) {
  while (deg < 0) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}
}  // namespace

RpLidar::RpLidar(HardwareSerial& serial, int8_t rxPin, int8_t txPin)
    : uart_(serial), rxPin_(rxPin), txPin_(txPin) {}

bool RpLidar::begin() { return begin(rxPin_, txPin_, kBaud); }

bool RpLidar::begin(int8_t rxPin, int8_t txPin, uint32_t baud) {
  rxPin_ = rxPin;
  txPin_ = txPin;
  uart_.end();
  delay(10);
  uart_.setRxBufferSize(4096);  // DenseBoost ~9 кГц, нужен запас / DenseBoost burst
  uart_.begin(baud, SERIAL_8N1, rxPin_, txPin_);
  delay(50);
  flushInput(20);
  return true;
}

size_t RpLidar::dumpRx(Stream& out, uint32_t waitMs) {
  const uint32_t start = millis();
  size_t n = 0;
  while (millis() - start < waitMs) {
    while (uart_.available()) {
      const uint8_t b = static_cast<uint8_t>(uart_.read());
      out.printf("%02X ", b);
      if (b >= 32 && b < 127) {
        out.printf("(%c) ", b);
      }
      ++n;
      if (n % 16 == 0) {
        out.println();
      }
    }
    delay(1);
  }
  if (n) {
    out.println();
  }
  out.printf("raw bytes: %u\n", static_cast<unsigned>(n));
  return n;
}

void RpLidar::flushInput(uint32_t waitMs) {
  const uint32_t start = millis();
  do {
    while (uart_.available()) {
      uart_.read();
    }
    if (waitMs) {
      delay(1);
    }
  } while (waitMs && (millis() - start) < waitMs);
}

void RpLidar::sendSimple(uint8_t cmd) {
  uint8_t pkt[2] = {kCmdSync, cmd};
  uart_.write(pkt, sizeof(pkt));
}

// Команда с payload: A5 cmd size payload XOR(A5..payload).
// Payload command: A5 cmd size payload XOR(A5..payload).
void RpLidar::sendPayload(uint8_t cmd, const uint8_t* payload, uint8_t size) {
  uint8_t pkt[16];
  pkt[0] = kCmdSync;
  pkt[1] = cmd;
  pkt[2] = size;
  memcpy(pkt + 3, payload, size);
  pkt[3 + size] = xorBytes(pkt, 3 + size);
  uart_.write(pkt, 4 + size);
}

// Дескриптор: A5 5A | len30 + sendMode2 | type. Ищем A5 5A в потоке.
// Descriptor: A5 5A | 30-bit length + 2-bit sendMode | type. Hunt A5 5A.
bool RpLidar::waitDescriptor(uint32_t& length, uint8_t& sendMode, uint8_t& type,
                             uint32_t timeoutMs) {
  const uint32_t start = millis();
  int state = 0;
  while (millis() - start < timeoutMs) {
    if (!uart_.available()) {
      delay(1);
      continue;
    }
    const uint8_t b = static_cast<uint8_t>(uart_.read());
    if (state == 0) {
      state = (b == kAnsSync1) ? 1 : 0;
    } else if (b == kAnsSync2) {
      uint8_t rest[5];
      if (!readExact(rest, 5, timeoutMs)) {
        return false;
      }
      const uint32_t raw = static_cast<uint32_t>(rest[0]) |
                           (static_cast<uint32_t>(rest[1]) << 8) |
                           (static_cast<uint32_t>(rest[2]) << 16) |
                           (static_cast<uint32_t>(rest[3]) << 24);
      length = raw & 0x3FFFFFFFu;
      sendMode = static_cast<uint8_t>((raw >> 30) & 0x3u);
      type = rest[4];
      return true;
    } else {
      state = (b == kAnsSync1) ? 1 : 0;
    }
  }
  return false;
}

bool RpLidar::readExact(uint8_t* buf, size_t n, uint32_t timeoutMs) {
  const uint32_t start = millis();
  size_t got = 0;
  while (got < n) {
    if (millis() - start >= timeoutMs) {
      return false;
    }
    const int avail = uart_.available();
    if (avail <= 0) {
      delay(1);
      continue;
    }
    const size_t chunk = min(static_cast<size_t>(avail), n - got);
    got += uart_.readBytes(buf + got, chunk);
  }
  return true;
}

void RpLidar::stop() {
  sendSimple(kCmdStop);
  delay(20);
  scanFormat_ = RpScanFormat::None;
  stdCount_ = 0;
  denseCount_ = 0;
  hasPrevDense_ = false;
  flushInput(5);
}

void RpLidar::reset() {
  sendSimple(kCmdReset);
  scanFormat_ = RpScanFormat::None;
  stdCount_ = 0;
  denseCount_ = 0;
  hasPrevDense_ = false;
  delay(600);  // баннер после ребута лидара / wait for lidar reboot banner
}

bool RpLidar::getInfo(RpDeviceInfo& info, uint32_t timeoutMs) {
  stop();
  sendSimple(kCmdGetInfo);
  uint32_t length = 0;
  uint8_t sendMode = 0;
  uint8_t type = 0;
  if (!waitDescriptor(length, sendMode, type, timeoutMs) || type != kAnsInfo ||
      length != 20) {
    return false;
  }
  uint8_t raw[20];
  if (!readExact(raw, 20, timeoutMs)) {
    return false;
  }
  info.model = raw[0];
  info.firmwareMinor = raw[1];
  info.firmwareMajor = raw[2];
  info.hardware = raw[3];
  memcpy(info.serial, raw + 4, 16);
  return true;
}

bool RpLidar::getHealth(RpHealth& health, uint32_t timeoutMs) {
  stop();
  sendSimple(kCmdGetHealth);
  uint32_t length = 0;
  uint8_t sendMode = 0;
  uint8_t type = 0;
  if (!waitDescriptor(length, sendMode, type, timeoutMs) || type != kAnsHealth ||
      length != 3) {
    return false;
  }
  uint8_t raw[3];
  if (!readExact(raw, 3, timeoutMs)) {
    return false;
  }
  health.status = raw[0];
  health.errorCode = static_cast<uint16_t>(raw[1] | (raw[2] << 8));
  return true;
}

bool RpLidar::getLidarConf(uint32_t confType, const uint8_t* extra,
                           uint8_t extraLen, uint8_t* out, size_t outMax,
                           size_t& outLen, uint32_t timeoutMs) {
  uint8_t payload[8];
  payload[0] = static_cast<uint8_t>(confType);
  payload[1] = static_cast<uint8_t>(confType >> 8);
  payload[2] = static_cast<uint8_t>(confType >> 16);
  payload[3] = static_cast<uint8_t>(confType >> 24);
  if (extraLen) {
    memcpy(payload + 4, extra, extraLen);
  }
  sendPayload(kCmdGetLidarConf, payload, 4 + extraLen);

  uint32_t length = 0;
  uint8_t sendMode = 0;
  uint8_t type = 0;
  if (!waitDescriptor(length, sendMode, type, timeoutMs) || type != kAnsConf ||
      length < 4 || length > 80) {
    return false;
  }
  uint8_t raw[80];
  if (!readExact(raw, length, timeoutMs)) {
    return false;
  }
  const size_t payloadBytes = length - 4;
  outLen = min(payloadBytes, outMax);
  memcpy(out, raw + 4, outLen);  // первые 4 байта — echo type / first 4 B echo type
  return true;
}

bool RpLidar::getTypicalScanMode(uint16_t& modeId, uint32_t timeoutMs) {
  uint8_t out[4];
  size_t outLen = 0;
  if (!getLidarConf(kConfTypical, nullptr, 0, out, sizeof(out), outLen,
                    timeoutMs) ||
      outLen < 2) {
    return false;
  }
  modeId = static_cast<uint16_t>(out[0] | (out[1] << 8));
  return true;
}

bool RpLidar::getScanModeName(uint16_t modeId, char* name, size_t nameMax,
                              uint32_t timeoutMs) {
  const uint8_t extra[2] = {static_cast<uint8_t>(modeId),
                            static_cast<uint8_t>(modeId >> 8)};
  uint8_t out[48];
  size_t outLen = 0;
  if (!getLidarConf(kConfName, extra, 2, out, sizeof(out), outLen, timeoutMs)) {
    return false;
  }
  const size_t n = min(outLen, nameMax - 1);
  memcpy(name, out, n);
  name[n] = 0;
  return true;
}

bool RpLidar::getScanModeAnsType(uint16_t modeId, uint8_t& ansType,
                                 uint32_t timeoutMs) {
  const uint8_t extra[2] = {static_cast<uint8_t>(modeId),
                            static_cast<uint8_t>(modeId >> 8)};
  uint8_t out[4];
  size_t outLen = 0;
  if (!getLidarConf(kConfAnsType, extra, 2, out, sizeof(out), outLen,
                    timeoutMs) ||
      outLen < 1) {
    return false;
  }
  ansType = out[0];
  return true;
}

bool RpLidar::setMotorRpm(uint16_t rpm) {
  const uint8_t payload[2] = {static_cast<uint8_t>(rpm),
                              static_cast<uint8_t>(rpm >> 8)};
  sendPayload(kCmdMotorSpeed, payload, 2);
  delay(10);
  return true;
}

bool RpLidar::startStandardScan() {
  stop();
  sendSimple(kCmdScan);
  uint32_t length = 0;
  uint8_t sendMode = 0;
  uint8_t type = 0;
  if (!waitDescriptor(length, sendMode, type, 1000) || type != kAnsScan ||
      length != 5) {
    return false;
  }
  scanFormat_ = RpScanFormat::Standard;
  stdCount_ = 0;
  pointsThisRev_ = 0;
  lastAngleDeg_ = -1.0f;
  return true;
}

bool RpLidar::startExpressScan(uint8_t workingMode) {
  stop();
  const uint8_t payload[5] = {workingMode, 0, 0, 0, 0};
  sendPayload(kCmdExpressScan, payload, 5);
  uint32_t length = 0;
  uint8_t sendMode = 0;
  uint8_t type = 0;
  if (!waitDescriptor(length, sendMode, type, 1500)) {
    return false;
  }
  // S1 typical — только Dense 0x85 / 84 байта. Legacy 0x82 здесь не принимаем.
  // S1 typical is Dense 0x85 / 84 bytes only. Legacy 0x82 is not accepted here.
  if (type != kAnsDense || length != 84) {
    scanFormat_ = RpScanFormat::None;
    return false;
  }
  scanFormat_ = RpScanFormat::Dense;
  denseCount_ = 0;
  hasPrevDense_ = false;
  pointsThisRev_ = 0;
  rxBytes_ = 0;
  denseOk_ = 0;
  denseFail_ = 0;
  lastAngleDeg_ = -1.0f;
  return true;
}

bool RpLidar::startTypicalScan(uint16_t* usedMode) {
  stop();
  uint16_t modeId = 0;
  uint8_t ansType = 0;
  if (getTypicalScanMode(modeId) && getScanModeAnsType(modeId, ansType)) {
    if (usedMode) {
      *usedMode = modeId;
    }
    if (ansType == kAnsScan) {
      return startStandardScan();
    }
    if (startExpressScan(static_cast<uint8_t>(modeId))) {
      return true;
    }
  }
  if (usedMode) {
    *usedMode = 0;
  }
  return startStandardScan();
}

void RpLidar::poll() {
  if (scanFormat_ == RpScanFormat::None) {
    return;
  }
  while (uart_.available()) {
    const uint8_t b = static_cast<uint8_t>(uart_.read());
    ++rxBytes_;
    if (scanFormat_ == RpScanFormat::Standard) {
      handleStandardByte(b);
    } else if (scanFormat_ == RpScanFormat::Dense) {
      handleDenseByte(b);
    }
    // Legacy/ultra capsule пока не разбираем: для S1 typical — dense 0x85.
    // Legacy/ultra capsules are not parsed: S1 typical is dense 0x85.
  }
}

void RpLidar::handleStandardByte(uint8_t b) {
  if (stdCount_ < 5) {
    stdBuf_[stdCount_++] = b;
  } else {
    memmove(stdBuf_, stdBuf_ + 1, 4);
    stdBuf_[4] = b;
  }
  if (stdCount_ < 5) {
    return;
  }

  // Байт0: S | ~S | quality6. Байт1: C | angleQ6[6:0]. Угол Q6, дистанция Q2.
  // Byte0: S | ~S | quality6. Byte1: C | angleQ6[6:0]. Angle Q6, distance Q2.
  const bool s = stdBuf_[0] & 0x01;
  const bool notS = (stdBuf_[0] >> 1) & 0x01;
  const bool check = stdBuf_[1] & 0x01;
  if (!check || s == notS) {
    return;
  }

  const uint8_t quality = stdBuf_[0] >> 2;
  const uint16_t angleQ6 =
      static_cast<uint16_t>(((stdBuf_[2] << 8) | stdBuf_[1]) >> 1);
  const uint16_t distQ2 =
      static_cast<uint16_t>(stdBuf_[3] | (stdBuf_[4] << 8));
  const float distMm = distQ2 / 4.0f;
  const float angleDeg = angleQ6 / 64.0f;
  emitPoint(angleDeg, distMm, quality, s);
  stdCount_ = 0;
}

bool RpLidar::denseChecksumOk(const uint8_t* pkt) const {
  // Заголовок A? 5?: нибблы checksum = XOR байт [2..83].
  // Header A? 5?: nibble checksum = XOR of bytes [2..83].
  if ((pkt[0] & 0xF0) != 0xA0 || (pkt[1] & 0xF0) != 0x50) {
    return false;
  }
  const uint8_t expected =
      static_cast<uint8_t>(((pkt[1] & 0x0F) << 4) | (pkt[0] & 0x0F));
  return xorBytes(pkt + 2, 82) == expected;
}

void RpLidar::handleDenseByte(uint8_t b) {
  if (denseCount_ == 0) {
    if ((b & 0xF0) != 0xA0) {
      return;
    }
    denseBuf_[denseCount_++] = b;
    return;
  }
  if (denseCount_ == 1) {
    if ((b & 0xF0) != 0x50) {
      denseCount_ = ((b & 0xF0) == 0xA0) ? 1 : 0;
      if (denseCount_ == 1) {
        denseBuf_[0] = b;
      }
      return;
    }
    denseBuf_[denseCount_++] = b;
    return;
  }

  denseBuf_[denseCount_++] = b;
  if (denseCount_ < 84) {
    return;
  }
  denseCount_ = 0;
  memcpy(lastDenseHead_, denseBuf_, sizeof(lastDenseHead_));
  if (!denseChecksumOk(denseBuf_)) {
    ++denseFail_;
    return;
  }
  ++denseOk_;

  // startQ6 в [2..3], бит 15 = S (новый оборот). Затем 40 × uint16 мм.
  // startQ6 in [2..3], bit 15 = S (new scan). Then 40 × uint16 mm.
  const uint16_t startQ6 = static_cast<uint16_t>(
      denseBuf_[2] | ((denseBuf_[3] & 0x7F) << 8));
  const bool s = denseBuf_[3] & 0x80;
  uint16_t dist[40];
  for (int i = 0; i < 40; ++i) {
    dist[i] = static_cast<uint16_t>(denseBuf_[4 + i * 2] |
                                    (denseBuf_[5 + i * 2] << 8));
  }

  if (s) {
    hasPrevDense_ = false;
  }
  // Капсулу отдаём, когда известен start следующей (линейная интерполяция угла).
  // Emit previous capsule once the next start angle is known (linear angle lerp).
  if (hasPrevDense_) {
    emitDensePacket(prevStartQ6_, startQ6, prevDist_, prevS_);
  }
  memcpy(prevDist_, dist, sizeof(dist));
  prevStartQ6_ = startQ6;
  prevS_ = s;
  hasPrevDense_ = true;
}

void RpLidar::emitDensePacket(uint16_t startQ6, uint16_t nextQ6,
                              const uint16_t* dist, bool startFlag) {
  int32_t diff = static_cast<int32_t>(nextQ6) - static_cast<int32_t>(startQ6);
  if (diff < 0) {
    diff += kAngleQ6Full;
  }
  for (int i = 0; i < 40; ++i) {
    const uint32_t angleQ6 =
        (static_cast<uint32_t>(startQ6) + (diff * i) / 40) % kAngleQ6Full;
    emitPoint(angleQ6 / 64.0f, static_cast<float>(dist[i]), 0,
              startFlag && i == 0);
  }
}

void RpLidar::emitPoint(float angleDeg, float distMm, uint8_t quality,
                        bool newScan) {
  angleDeg = wrapDeg(angleDeg);
  // DenseBoost S1 часто не ставит sync-бит каждый оборот — ловим переход через 0°.
  // DenseBoost S1 often skips the sync bit; treat a 0° wrap as a new scan.
  if (lastAngleDeg_ >= 0.0f && (lastAngleDeg_ - angleDeg) > 180.0f) {
    newScan = true;
  }
  lastAngleDeg_ = angleDeg;
  if (newScan) {
    ++revolutions_;
    pointsThisRev_ = 0;
  }
  ++pointsThisRev_;
  if (pointCb_) {
    pointCb_(angleDeg, distMm, quality, newScan);
  }
}
