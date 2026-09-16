#include <Arduino.h>
#include <esp_system.h>

#include "rplidar.h"

// Стенд: TX лидара -> GPIO16 (ESP RX), RX лидара -> GPIO15 (ESP TX). Линии скрещены.
// Bench: lidar TX -> GPIO16 (ESP RX), lidar RX -> GPIO15 (ESP TX). UART is crossed.
static constexpr int kLidarRxPin = 16;
static constexpr int kLidarTxPin = 15;

static RpLidar lidar(Serial1, kLidarRxPin, kLidarTxPin);

// Кадр оборота для ПК: magic + заголовок + 360 дистанций мм + XOR.
// ~737 байт * 10 Гц ≈ 7.4 кБ/с — укладывается в USB 115200.
// PC revolution frame: magic + header + 360 ranges in mm + XOR. Fits USB 115200.
static constexpr uint8_t kFrameMagic[4] = {0xAA, 0x55, 'L', 'D'};
static constexpr uint8_t kFrameVersion = 1;
static constexpr int kBins = 360;
static constexpr size_t kFrameLen = 4 + 2 + 10 + kBins * 2 + 1;  // 737

static bool verbose = false;
static uint32_t revPoints = 0;
static uint32_t revValid = 0;
static float revMinMm = 1e9f;
static float revMaxMm = 0;
static uint32_t revStartMs = 0;
static uint16_t bins[kBins];

// Двойной буфер: callback заполняет bins, loop шлёт pending*, без гонки с poll().
// Double buffer: callback fills bins, loop sends pending* so poll() is not stalled.
static uint16_t pendingBins[kBins];
static uint16_t pendingPts = 0;
static uint16_t pendingValid = 0;
static uint16_t pendingRpmX10 = 0;
static uint16_t pendingMinMm = 0;
static uint16_t pendingMaxMm = 0;
static volatile bool haveFrame = false;

static void putU16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}

static void resetRevStats() {
  revPoints = 0;
  revValid = 0;
  revMinMm = 1e9f;
  revMaxMm = 0;
  memset(bins, 0, sizeof(bins));
}

static void sendScanFrame() {
  uint8_t pkt[kFrameLen];
  pkt[0] = kFrameMagic[0];
  pkt[1] = kFrameMagic[1];
  pkt[2] = kFrameMagic[2];
  pkt[3] = kFrameMagic[3];
  pkt[4] = kFrameVersion;
  pkt[5] = 0;
  putU16(pkt + 6, pendingPts);
  putU16(pkt + 8, pendingValid);
  putU16(pkt + 10, pendingRpmX10);
  putU16(pkt + 12, pendingMinMm);
  putU16(pkt + 14, pendingMaxMm);
  memcpy(pkt + 16, pendingBins, sizeof(pendingBins));

  // XOR байт [4 .. len-2]; magic не входит / XOR bytes [4 .. len-2]; magic excluded.
  uint8_t cs = 0;
  for (size_t i = 4; i < kFrameLen - 1; ++i) {
    cs ^= pkt[i];
  }
  pkt[kFrameLen - 1] = cs;
  Serial.write(pkt, kFrameLen);

  Serial.printf("#rev rpm=%.1f pts=%u valid=%u min=%u max=%u\n",
                pendingRpmX10 / 10.0f, pendingPts, pendingValid, pendingMinMm,
                pendingMaxMm);
}

static void onPoint(float angleDeg, float distMm, uint8_t quality, bool newScan) {
  if (newScan) {
    if (revStartMs != 0) {
      const uint32_t dt = millis() - revStartMs;
      float rpm = dt > 0 ? 60000.0f / static_cast<float>(dt) : 0.0f;
      if (rpm > 6553.5f) {
        rpm = 6553.5f;  // uint16 rpm*10 / saturate rpm×10 into uint16
      }
      pendingRpmX10 = static_cast<uint16_t>(rpm * 10.0f + 0.5f);
      pendingPts = static_cast<uint16_t>(revPoints > 65535u ? 65535u : revPoints);
      pendingValid =
          static_cast<uint16_t>(revValid > 65535u ? 65535u : revValid);
      pendingMinMm = revValid ? static_cast<uint16_t>(revMinMm) : 0;
      pendingMaxMm = static_cast<uint16_t>(revMaxMm);
      memcpy(pendingBins, bins, sizeof(bins));
      haveFrame = true;
    }
    resetRevStats();
    revStartMs = millis();
  }

  ++revPoints;
  const bool valid = distMm > 0.5f;
  if (valid) {
    ++revValid;
    if (distMm < revMinMm) revMinMm = distMm;
    if (distMm > revMaxMm) revMaxMm = distMm;
    int bin = static_cast<int>(angleDeg);
    if (bin < 0) bin = 0;
    if (bin > kBins - 1) bin = kBins - 1;
    const uint16_t d =
        distMm > 65535.0f ? 65535u : static_cast<uint16_t>(distMm);
    // 1° бин: оставляем ближайшую (мин. мм) точку / 1° bin keeps nearest (min mm).
    if (bins[bin] == 0 || d < bins[bin]) {
      bins[bin] = d;
    }
  }

  if (verbose && valid) {
    Serial.printf("%.2f,%.0f,%u\n", angleDeg, distMm, quality);
  }
}

static const char* healthText(uint8_t status) {
  switch (status) {
    case 0:
      return "ok";
    case 1:
      return "warning";
    case 2:
      return "protection-stop";
    default:
      return "unknown";
  }
}

static void printInfo(const RpDeviceInfo& info) {
  // У S-серии major = (model & 0xF0)>>4; S1 = 6 после вычитания 5 в SDK.
  // S-series major = (model & 0xF0)>>4; S1 shows as 6 after SDK subtracts 5.
  const uint8_t major = info.model >> 4;
  const uint8_t sub = info.model & 0x0F;
  Serial.printf("model=0x%02X (major=%u sub=%u) fw=%u.%02u hw=%u\n", info.model,
                major, sub, info.firmwareMajor, info.firmwareMinor,
                info.hardware);
  Serial.print("sn=");
  for (int i = 0; i < 16; ++i) {
    Serial.printf("%02X", info.serial[i]);
  }
  Serial.println();
}

static void printHelp() {
  Serial.println("Команды (отправьте символ в Serial Monitor):");
  Serial.println("  i  — GET_INFO");
  Serial.println("  h  — GET_HEALTH");
  Serial.println("  s  — стандартный SCAN (~4.6 кГц)");
  Serial.println("  e  — typical EXPRESS_SCAN (у S1 обычно DenseBoost 9.2 кГц)");
  Serial.println("  x  — STOP");
  Serial.println("  r  — RESET");
  Serial.println("  v  — подробный CSV: угол,мм,качество");
  Serial.println("  m  — мотор 600 RPM (S1)");
  Serial.println("Каждый оборот: бинарный кадр AA 55 LD + строка #rev");
}

static bool ensureHealthy() {
  RpHealth health{};
  if (!lidar.getHealth(health)) {
    Serial.println("GET_HEALTH: нет ответа. Проверьте TX/RX, GND и 5V лидара.");
    return false;
  }
  Serial.printf("health=%s error=%u\n", healthText(health.status),
                health.errorCode);
  if (health.status == 2) {
    Serial.println("Protection Stop — пробую RESET...");
    lidar.reset();
    if (!lidar.getHealth(health) || health.status == 2) {
      Serial.println("Лидар в защите. Питание/механика.");
      return false;
    }
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("BOOT");
  Serial.printf("reset_reason=%d\n", static_cast<int>(esp_reset_reason()));
  Serial.println("ESP32-S3 + RPLIDAR S1M1");
  Serial.printf("Лидар UART: RX=GPIO%d TX=GPIO%d baud=%lu 8N1\n", kLidarRxPin,
                kLidarTxPin, static_cast<unsigned long>(RpLidar::kBaud));
  printHelp();

  lidar.begin(kLidarRxPin, kLidarTxPin);
  lidar.setPointCallback(onPoint);
  lidar.reset();
  lidar.dumpRx(Serial, 400);

  RpDeviceInfo info{};
  if (!lidar.getInfo(info)) {
    Serial.println("GET_INFO не прошёл. Нет связи с лидаром.");
    return;
  }
  printInfo(info);

  if (!ensureHealthy()) {
    return;
  }

  uint16_t modeId = 0;
  char modeName[32] = {0};
  if (lidar.getTypicalScanMode(modeId)) {
    lidar.getScanModeName(modeId, modeName, sizeof(modeName));
    Serial.printf("typical scan mode id=%u name=%s\n", modeId,
                  modeName[0] ? modeName : "?");
  }

  Serial.println("Стартую typical scan...");
  lidar.setMotorRpm(RpLidar::kDefaultRpm);
  delay(200);
  haveFrame = false;
  resetRevStats();
  revStartMs = 0;
  if (lidar.startTypicalScan(&modeId)) {
    Serial.printf("скан идёт, format=0x%02X mode=%u\n",
                  static_cast<unsigned>(lidar.scanFormat()), modeId);
    delay(250);
    Serial.printf("uart avail=%d\n", lidar.rxAvailable());
  } else {
    Serial.println("Не удалось начать скан.");
  }
}

void loop() {
  lidar.poll();

  if (haveFrame) {
    haveFrame = false;
    sendScanFrame();
  }

  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 3000) {
    lastBeat = millis();
    if (lidar.scanFormat() == RpScanFormat::None) {
      Serial.println("ожидаю команды: i/h/s/e/x/r/?");
    } else {
      Serial.printf("scan fmt=0x%02X revs=%lu pts=%lu rx=%lu ok=%lu fail=%lu avail=%d\n",
                    static_cast<unsigned>(lidar.scanFormat()),
                    static_cast<unsigned long>(lidar.revolutions()),
                    static_cast<unsigned long>(lidar.pointsThisRev()),
                    static_cast<unsigned long>(lidar.rxBytes()),
                    static_cast<unsigned long>(lidar.denseOk()),
                    static_cast<unsigned long>(lidar.denseFail()),
                    lidar.rxAvailable());
    }
  }

  if (!Serial.available()) {
    return;
  }
  const char c = static_cast<char>(Serial.read());
  if (c == '\n' || c == '\r') {
    return;
  }

  switch (c) {
    case 'i': {
      RpDeviceInfo info{};
      if (lidar.getInfo(info)) {
        printInfo(info);
      } else {
        Serial.println("GET_INFO fail");
      }
      break;
    }
    case 'h':
      ensureHealthy();
      break;
    case 's':
      Serial.println(lidar.startStandardScan() ? "SCAN ok" : "SCAN fail");
      haveFrame = false;
      resetRevStats();
      revStartMs = 0;
      break;
    case 'e': {
      uint16_t modeId = 0;
      Serial.println(lidar.startTypicalScan(&modeId)
                         ? "EXPRESS/typical ok"
                         : "EXPRESS fail");
      Serial.printf("mode=%u format=0x%02X\n", modeId,
                    static_cast<unsigned>(lidar.scanFormat()));
      haveFrame = false;
      resetRevStats();
      revStartMs = 0;
      break;
    }
    case 'x':
      lidar.stop();
      Serial.println("STOP");
      break;
    case 'r':
      lidar.reset();
      Serial.println("RESET");
      break;
    case 'v':
      verbose = !verbose;
      Serial.printf("verbose=%s\n", verbose ? "on" : "off");
      break;
    case 'm':
      lidar.setMotorRpm(RpLidar::kDefaultRpm);
      Serial.println("motor 600 rpm");
      break;
    case '?':
      printHelp();
      break;
    default:
      break;
  }
}
