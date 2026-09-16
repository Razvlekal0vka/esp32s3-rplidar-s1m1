#pragma once

#include <Arduino.h>

// Форматы ответа SCAN / answer types from the SCAN descriptor (SLAMTEC protocol).
enum class RpScanFormat : uint8_t {
  None = 0,
  Standard = 0x81,       // 5-байтовая точка / 5-byte node
  ExpressLegacy = 0x82,  // старая капсула / legacy capsule (не разбираем / unused)
  UltraCapsule = 0x84,   // ultra capsule (не разбираем / unused)
  Dense = 0x85,          // DenseBoost S1: 84 байта, 40 дистанций / 84 B, 40 ranges
};

// GET_INFO: модель, прошивка, железо, серийник / device identity payload.
struct RpDeviceInfo {
  uint8_t model;
  uint8_t firmwareMinor;
  uint8_t firmwareMajor;
  uint8_t hardware;
  uint8_t serial[16];
};

// GET_HEALTH: 0=ok, 1=warning, 2=protection stop (нужен RESET / needs RESET).
struct RpHealth {
  uint8_t status;
  uint16_t errorCode;
};

// Драйвер UART RPLIDAR S1 / UART driver for RPLIDAR S1 (256000 8N1).
class RpLidar {
 public:
  static constexpr uint32_t kBaud = 256000;          // штатный baud S1 / S1 default
  static constexpr uint16_t kDefaultRpm = 600;       // типовые обороты S1 / typical S1 RPM
  static constexpr uint32_t kAngleQ6Full = 360 * 64;  // полный круг в Q6 / 360° as Q6

  // newScan=true в начале оборота / true at the start of a revolution.
  using PointCallback = void (*)(float angleDeg, float distMm, uint8_t quality,
                                 bool newScan);

  // rxPin — GPIO ESP, куда приходит TX лидара / ESP GPIO that receives lidar TX.
  RpLidar(HardwareSerial& serial, int8_t rxPin, int8_t txPin);

  bool begin();
  bool begin(int8_t rxPin, int8_t txPin, uint32_t baud = kBaud);
  void stop();   // STOP 0x25, сброс парсера / STOP then drop scan state
  void reset();  // RESET 0x40, пауза ~600 мс / RESET then wait for reboot
  size_t dumpRx(Stream& out, uint32_t waitMs);

  bool getInfo(RpDeviceInfo& info, uint32_t timeoutMs = 1000);
  bool getHealth(RpHealth& health, uint32_t timeoutMs = 1000);
  bool getTypicalScanMode(uint16_t& modeId, uint32_t timeoutMs = 1000);
  bool getScanModeName(uint16_t modeId, char* name, size_t nameMax,
                       uint32_t timeoutMs = 1000);
  bool getScanModeAnsType(uint16_t modeId, uint8_t& ansType,
                          uint32_t timeoutMs = 1000);

  // S1: RPM; 0 останавливает мотор. Обычно 480–900, типовое 600.
  // S1: RPM; 0 stops the motor. Typical range 480–900, default 600.
  bool setMotorRpm(uint16_t rpm);

  bool startStandardScan();  // SCAN 0x20, ~4.6 кГц / classic 5-byte nodes
  // workingMode — id режима EXPRESS / EXPRESS_SCAN mode id (S1 typical = Dense).
  bool startExpressScan(uint8_t workingMode);
  // Typical для S1 — DenseBoost 0x85; иначе откат на SCAN.
  // S1 typical is DenseBoost 0x85; falls back to standard SCAN.
  bool startTypicalScan(uint16_t* usedMode = nullptr);

  // Читать UART и отдавать точки; вызывать часто из loop().
  // Drain UART and emit points; call often from loop().
  void poll();
  void setPointCallback(PointCallback cb) { pointCb_ = cb; }

  RpScanFormat scanFormat() const { return scanFormat_; }
  uint32_t pointsThisRev() const { return pointsThisRev_; }
  uint32_t revolutions() const { return revolutions_; }
  uint32_t rxBytes() const { return rxBytes_; }
  uint32_t denseOk() const { return denseOk_; }
  uint32_t denseFail() const { return denseFail_; }
  int rxAvailable() const { return uart_.available(); }

 private:
  HardwareSerial& uart_;
  int8_t rxPin_;
  int8_t txPin_;
  RpScanFormat scanFormat_ = RpScanFormat::None;
  PointCallback pointCb_ = nullptr;

  uint8_t stdBuf_[5]{};  // скользящее окно SCAN / 5-byte SCAN window
  uint8_t stdCount_ = 0;

  // DenseBoost: угол старта следующей капсулы нужен для интерполяции текущей.
  // Next capsule's start angle is required to interpolate the current one.
  uint8_t denseBuf_[84]{};
  uint8_t denseCount_ = 0;
  bool hasPrevDense_ = false;
  uint16_t prevStartQ6_ = 0;
  uint16_t prevDist_[40]{};
  bool prevS_ = false;

  uint32_t pointsThisRev_ = 0;
  uint32_t revolutions_ = 0;
  uint32_t rxBytes_ = 0;
  uint32_t denseOk_ = 0;
  uint32_t denseFail_ = 0;
  uint8_t lastDenseHead_[8]{};
  float lastAngleDeg_ = -1.0f;  // детектор перехода через 0° / wrap-around detector

  void flushInput(uint32_t waitMs = 0);
  void sendSimple(uint8_t cmd);
  void sendPayload(uint8_t cmd, const uint8_t* payload, uint8_t size);
  bool waitDescriptor(uint32_t& length, uint8_t& sendMode, uint8_t& type,
                      uint32_t timeoutMs);
  bool readExact(uint8_t* buf, size_t n, uint32_t timeoutMs);
  bool getLidarConf(uint32_t confType, const uint8_t* extra, uint8_t extraLen,
                    uint8_t* out, size_t outMax, size_t& outLen,
                    uint32_t timeoutMs);

  void handleStandardByte(uint8_t b);
  void handleDenseByte(uint8_t b);
  bool denseChecksumOk(const uint8_t* pkt) const;
  void emitDensePacket(uint16_t startQ6, uint16_t nextQ6, const uint16_t* dist,
                       bool startFlag);
  void emitPoint(float angleDeg, float distMm, uint8_t quality, bool newScan);
};
