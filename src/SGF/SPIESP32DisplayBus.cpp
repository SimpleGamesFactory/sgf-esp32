#include "SPIESP32DisplayBus.h"

#include <Arduino.h>

namespace {

constexpr uint32_t DISPLAY_LEVEL_MAX = 255u;
constexpr uint32_t BACKLIGHT_PWM_FREQ_HZ = 5000u;
constexpr uint8_t BACKLIGHT_PWM_RESOLUTION_BITS = 12u;
constexpr uint32_t BACKLIGHT_PWM_LEVEL_MAX = (1u << BACKLIGHT_PWM_RESOLUTION_BITS) - 1u;
constexpr bool BACKLIGHT_ACTIVE_LOW = false;

}  // namespace

SPIESP32DisplayBus::SPIESP32DisplayBus(const Config& config) : config_(config) {}

bool SPIESP32DisplayBus::begin(uint32_t spiHz) {
  if (config_.dcPin < 0) {
    return false;
  }

  if (config_.csPin >= 0) {
    pinMode(config_.csPin, OUTPUT);
    digitalWrite(config_.csPin, HIGH);
  }
  pinMode(config_.dcPin, OUTPUT);
  if (config_.rstPin >= 0) {
    pinMode(config_.rstPin, OUTPUT);
  }

  digitalWrite(config_.dcPin, HIGH);
  if (config_.rstPin >= 0) {
    digitalWrite(config_.rstPin, HIGH);
  }

  if (config_.sckPin >= 0 || config_.mosiPin >= 0 || config_.misoPin >= 0) {
    spi().begin(config_.sckPin, config_.misoPin, config_.mosiPin, config_.csPin);
  } else {
    spi().begin();
  }

  setFrequency(spiHz);
  return true;
}

void SPIESP32DisplayBus::setFrequency(uint32_t spiHz) {
  spiHz_ = spiHz;
}

void SPIESP32DisplayBus::hardwareReset() {
  if (config_.rstPin < 0) {
    return;
  }
  digitalWrite(config_.rstPin, HIGH);
  delay(5);
  digitalWrite(config_.rstPin, LOW);
  delay(20);
  digitalWrite(config_.rstPin, HIGH);
  delay(120);
}

void SPIESP32DisplayBus::writeCommand(uint8_t command) {
  beginTransaction();
  digitalWrite(config_.dcPin, LOW);
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, LOW);
  }
  spi().transfer(command);
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, HIGH);
  }
  endTransaction();
}

void SPIESP32DisplayBus::writeData(const uint8_t* bytes, size_t size) {
  beginTransaction();
  digitalWrite(config_.dcPin, HIGH);
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, LOW);
  }
  transfer(bytes, size);
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, HIGH);
  }
  endTransaction();
}

void SPIESP32DisplayBus::beginDataWrite() {
  beginTransaction();
  digitalWrite(config_.dcPin, HIGH);
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, LOW);
  }
}

void SPIESP32DisplayBus::writeDataChunk(const uint8_t* bytes, size_t size) {
  transfer(bytes, size);
}

void SPIESP32DisplayBus::endDataWrite() {
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, HIGH);
  }
  endTransaction();
}

void SPIESP32DisplayBus::writePixels565(const uint16_t* pixels, size_t count) {
  if (!pixels || count == 0u) {
    return;
  }
  spi().writePixels(pixels, count * sizeof(uint16_t));
}

void SPIESP32DisplayBus::setBacklight(uint8_t level) {
  if (config_.ledPin < 0) {
    return;
  }
  if (!ensureBacklightPwmConfigured()) {
    return;
  }

  uint8_t effectiveLevel = BACKLIGHT_ACTIVE_LOW ? (DISPLAY_LEVEL_MAX - level) : level;
  uint32_t duty = (effectiveLevel * BACKLIGHT_PWM_LEVEL_MAX +
                   (DISPLAY_LEVEL_MAX / 2u)) / DISPLAY_LEVEL_MAX;
  ledcWrite(config_.ledPin, duty);
}

void SPIESP32DisplayBus::beginTransaction() {
  if (transactionOpen_) {
    return;
  }
  spi().beginTransaction(SPISettings(spiHz_, MSBFIRST, config_.dataMode));
  transactionOpen_ = true;
}

void SPIESP32DisplayBus::endTransaction() {
  if (!transactionOpen_) {
    return;
  }
  spi().endTransaction();
  transactionOpen_ = false;
}

void SPIESP32DisplayBus::transfer(const uint8_t* bytes, size_t size) {
  if (!bytes || size == 0) {
    return;
  }
  spi().writeBytes(bytes, size);
}

bool SPIESP32DisplayBus::ensureBacklightPwmConfigured() {
  if (config_.ledPin < 0) {
    return false;
  }
  if (backlightPwmConfigured_) {
    return true;
  }
  if (!ledcAttach(
        config_.ledPin,
        BACKLIGHT_PWM_FREQ_HZ,
        BACKLIGHT_PWM_RESOLUTION_BITS)) {
    return false;
  }

  backlightPwmConfigured_ = true;
  return true;
}
