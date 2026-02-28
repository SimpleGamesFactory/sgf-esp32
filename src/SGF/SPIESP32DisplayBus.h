#pragma once

#include <SPI.h>

#include "SGF/IDisplayBus.h"

class SPIESP32DisplayBus : public IDisplayBus {
public:
  static constexpr int PIN_UNUSED = -1;

  struct Config {
    SPIClass* spi = &SPI;
    int sckPin = PIN_UNUSED;
    int misoPin = PIN_UNUSED;
    int mosiPin = PIN_UNUSED;
    int csPin = PIN_UNUSED;
    int dcPin = PIN_UNUSED;
    int rstPin = PIN_UNUSED;
    int ledPin = PIN_UNUSED;
    uint8_t dataMode = SPI_MODE0;
  };

  explicit SPIESP32DisplayBus(const Config& config);

  bool begin(uint32_t spiHz) override;
  void setFrequency(uint32_t spiHz) override;
  void hardwareReset() override;

  void writeCommand(uint8_t command) override;
  void writeData(const uint8_t* bytes, size_t size) override;

  void beginDataWrite() override;
  void writeDataChunk(const uint8_t* bytes, size_t size) override;
  void endDataWrite() override;
  bool supportsWritePixels565() const override { return true; }
  void writePixels565(const uint16_t* pixels, size_t count) override;

  void setBacklight(uint8_t level) override;

private:
  Config config_;
  uint32_t spiHz_ = 0;
  bool transactionOpen_ = false;
  bool backlightPwmConfigured_ = false;

  SPIClass& spi() const { return *config_.spi; }
  void beginTransaction();
  void endTransaction();
  void transfer(const uint8_t* bytes, size_t size);
  bool ensureBacklightPwmConfigured();
};
