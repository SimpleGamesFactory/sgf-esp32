#pragma once

#include <SPI.h>
#include <driver/spi_master.h>

#include "SGF/IDisplayBus.h"

class SPIESP32DisplayBusDMA : public IDisplayBus {
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
    int host = PIN_UNUSED;
  };

  explicit SPIESP32DisplayBusDMA(const Config& config);
  ~SPIESP32DisplayBusDMA() override;

  bool begin(uint32_t spiHz) override;
  void setFrequency(uint32_t spiHz) override;
  void hardwareReset() override;

  void writeCommand(uint8_t command) override;
  void writeData(const uint8_t* bytes, size_t size) override;

  void beginDataWrite() override;
  void writeDataChunk(const uint8_t* bytes, size_t size) override;
  void endDataWrite() override;
  bool supportsWritePixels565() const override { return true; }
  bool writePixels565ExpectsByteSwapped() const override { return true; }
  bool supportsQueuedWritePixels565() const override { return true; }
  void writePixels565(const uint16_t* pixels, size_t count) override;
  void queueWritePixels565(const uint16_t* pixels, size_t count) override;
  void waitQueuedWritePixels565() override;
  void finishQueuedWritePixels565() override;

  void setBacklight(uint8_t level) override;

private:
  static constexpr size_t QUEUED_TX_DEPTH = 4u;

  Config config_;
  uint32_t spiHz_ = 0;
  spi_host_device_t host_ = SPI2_HOST;
  spi_device_handle_t device_ = nullptr;
  bool busInitialized_ = false;
  bool dataWriteOpen_ = false;
  bool backlightPwmConfigured_ = false;
  spi_transaction_t queuedTrans_[QUEUED_TX_DEPTH]{};
  size_t queuedHead_ = 0;
  size_t queuedCount_ = 0;

  spi_host_device_t resolveHost() const;
  bool initBus();
  void deinitBus();
  void resetQueuedWrites();
  void setCs(bool active);
  void setDc(bool dataMode);
  void transmit(const void* bytes, size_t size, bool keepCsActive);
  void queueTransmit(const void* bytes, size_t size);
  void waitOneQueuedTransmit();
  bool ensureBacklightPwmConfigured();
};
