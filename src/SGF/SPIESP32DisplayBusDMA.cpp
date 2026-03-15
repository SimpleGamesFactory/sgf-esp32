#include "SPIESP32DisplayBusDMA.h"

#include <Arduino.h>
#include <esp_err.h>

namespace {

constexpr uint32_t DISPLAY_LEVEL_MAX = 255u;
constexpr uint32_t BACKLIGHT_PWM_FREQ_HZ = 5000u;
constexpr uint8_t BACKLIGHT_PWM_RESOLUTION_BITS = 12u;
constexpr uint32_t BACKLIGHT_PWM_LEVEL_MAX = (1u << BACKLIGHT_PWM_RESOLUTION_BITS) - 1u;
constexpr bool BACKLIGHT_ACTIVE_LOW = false;
constexpr size_t DISPLAY_DMA_MAX_TRANSFER = 128u * 1024u;

}  // namespace

SPIESP32DisplayBusDMA::SPIESP32DisplayBusDMA(const Config& config) : config_(config) {}

SPIESP32DisplayBusDMA::~SPIESP32DisplayBusDMA() {
  endDataWrite();
  deinitBus();
}

bool SPIESP32DisplayBusDMA::begin(uint32_t spiHz) {
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

  spiHz_ = spiHz;
  host_ = resolveHost();
  return initBus();
}

void SPIESP32DisplayBusDMA::setFrequency(uint32_t spiHz) {
  spiHz_ = spiHz;
}

void SPIESP32DisplayBusDMA::hardwareReset() {
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

void SPIESP32DisplayBusDMA::writeCommand(uint8_t command) {
  setDc(false);
  setCs(true);
  transmit(&command, sizeof(command), false);
  setCs(false);
}

void SPIESP32DisplayBusDMA::writeData(const uint8_t* bytes, size_t size) {
  if (!bytes || size == 0u) {
    return;
  }
  setDc(true);
  setCs(true);
  transmit(bytes, size, false);
  setCs(false);
}

void SPIESP32DisplayBusDMA::beginDataWrite() {
  if (dataWriteOpen_) {
    return;
  }
  if (device_) {
    spi_device_acquire_bus(device_, portMAX_DELAY);
  }
  resetQueuedWrites();
  setDc(true);
  setCs(true);
  dataWriteOpen_ = true;
}

void SPIESP32DisplayBusDMA::writeDataChunk(const uint8_t* bytes, size_t size) {
  if (!bytes || size == 0u) {
    return;
  }
  transmit(bytes, size, dataWriteOpen_);
}

void SPIESP32DisplayBusDMA::endDataWrite() {
  if (!dataWriteOpen_) {
    return;
  }
  finishQueuedWritePixels565();
  setCs(false);
  if (device_) {
    spi_device_release_bus(device_);
  }
  dataWriteOpen_ = false;
}

void SPIESP32DisplayBusDMA::writePixels565(const uint16_t* pixels, size_t count) {
  if (!pixels || count == 0u) {
    return;
  }
  transmit(pixels, count * sizeof(uint16_t), dataWriteOpen_);
}

void SPIESP32DisplayBusDMA::queueWritePixels565(const uint16_t* pixels, size_t count) {
  if (!pixels || count == 0u) {
    return;
  }
  if (!dataWriteOpen_) {
    writePixels565(pixels, count);
    return;
  }

  const uint8_t* bytes = static_cast<const uint8_t*>(static_cast<const void*>(pixels));
  size_t remaining = count * sizeof(uint16_t);
  while (remaining > 0u) {
    size_t chunk = remaining;
    if (chunk > DISPLAY_DMA_MAX_TRANSFER) {
      chunk = DISPLAY_DMA_MAX_TRANSFER;
    }
    queueTransmit(bytes, chunk);
    bytes += chunk;
    remaining -= chunk;
  }
}

void SPIESP32DisplayBusDMA::waitQueuedWritePixels565() {
  if (queuedCount_ == 0u) {
    return;
  }
  waitOneQueuedTransmit();
}

void SPIESP32DisplayBusDMA::finishQueuedWritePixels565() {
  while (queuedCount_ > 0u) {
    waitOneQueuedTransmit();
  }
}

void SPIESP32DisplayBusDMA::setBacklight(uint8_t level) {
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

spi_host_device_t SPIESP32DisplayBusDMA::resolveHost() const {
  if (config_.host != PIN_UNUSED) {
    return static_cast<spi_host_device_t>(config_.host);
  }

#if CONFIG_IDF_TARGET_ESP32
  return SPI3_HOST;
#else
  return SPI2_HOST;
#endif
}

bool SPIESP32DisplayBusDMA::initBus() {
  if (busInitialized_) {
    return true;
  }

  spi_bus_config_t buscfg = {};
  buscfg.sclk_io_num = config_.sckPin;
  buscfg.mosi_io_num = config_.mosiPin;
  buscfg.miso_io_num = config_.misoPin;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.data4_io_num = -1;
  buscfg.data5_io_num = -1;
  buscfg.data6_io_num = -1;
  buscfg.data7_io_num = -1;
  buscfg.max_transfer_sz = DISPLAY_DMA_MAX_TRANSFER;

  esp_err_t err = spi_bus_initialize(host_, &buscfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK) {
    return false;
  }

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = spiHz_;
  devcfg.mode = config_.dataMode;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 4;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;

  err = spi_bus_add_device(host_, &devcfg, &device_);
  if (err != ESP_OK) {
    spi_bus_free(host_);
    return false;
  }

  busInitialized_ = true;
  return true;
}

void SPIESP32DisplayBusDMA::deinitBus() {
  if (!busInitialized_) {
    return;
  }
  resetQueuedWrites();
  if (device_) {
    spi_bus_remove_device(device_);
    device_ = nullptr;
  }
  spi_bus_free(host_);
  busInitialized_ = false;
}

void SPIESP32DisplayBusDMA::resetQueuedWrites() {
  queuedHead_ = 0u;
  queuedCount_ = 0u;
  for (size_t i = 0; i < QUEUED_TX_DEPTH; i++) {
    queuedTrans_[i] = {};
  }
}

void SPIESP32DisplayBusDMA::setCs(bool active) {
  if (config_.csPin >= 0) {
    digitalWrite(config_.csPin, active ? LOW : HIGH);
  }
}

void SPIESP32DisplayBusDMA::setDc(bool dataMode) {
  digitalWrite(config_.dcPin, dataMode ? HIGH : LOW);
}

void SPIESP32DisplayBusDMA::transmit(const void* bytes, size_t size, bool keepCsActive) {
  if (!bytes || size == 0u || !device_) {
    return;
  }

  spi_transaction_t trans = {};
  trans.length = size * 8u;
  trans.tx_buffer = bytes;
  trans.override_freq_hz = spiHz_;
  if (keepCsActive) {
    trans.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  }
  spi_device_polling_transmit(device_, &trans);
}

void SPIESP32DisplayBusDMA::queueTransmit(const void* bytes, size_t size) {
  if (!bytes || size == 0u || !device_) {
    return;
  }

  if (queuedCount_ == QUEUED_TX_DEPTH) {
    waitOneQueuedTransmit();
  }

  size_t slot = (queuedHead_ + queuedCount_) % QUEUED_TX_DEPTH;
  spi_transaction_t& trans = queuedTrans_[slot];
  trans = {};
  trans.length = size * 8u;
  trans.tx_buffer = bytes;
  trans.override_freq_hz = spiHz_;
  spi_device_queue_trans(device_, &trans, portMAX_DELAY);
  queuedCount_++;
}

void SPIESP32DisplayBusDMA::waitOneQueuedTransmit() {
  if (queuedCount_ == 0u || !device_) {
    return;
  }

  spi_transaction_t* completed = nullptr;
  spi_device_get_trans_result(device_, &completed, portMAX_DELAY);
  (void)completed;
  queuedHead_ = (queuedHead_ + 1u) % QUEUED_TX_DEPTH;
  queuedCount_--;
}

bool SPIESP32DisplayBusDMA::ensureBacklightPwmConfigured() {
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
