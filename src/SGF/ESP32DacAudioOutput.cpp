#include "ESP32DacAudioOutput.h"

#include <Arduino.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if SGF_ENABLE_AUDIO_PROFILER
#include "SGF/SerialMonitor.h"
#endif

namespace SGFAudio {

ESP32DacAudioOutput::ESP32DacAudioOutput(IAudioSource& sourceRef, uint8_t pinRef)
  : source(sourceRef), pin(pinRef)
#if SGF_ENABLE_AUDIO_PROFILER
  , audioProfiler("audio", profilerSlots, PROFILER_SLOT_COUNT)
#endif
{
#if SGF_ENABLE_AUDIO_PROFILER
  audioProfiler.setLabel(FillCallsSlot, "fills");
  audioProfiler.setMode(FillCallsSlot, Profiler::CounterSlot);
  audioProfiler.setLabel(FillTimeSlot, "fill");
  audioProfiler.setMode(FillTimeSlot, Profiler::SampleSlot);
  audioProfiler.setLabel(RingUsedSlot, "ring");
  audioProfiler.setMode(RingUsedSlot, Profiler::SampleSlot);
  audioProfiler.setLabel(UnderrunSlot, "und");
  audioProfiler.setMode(UnderrunSlot, Profiler::CounterSlot);
  audioProfiler.setLabel(OverrunSlot, "ovr");
  audioProfiler.setMode(OverrunSlot, Profiler::CounterSlot);
#endif
}

ESP32DacAudioOutput::~ESP32DacAudioOutput() {
  end();
}

bool ESP32DacAudioOutput::begin() {
  if (taskHandle != nullptr || dacHandle != nullptr) {
    return true;
  }
  const dac_continuous_config_t config{
    .chan_mask = channelMask(),
    .desc_num = DMA_DESC_NUM,
    .buf_size = DMA_BUF_SIZE,
    .freq_hz = source.sampleRate(),
    .offset = 0,
    .clk_src = DAC_DIGI_CLK_SRC_APLL,
    .chan_mode = DAC_CHANNEL_MODE_SIMUL,
  };
  if (config.chan_mask == 0u) {
    return false;
  }
  if (dac_continuous_new_channels(&config, &dacHandle) != ESP_OK || dacHandle == nullptr) {
    dacHandle = nullptr;
    return false;
  }
  memset(writeBuffer, 128, sizeof(writeBuffer));
  memset(sampleBuffer, 0, sizeof(sampleBuffer));
  outputEnabled = false;
  silenceSamples = 0u;
  lastSample = 0;

  running = true;
  TaskHandle_t handle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(
    &ESP32DacAudioOutput::audioTaskThunk,
    "sgf_audio",
    4096,
    this,
    1,
    &handle,
    0);
  if (result != pdPASS || handle == nullptr) {
    running = false;
    dac_continuous_del_channels(dacHandle);
    dacHandle = nullptr;
    taskHandle = nullptr;
    return false;
  }
  taskHandle = handle;
  return true;
}

bool ESP32DacAudioOutput::attachSerialMonitor(SerialMonitor& serialMonitor) {
#if SGF_ENABLE_AUDIO_PROFILER
  return serialMonitor.attachProfiler(audioProfiler);
#else
  (void)serialMonitor;
  return false;
#endif
}

void ESP32DacAudioOutput::end() {
  if (taskHandle == nullptr && dacHandle == nullptr) {
    return;
  }
  running = false;
  while (taskHandle != nullptr) {
    delay(1);
  }
  if (dacHandle != nullptr) {
    disableOutput();
    dac_continuous_del_channels(dacHandle);
    dacHandle = nullptr;
  }
}

void ESP32DacAudioOutput::audioTaskThunk(void* arg) {
  if (arg == nullptr) {
    vTaskDelete(nullptr);
    return;
  }
  auto* output = static_cast<ESP32DacAudioOutput*>(arg);
  output->audioTaskLoop();
}

void ESP32DacAudioOutput::audioTaskLoop() {
  const uint32_t sampleRate = source.sampleRate();
  const uint32_t holdSamples =
    (sampleRate * 1ull * SGF_ESP32_DAC_SILENCE_HOLD_MS) / 1000u;
  const uint32_t fadeOutSamplesTotal =
    (sampleRate * 1ull * SGF_ESP32_DAC_FADE_OUT_MS) / 1000u;
  const uint32_t fadeInSamplesTotal =
    (sampleRate * 1ull * SGF_ESP32_DAC_FADE_IN_MS) / 1000u;
  const TickType_t silentDelayTicks = pdMS_TO_TICKS(
    ((AUDIO_CHUNK_SIZE * 1000ull) + sampleRate - 1u) / sampleRate);
  while (running && dacHandle != nullptr) {
    bool hasSignal = false;
    for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
      const int16_t sample = source.renderSample();
      sampleBuffer[i] = sample;
      if (sample > SGF_ESP32_DAC_SILENCE_THRESHOLD ||
          sample < -SGF_ESP32_DAC_SILENCE_THRESHOLD) {
        hasSignal = true;
      }
    }

    if (!hasSignal) {
      if (outputEnabled && silenceSamples == 0u && fadeOutSamplesTotal > 0u) {
        const uint32_t fadeCount =
          fadeOutSamplesTotal < AUDIO_CHUNK_SIZE ? fadeOutSamplesTotal : AUDIO_CHUNK_SIZE;
        const int32_t fadeDen = fadeCount;
        for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
          const int32_t numerator = i < fadeCount ? fadeDen - i : 0;
          const int16_t faded = numerator > 0 ? (lastSample * numerator) / fadeDen : 0;
          writeBuffer[i] = sampleToDac(faded);
        }
        lastSample = 0;
      } else {
        memset(writeBuffer, 128, sizeof(writeBuffer));
      }
      silenceSamples += AUDIO_CHUNK_SIZE;
      if (outputEnabled) {
#if SGF_ENABLE_AUDIO_PROFILER
        const uint32_t fillStartUs = micros();
#endif
        size_t bytesLoaded = 0u;
        const esp_err_t err = dac_continuous_write(
          dacHandle,
          writeBuffer,
          sizeof(writeBuffer),
          &bytesLoaded,
          20);
#if SGF_ENABLE_AUDIO_PROFILER
        audioProfiler.increment(FillCallsSlot);
        audioProfiler.probe(FillTimeSlot, micros() - fillStartUs);
        const uint32_t bufferedUs = (bytesLoaded * 1000000ull) / sampleRate;
        audioProfiler.probe(RingUsedSlot, bufferedUs);
#endif
        if (err == ESP_ERR_TIMEOUT) {
          overrunCount++;
        } else if (err != ESP_OK || bytesLoaded < sizeof(writeBuffer)) {
          underrunCount++;
        }
      } else {
#if SGF_ENABLE_AUDIO_PROFILER
        audioProfiler.increment(FillCallsSlot);
        audioProfiler.probe(FillTimeSlot, 0u);
        audioProfiler.probe(RingUsedSlot, 0u);
#endif
      }

      if (outputEnabled && silenceSamples >= holdSamples) {
        disableOutput();
      }
#if SGF_ENABLE_AUDIO_PROFILER
      if (underrunCount > 0u) {
        audioProfiler.increment(UnderrunSlot, underrunCount);
        underrunCount = 0u;
      }
      if (overrunCount > 0u) {
        audioProfiler.increment(OverrunSlot, overrunCount);
        overrunCount = 0u;
      }
#endif
      vTaskDelay(silentDelayTicks > 0 ? silentDelayTicks : 1);
      continue;
    }

    silenceSamples = 0u;
    const bool wasMuted = !outputEnabled;
    if (wasMuted && !enableOutput()) {
      overrunCount++;
      vTaskDelay(1);
      continue;
    }

    if (wasMuted && fadeInSamplesTotal > 0u) {
      const uint32_t fadeCount =
        fadeInSamplesTotal < AUDIO_CHUNK_SIZE ? fadeInSamplesTotal : AUDIO_CHUNK_SIZE;
      const int32_t fadeDen = fadeCount;
      for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
        int32_t sample = sampleBuffer[i];
        if (i < fadeCount) {
          sample = (sample * (i + 1u)) / fadeDen;
        }
        writeBuffer[i] = sampleToDac(sample);
      }
    } else {
      for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
        writeBuffer[i] = sampleToDac(sampleBuffer[i]);
      }
    }
    lastSample = sampleBuffer[AUDIO_CHUNK_SIZE - 1];

#if SGF_ENABLE_AUDIO_PROFILER
    const uint32_t fillStartUs = micros();
#endif
    size_t bytesLoaded = 0u;
    const esp_err_t err = dac_continuous_write(
      dacHandle,
      writeBuffer,
      sizeof(writeBuffer),
      &bytesLoaded,
      20);
#if SGF_ENABLE_AUDIO_PROFILER
    audioProfiler.increment(FillCallsSlot);
    audioProfiler.probe(FillTimeSlot, micros() - fillStartUs);
    const uint32_t bufferedUs = (bytesLoaded * 1000000ull) / source.sampleRate();
    audioProfiler.probe(RingUsedSlot, bufferedUs);
#endif
    if (err == ESP_ERR_TIMEOUT) {
      overrunCount++;
    } else if (err != ESP_OK || bytesLoaded < sizeof(writeBuffer)) {
      underrunCount++;
    }
#if SGF_ENABLE_AUDIO_PROFILER
    if (underrunCount > 0u) {
      audioProfiler.increment(UnderrunSlot, underrunCount);
      underrunCount = 0u;
    }
    if (overrunCount > 0u) {
      audioProfiler.increment(OverrunSlot, overrunCount);
      overrunCount = 0u;
    }
#endif
  }
  taskHandle = nullptr;
  vTaskDelete(nullptr);
}

bool ESP32DacAudioOutput::enableOutput() {
  if (dacHandle == nullptr) {
    return false;
  }
  if (outputEnabled) {
    return true;
  }
  if (dac_continuous_enable(dacHandle) != ESP_OK) {
    return false;
  }
  outputEnabled = true;
  return true;
}

void ESP32DacAudioOutput::disableOutput() {
  if (!outputEnabled || dacHandle == nullptr) {
    outputEnabled = false;
    return;
  }
  dac_continuous_disable(dacHandle);
  outputEnabled = false;
}

dac_channel_mask_t ESP32DacAudioOutput::channelMask() const {
  if (pin == 25u) {
    return DAC_CHANNEL_MASK_CH0;
  }
  if (pin == 26u) {
    return DAC_CHANNEL_MASK_CH1;
  }
  return {};
}

uint8_t ESP32DacAudioOutput::sampleToDac(int16_t sample) {
  return (sample + 32768) >> 8;
}

}  // namespace SGFAudio
