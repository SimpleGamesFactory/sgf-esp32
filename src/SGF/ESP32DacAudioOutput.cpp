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
  memset(pcmRing, 0, sizeof(pcmRing));
  memset(pcmHasSignal, 0, sizeof(pcmHasSignal));
  for (uint8_t i = 0u; i < PCM_RING_CHUNKS; ++i) {
    pcmFirstSignal[i] = AUDIO_CHUNK_SIZE;
  }
  outputEnabled = false;
  silenceSamples = 0u;
  lastSample = 0;
  nextChunkUs = micros();
  ringRead = 0u;
  ringWrite = 0u;
  ringCount = 0u;
#if SGF_ESP32_DAC_AUTO_MUTE
  gateGain = 0u;
  gateGainTarget = 0u;
  gateFadeInStep = GATE_GAIN_MAX;
  gateFadeOutStep = GATE_GAIN_MAX;
#endif

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
#if SGF_ESP32_DAC_AUTO_MUTE
  const uint32_t sampleRate = source.sampleRate();
  const uint32_t holdSamples =
    (sampleRate * 1ull * SGF_ESP32_DAC_SILENCE_HOLD_MS) / 1000u;
  const uint32_t fadeOutSamplesTotal =
    (sampleRate * 1ull * SGF_ESP32_DAC_FADE_OUT_MS) / 1000u;
  const uint32_t fadeInSamplesTotal =
    (sampleRate * 1ull * SGF_ESP32_DAC_FADE_IN_MS) / 1000u;
  gateFadeInStep =
    fadeInSamplesTotal > 0u ? (GATE_GAIN_MAX + fadeInSamplesTotal - 1u) / fadeInSamplesTotal : GATE_GAIN_MAX;
  gateFadeOutStep =
    fadeOutSamplesTotal > 0u ? (GATE_GAIN_MAX + fadeOutSamplesTotal - 1u) / fadeOutSamplesTotal : GATE_GAIN_MAX;
#endif
  const uint32_t chunkUs =
    (AUDIO_CHUNK_SIZE * 1000000ull + source.sampleRate() - 1u) / source.sampleRate();
  while (running && dacHandle != nullptr) {
    const uint32_t nowUs = micros();
    while (ringCount < PCM_RING_CHUNKS && (int32_t)(nowUs - nextChunkUs) >= 0) {
      uint8_t hasSignal = 0u;
      uint16_t firstSignal = AUDIO_CHUNK_SIZE;
      produceChunk(pcmRing[ringWrite], &hasSignal, &firstSignal);
      pcmHasSignal[ringWrite] = hasSignal;
      pcmFirstSignal[ringWrite] = firstSignal;
      ringWrite = (ringWrite + 1u) % PCM_RING_CHUNKS;
      ++ringCount;
      nextChunkUs += chunkUs;
    }

    if (ringCount == 0u) {
#if SGF_ENABLE_AUDIO_PROFILER
      audioProfiler.increment(FillCallsSlot);
      audioProfiler.probe(FillTimeSlot, 0u);
      audioProfiler.probe(RingUsedSlot, 0u);
      if (underrunCount > 0u) {
        audioProfiler.increment(UnderrunSlot, underrunCount);
        underrunCount = 0u;
      }
      if (overrunCount > 0u) {
        audioProfiler.increment(OverrunSlot, overrunCount);
        overrunCount = 0u;
      }
#endif
      vTaskDelay(1);
      continue;
    }

    if (ringCount < 2u) {
#if SGF_ENABLE_AUDIO_PROFILER
      audioProfiler.increment(FillCallsSlot);
      audioProfiler.probe(FillTimeSlot, 0u);
      audioProfiler.probe(RingUsedSlot, 0u);
      if (underrunCount > 0u) {
        audioProfiler.increment(UnderrunSlot, underrunCount);
        underrunCount = 0u;
      }
      if (overrunCount > 0u) {
        audioProfiler.increment(OverrunSlot, overrunCount);
        overrunCount = 0u;
      }
#endif
      vTaskDelay(1);
      continue;
    }

    int16_t* chunk = pcmRing[ringRead];
    const bool hasSignal = pcmHasSignal[ringRead] != 0u;
    const uint8_t nextIndex = (ringRead + 1u) % PCM_RING_CHUNKS;
    const bool nextHasSignal = pcmHasSignal[nextIndex] != 0u;

#if SGF_ESP32_DAC_AUTO_MUTE
    if (hasSignal) {
      silenceSamples = 0u;
      gateGainTarget = GATE_GAIN_MAX;
    } else if (nextHasSignal) {
      silenceSamples = 0u;
      if (gateGain > 0u) {
        gateGainTarget = GATE_GAIN_MAX;
      }
    } else {
      silenceSamples += AUDIO_CHUNK_SIZE;
      gateGainTarget = silenceSamples >= holdSamples ? 0u : GATE_GAIN_MAX;
    }
#endif
    for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
      int16_t outSample = chunk[i];
#if SGF_ESP32_DAC_AUTO_MUTE
      if (gateGain < gateGainTarget) {
        gateGain += gateFadeInStep;
        if (gateGain > gateGainTarget) {
          gateGain = gateGainTarget;
        }
      } else if (gateGain > gateGainTarget) {
        gateGain = gateGain > gateFadeOutStep ? gateGain - gateFadeOutStep : 0u;
        if (gateGain < gateGainTarget) {
          gateGain = gateGainTarget;
        }
      }
      outSample = (outSample * gateGain) / GATE_GAIN_MAX;
#endif
      writeBuffer[i] = sampleToDac(outSample);
      lastSample = outSample;
    }

    bool shouldWrite = true;
#if SGF_ESP32_DAC_AUTO_MUTE
    if (gateGain == 0u && gateGainTarget == 0u && !hasSignal && !nextHasSignal) {
      shouldWrite = false;
      disableOutput();
    }
#endif

    if (shouldWrite && !outputEnabled) {
      if (!enableOutput()) {
        overrunCount++;
        vTaskDelay(1);
        continue;
      }
    }

#if SGF_ENABLE_AUDIO_PROFILER
    const uint32_t fillStartUs = micros();
#endif
    size_t bytesLoaded = 0u;
    esp_err_t err = ESP_OK;
    if (shouldWrite) {
      err = dac_continuous_write(
        dacHandle,
        writeBuffer,
        sizeof(writeBuffer),
        &bytesLoaded,
        20);
    }
#if SGF_ENABLE_AUDIO_PROFILER
    audioProfiler.increment(FillCallsSlot);
    audioProfiler.probe(FillTimeSlot, micros() - fillStartUs);
    const uint32_t bufferedUs = (bytesLoaded * 1000000ull) / source.sampleRate();
    audioProfiler.probe(RingUsedSlot, bufferedUs);
#endif
    if (!shouldWrite) {
      bytesLoaded = sizeof(writeBuffer);
    } else if (err == ESP_ERR_TIMEOUT) {
      overrunCount++;
    } else if (err != ESP_OK || bytesLoaded < sizeof(writeBuffer)) {
      underrunCount++;
    }

    ringRead = (ringRead + 1u) % PCM_RING_CHUNKS;
    --ringCount;
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

bool ESP32DacAudioOutput::produceChunk(int16_t* chunk, uint8_t* hasSignalOut, uint16_t* firstSignalOut) {
  if (chunk == nullptr || hasSignalOut == nullptr || firstSignalOut == nullptr) {
    return false;
  }
  uint8_t hasSignal = 0u;
  uint16_t firstSignal = AUDIO_CHUNK_SIZE;
  for (size_t i = 0; i < AUDIO_CHUNK_SIZE; ++i) {
    source.advanceSamples(1u);
    const int16_t sample = source.renderSample();
    chunk[i] = sample;
#if SGF_ESP32_DAC_AUTO_MUTE
    if (sample > SGF_ESP32_DAC_SILENCE_THRESHOLD ||
        sample < -SGF_ESP32_DAC_SILENCE_THRESHOLD) {
      hasSignal = 1u;
      if (firstSignal == AUDIO_CHUNK_SIZE) {
        firstSignal = i;
      }
    }
#endif
  }
  *hasSignalOut = hasSignal;
  *firstSignalOut = firstSignal;
  return true;
}

}  // namespace SGFAudio
