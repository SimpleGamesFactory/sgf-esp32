#pragma once

#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <driver/dac_continuous.h>

#include "SGF/IAudioSource.h"

#if defined(ENABLE_PROFILER) && ENABLE_PROFILER
#define SGF_ENABLE_AUDIO_PROFILER 1
#include "SGF/Profiler.h"
#else
#define SGF_ENABLE_AUDIO_PROFILER 0
#endif

#ifndef SGF_ESP32_DAC_AUTO_MUTE
#define SGF_ESP32_DAC_AUTO_MUTE 0
#endif

#ifndef SGF_ESP32_DAC_SILENCE_THRESHOLD
#define SGF_ESP32_DAC_SILENCE_THRESHOLD 96
#endif

#ifndef SGF_ESP32_DAC_SILENCE_HOLD_MS
#define SGF_ESP32_DAC_SILENCE_HOLD_MS 320u
#endif

#ifndef SGF_ESP32_DAC_FADE_OUT_MS
#define SGF_ESP32_DAC_FADE_OUT_MS 24u
#endif

#ifndef SGF_ESP32_DAC_FADE_IN_MS
#define SGF_ESP32_DAC_FADE_IN_MS 16u
#endif

class SerialMonitor;

namespace SGFAudio {

class ESP32DacAudioOutput {
public:
  ESP32DacAudioOutput(IAudioSource& source, uint8_t pin);
  ~ESP32DacAudioOutput();

  bool begin();
  void end();
  bool isRunning() const { return taskHandle != nullptr; }
#if SGF_ENABLE_AUDIO_PROFILER
  Profiler& profiler() { return audioProfiler; }
  const Profiler& profiler() const { return audioProfiler; }
#endif
  bool attachSerialMonitor(SerialMonitor& serialMonitor);

private:
  static constexpr uint32_t DMA_DESC_NUM = 6u;
  static constexpr size_t DMA_BUF_SIZE = 256u;
  static constexpr size_t AUDIO_CHUNK_SIZE = 256u;
  static constexpr uint8_t PCM_RING_CHUNKS = 6u;
  static constexpr uint32_t GATE_GAIN_MAX = 32768u;
  static_assert(AUDIO_CHUNK_SIZE <= DMA_BUF_SIZE, "AUDIO_CHUNK_SIZE must fit in DMA_BUF_SIZE");

#if SGF_ENABLE_AUDIO_PROFILER
  enum ProfilerSlot : uint8_t {
    FillCallsSlot = 0,
    FillTimeSlot,
    RingUsedSlot,
    UnderrunSlot,
    OverrunSlot,
    PROFILER_SLOT_COUNT
  };
#endif

  static void audioTaskThunk(void* arg);
  void audioTaskLoop();
  dac_channel_mask_t channelMask() const;
  bool enableOutput();
  void disableOutput();
  static uint8_t sampleToDac(int16_t sample);
  bool produceChunk(int16_t* chunk, uint8_t* hasSignalOut, uint16_t* firstSignalOut);

  IAudioSource& source;
  uint8_t pin = 25u;
  void* taskHandle = nullptr;
  dac_continuous_handle_t dacHandle = nullptr;
  volatile bool running = false;
  uint32_t underrunCount = 0u;
  uint32_t overrunCount = 0u;
  bool outputEnabled = false;
  uint32_t silenceSamples = 0u;
  int16_t lastSample = 0;
  uint32_t nextChunkUs = 0u;
  uint8_t ringRead = 0u;
  uint8_t ringWrite = 0u;
  uint8_t ringCount = 0u;
#if SGF_ESP32_DAC_AUTO_MUTE
  uint32_t gateGain = GATE_GAIN_MAX;
  uint32_t gateGainTarget = GATE_GAIN_MAX;
  uint32_t gateFadeInStep = GATE_GAIN_MAX;
  uint32_t gateFadeOutStep = GATE_GAIN_MAX;
#endif
  uint8_t writeBuffer[AUDIO_CHUNK_SIZE]{};
  int16_t pcmRing[PCM_RING_CHUNKS][AUDIO_CHUNK_SIZE]{};
  uint8_t pcmHasSignal[PCM_RING_CHUNKS]{};
  uint16_t pcmFirstSignal[PCM_RING_CHUNKS]{};
#if SGF_ENABLE_AUDIO_PROFILER
  Profiler::Slot profilerSlots[PROFILER_SLOT_COUNT]{};
  Profiler audioProfiler;
#endif
};

}  // namespace SGFAudio
