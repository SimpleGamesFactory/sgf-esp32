#pragma once

#include <stdint.h>

#include "SGF/IAudioSource.h"

#if defined(ENABLE_PROFILER) && ENABLE_PROFILER
#define SGF_ENABLE_AUDIO_PROFILER 1
#include "SGF/Profiler.h"
#else
#define SGF_ENABLE_AUDIO_PROFILER 0
#endif

struct timer_struct_t;
typedef struct timer_struct_t hw_timer_t;
class SerialMonitor;

namespace SGFAudio {

class ESP32DacSynthOutput {
public:
  ESP32DacSynthOutput(IAudioSource& source, uint8_t pin);
  ~ESP32DacSynthOutput();

  bool begin();
  void end();
  bool isRunning() const { return taskHandle != nullptr; }
#if SGF_ENABLE_AUDIO_PROFILER
  Profiler& profiler() { return audioProfiler; }
  const Profiler& profiler() const { return audioProfiler; }
#endif
  bool attachSerialMonitor(SerialMonitor& serialMonitor);

private:
  static constexpr uint16_t RING_BUFFER_SIZE = 512;
  static constexpr uint16_t RING_FILL_CHUNK = 64;

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
  static void onTimerThunk(void* arg);
  void audioTaskLoop();
  void onTimer();
  uint16_t ringUsed() const;
  uint16_t ringFree() const;
  void pushSample(uint8_t sample);
  uint8_t popSample();

  IAudioSource& source;
  uint8_t pin = 25u;
  void* taskHandle = nullptr;
  hw_timer_t* timer = nullptr;
  volatile bool running = false;
  volatile uint16_t readIndex = 0u;
  volatile uint16_t writeIndex = 0u;
  volatile uint32_t underrunCount = 0u;
  volatile uint32_t overrunCount = 0u;
  uint8_t ringBuffer[RING_BUFFER_SIZE]{};
#if SGF_ENABLE_AUDIO_PROFILER
  Profiler::Slot profilerSlots[PROFILER_SLOT_COUNT]{};
  Profiler audioProfiler;
#endif
};

}  // namespace SGFAudio
