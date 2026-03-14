#include "ESP32DacSynthOutput.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if SGF_ENABLE_AUDIO_PROFILER
#include "SGF/SerialMonitor.h"
#endif

namespace SGFAudio {

ESP32DacSynthOutput::ESP32DacSynthOutput(IAudioSource& sourceRef, uint8_t pinRef)
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

ESP32DacSynthOutput::~ESP32DacSynthOutput() {
  end();
}

bool ESP32DacSynthOutput::begin() {
  if (taskHandle != nullptr || timer != nullptr) {
    return true;
  }
  readIndex = 0u;
  writeIndex = 0u;
  dacWrite(pin, 128u);
  timer = timerBegin(source.sampleRate());
  if (timer == nullptr) {
    return false;
  }
  timerAttachInterruptArg(timer, &ESP32DacSynthOutput::onTimerThunk, this);
  timerAlarm(timer, 1u, true, 0u);

  running = true;
  TaskHandle_t handle = nullptr;
  BaseType_t result = xTaskCreatePinnedToCore(
    &ESP32DacSynthOutput::audioTaskThunk,
    "sgf_audio",
    4096,
    this,
    1,
    &handle,
    0);
  if (result != pdPASS || handle == nullptr) {
    running = false;
    timerEnd(timer);
    timer = nullptr;
    taskHandle = nullptr;
    return false;
  }
  taskHandle = handle;
  return true;
}

bool ESP32DacSynthOutput::attachSerialMonitor(SerialMonitor& serialMonitor) {
#if SGF_ENABLE_AUDIO_PROFILER
  return serialMonitor.attachProfiler(audioProfiler);
#else
  (void)serialMonitor;
  return false;
#endif
}

void ESP32DacSynthOutput::end() {
  if (taskHandle == nullptr && timer == nullptr) {
    return;
  }
  running = false;
  while (taskHandle != nullptr) {
    delay(1);
  }
  if (timer != nullptr) {
    timerEnd(timer);
    timer = nullptr;
  }
  dacWrite(pin, 128u);
}

void ESP32DacSynthOutput::audioTaskThunk(void* arg) {
  if (arg == nullptr) {
    vTaskDelete(nullptr);
    return;
  }
  static_cast<ESP32DacSynthOutput*>(arg)->audioTaskLoop();
}

void ESP32DacSynthOutput::onTimerThunk(void* arg) {
  if (arg == nullptr) {
    return;
  }
  static_cast<ESP32DacSynthOutput*>(arg)->onTimer();
}

void ESP32DacSynthOutput::audioTaskLoop() {
#if SGF_ENABLE_AUDIO_PROFILER
  uint32_t lastUnderrunCount = 0u;
  uint32_t lastOverrunCount = 0u;
#endif
  while (running) {
    if (ringFree() >= RING_FILL_CHUNK) {
#if SGF_ENABLE_AUDIO_PROFILER
      uint32_t fillStartUs = micros();
#endif
      for (uint16_t i = 0; i < RING_FILL_CHUNK; ++i) {
        const int16_t sample = source.renderSample();
        const uint8_t dacValue = static_cast<uint8_t>((static_cast<int32_t>(sample) + 32768) >> 8);
        pushSample(dacValue);
      }
#if SGF_ENABLE_AUDIO_PROFILER
      audioProfiler.increment(FillCallsSlot);
      audioProfiler.probe(FillTimeSlot, micros() - fillStartUs);
      audioProfiler.probe(RingUsedSlot, ringUsed());
      uint32_t currentUnderruns = underrunCount;
      if (currentUnderruns > lastUnderrunCount) {
        audioProfiler.increment(UnderrunSlot, currentUnderruns - lastUnderrunCount);
        lastUnderrunCount = currentUnderruns;
      }
      uint32_t currentOverruns = overrunCount;
      if (currentOverruns > lastOverrunCount) {
        audioProfiler.increment(OverrunSlot, currentOverruns - lastOverrunCount);
        lastOverrunCount = currentOverruns;
      }
#endif
    } else {
      vTaskDelay(1);
    }
  }
  taskHandle = nullptr;
  vTaskDelete(nullptr);
}

void ESP32DacSynthOutput::onTimer() {
  dacWrite(pin, popSample());
}

uint16_t ESP32DacSynthOutput::ringUsed() const {
  const uint16_t r = readIndex;
  const uint16_t w = writeIndex;
  if (w >= r) {
    return static_cast<uint16_t>(w - r);
  }
  return static_cast<uint16_t>(RING_BUFFER_SIZE - (r - w));
}

uint16_t ESP32DacSynthOutput::ringFree() const {
  return static_cast<uint16_t>((RING_BUFFER_SIZE - 1u) - ringUsed());
}

void ESP32DacSynthOutput::pushSample(uint8_t sample) {
  const uint16_t next = static_cast<uint16_t>((writeIndex + 1u) % RING_BUFFER_SIZE);
  if (next == readIndex) {
    overrunCount++;
    return;
  }
  ringBuffer[writeIndex] = sample;
  writeIndex = next;
}

uint8_t ESP32DacSynthOutput::popSample() {
  if (readIndex == writeIndex) {
    underrunCount++;
    return 128u;
  }
  const uint8_t sample = ringBuffer[readIndex];
  readIndex = static_cast<uint16_t>((readIndex + 1u) % RING_BUFFER_SIZE);
  return sample;
}

}  // namespace SGFAudio
