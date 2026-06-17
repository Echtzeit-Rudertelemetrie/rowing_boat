#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "AppTypes.h"

class EventQueue {
public:
  EventQueue() = default;
  ~EventQueue();

  bool begin(size_t length);
  bool pushFromISR(EventType event, BaseType_t* higherPriorityTaskWoken);
  bool pop(EventType& event, TickType_t timeoutTicks);
  bool isEmpty() const;
  uint32_t droppedEvents() const;

private:
  QueueHandle_t queue_ = nullptr;
  volatile uint32_t droppedEvents_ = 0;
};