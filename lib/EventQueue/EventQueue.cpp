#include "EventQueue.h"

EventQueue::~EventQueue() {
  if (queue_ != nullptr) {
    vQueueDelete(queue_);
    queue_ = nullptr;
  }
}

bool EventQueue::begin(size_t length) {
  if (length == 0) {
    return false;
  }

  queue_ = xQueueCreate(length, sizeof(EventType));
  return queue_ != nullptr;
}

bool EventQueue::pushFromISR(EventType event, BaseType_t* higherPriorityTaskWoken) {
  if (queue_ == nullptr) {
    return false;
  }

  if (xQueueSendFromISR(queue_, &event, higherPriorityTaskWoken) != pdTRUE) {
    ++droppedEvents_;
    return false;
  }

  return true;
}

bool EventQueue::pop(EventType& event, TickType_t timeoutTicks) {
  if (queue_ == nullptr) {
    return false;
  }

  return xQueueReceive(queue_, &event, timeoutTicks) == pdTRUE;
}

bool EventQueue::isEmpty() const {
  if (queue_ == nullptr) {
    return true;
  }

  return uxQueueMessagesWaiting(queue_) == 0;
}

uint32_t EventQueue::droppedEvents() const {
  return droppedEvents_;
}