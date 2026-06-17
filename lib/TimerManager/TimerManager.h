#pragma once

#include <Arduino.h>
#include "EventQueue.h"

class TimerManager {
public:
    explicit TimerManager(EventQueue& queue);

    bool begin(uint32_t frequencyHz = 200);
    void end();

private:
    static void IRAM_ATTR onTimer();

    static EventQueue* queue_;

    hw_timer_t* timer_ = nullptr;
};