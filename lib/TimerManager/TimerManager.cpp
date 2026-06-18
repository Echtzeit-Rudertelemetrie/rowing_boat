#include "TimerManager.h"

EventQueue* TimerManager::queue_ = nullptr;

TimerManager::TimerManager(EventQueue& queue)
{
    queue_ = &queue;
}

bool TimerManager::begin(uint32_t frequencyHz)
{
    // Timer 0
    // Prescaler 80 -> 80 MHz / 80 = 1 MHz
    // 1 Tick = 1 µs

    timer_ = timerBegin(0, 80, true); //TODO: Check if this is 80 for ESP Wroom or 40 for ESP Mini S3

    if (timer_ == nullptr)
        return false;

    timerAttachInterrupt(timer_, &TimerManager::onTimer, true);

    const uint32_t periodUs = 1000000UL / frequencyHz;

    timerAlarmWrite(timer_, periodUs, true);
    timerAlarmEnable(timer_);

    return true;
}

void TimerManager::end()
{
    if (timer_ != nullptr)
    {
        timerAlarmDisable(timer_);
        timerEnd(timer_);
        timer_ = nullptr;
    }
}

void IRAM_ATTR TimerManager::onTimer()
{
    if (queue_ == nullptr)
        return;

    BaseType_t higherPriorityTaskWoken = pdFALSE;

    queue_->pushFromISR(
        EventType::ReadSensor1,
        &higherPriorityTaskWoken);

    queue_->pushFromISR(
        EventType::ReadSensor2,
        &higherPriorityTaskWoken);

    queue_->pushFromISR(
        EventType::SendData,
        &higherPriorityTaskWoken);

    if (higherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}