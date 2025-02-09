#pragma once
#include "Arduino.h"
#include <core_debug.h>
#include <hc32_ddl.h>
#include "timer0_config.h"

/**
 * @brief timer0 base frequency
 * @note timer0 uses PCLK1 as base frequency. with custom config, this can be changed
 */
#define TIMER0_BASE_FREQUENCY (SYSTEM_CLOCK_FREQUENCIES.pclk1)

/**
 * @brief timer0 channel enum class
 */
enum class Timer0Channel
{
    /**
     * @brief timer0 unit channel A
     */
    A,

    /**
     * @brief timer0 unit channel B
     */
    B,
};

class Timer0
{
public:
    /**
     * @brief Construct a new Timer0 object
     * @param config pointer to timer0 peripheral configuration
     * @note Timer0 Unit 1 and 2 are not identical. Unit 1 only supports async mode, while Unit 2 supports both.
     */
    Timer0(timer0_config_t *config);

    /**
     * @brief start timer0 channel with frequency and prescaler
     * @param channel timer0 channel to start
     * @param frequency the frequency to set the timer to
     * @param prescaler the prescaler to use. must be one of [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]
     * @note compare value = (base_freq / prescaler) / frequency
     * @note Timer0 Unit 1 Channel A will use LRC clock (32KHz) instead of PCLK1 (EXPERIMENTAL, might not work properly)
     * @note this function will not automatically start the timer interrupt. call resume() to start the interrupt
     * @note if the channel is already start()-ed, this function will stop the channel first
     */
    void start(const Timer0Channel channel, const uint32_t frequency, const uint16_t prescaler = 1);

    /**
     * @brief start timer0 channel with custom channel config
     * @param channel timer0 channel to start
     * @param channel_config pointer to timer0 channel configuration
     * @note this function will not automatically start the timer interrupt. call resume() to start the interrupt
     * @note if the channel is already start()-ed, this function will stop the channel first
     */
    void start(const Timer0Channel channel, const stc_tim0_base_init_t *channel_config);

    /**
     * @brief stop timer0 channel
     * @param channel timer0 channel to stop
     * @note this function will automatically stop the timer interrupt
     * @note if the channel is not start()-ed, this function will do nothing
     * @note if all channels are stopped, the timer0 peripheral will be disabled automatically
     */
    void stop(const Timer0Channel channel);

    /**
     * @brief pause timer0 channel interrupt
     * @param channel timer0 channel to pause
     * @note if the channel is not start()-ed, this function will do nothing
     */
    void pause(const Timer0Channel channel);

    /**
     * @brief resume timer0 channel interrupt
     * @param channel timer0 channel to resume
     * @note if the channel is not start()-ed, this function will do nothing
     */
    void resume(const Timer0Channel channel);

    /**
     * @brief check if timer0 channel interrupt is currently paused
     * @param channel timer0 channel to check
     * @return true if paused
     * @note if the channel is not start()-ed, this function will return false
     */
    bool isPaused(const Timer0Channel channel);

    /**
     * @brief set timer0 channel compare value
     * @param channel timer0 channel to set compare value
     * @param compare compare value to set
     * @note the channel must be start()-ed before calling this function
     */
    void setCompareValue(const Timer0Channel channel, const uint16_t compare);

    /**
     * @brief get timer0 channel counter value
     * @param channel timer0 channel to get counter value
     * @return current counter value
     * @note the channel must be start()-ed before calling this function
     */
    uint16_t getCount(const Timer0Channel channel);

    /**
     * @brief set timer0 channel callback
     * @param channel timer0 channel to set callback for
     * @param callback callback function to set. NULL to remove callback
     */
    void setCallback(const Timer0Channel channel, voidFuncPtr callback);

    /**
     * @brief set timer0 channel callback priority
     * @param channel timer0 channel to set callback priority for
     * @param priority priority to set
     */
    void setCallbackPriority(const Timer0Channel channel, const uint32_t priority);

    /**
     * @brief remove timer0 channel callback
     * @param channel timer0 channel to remove callback for
     */
    void removeCallback(const Timer0Channel channel)
    {
        setCallback(channel, NULL);
    }

private:
    timer0_config_t *config;

    timer0_channel_state_t *get_channel_state(const Timer0Channel channel)
    {
        switch (channel)
        {
        case Timer0Channel::A:
            return &this->config->channel_a_state;
        case Timer0Channel::B:
            return &this->config->channel_b_state;
        default:
            panic("invalid timer0 channel");
            return NULL;
        }
    }
};
