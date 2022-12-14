//###########################################################################
//
// FILE:   dcc.c
//
// TITLE:  C28x DCC driver.
//
//###########################################################################
// $Copyright:  $
//###########################################################################

#include "dcc.h"


//*****************************************************************************
//
// DCC_verifyClockFrequency
//
//*****************************************************************************
bool
DCC_verifyClockFrequency(uint32_t base,
                         DCC_Count1ClockSource clock1,
                         float32_t freq1,
                         DCC_Count0ClockSource clock0,
                         float32_t freq0,
                         float32_t tolerance,
                         float32_t freqerr,
                         float32_t freq_sysclk)
{
    uint32_t total_error;
    uint32_t window;
    uint32_t count0;
    uint32_t valid;
    uint32_t count1;

    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // If Fclk1 > Fclk0,
    //    Async. Error (In Clock0 cycles) = 2 + 2*(Fsysclk/Fclk0)
    //
    // If Fclk1 < Fclk0,
    //    Async. Error (In Clock0 cycles) = 2*(Fclk0/Fclk1) + 2*(Fsysclk/Fclk0)
    //
    if(freq1 > freq0)
    {
        total_error = (uint32_t)(2.0F + 2.0F * (freq_sysclk / freq0));
    }
    else
    {
        total_error = (uint32_t)(2.0F * (freq0 / freq1) +
                                 2.0F * (freq_sysclk / freq0));
    }

    //
    // Digitization error = 8 Clock0 cycles
    //
    total_error += 8U;

    //
    // Window (in Cycles) = (Total Error) / (0.01 * Tolerance)
    //
    window = (float32_t)total_error / (0.01F * tolerance);

    //
    // Error due to variance in clock frequency =
    // window * (Allowable Frequency Tolerance (in %) / 100)
    //
    total_error += (float32_t)window * (freqerr / 100.0F);

    //
    // DCC counter configuration :
    // Counter0 = Window - Total Error
    // Valid0 = 2 * Total Error
    // Counter1 = Window * (Fclk1 / Fclk0)
    //
    count0 = window - total_error;
    valid  = 2 * total_error;
    count1 = window * freq1 / freq0;

    //
    // Clear Error & Done Flag
    //
    DCC_clearErrorFlag(base);
    DCC_clearDoneFlag(base);

    //
    // Disable DCC and the error and done signals
    //
    DCC_disableModule(base);
    DCC_disableErrorSignal(base);
    DCC_disableDoneSignal(base);

    //
    // Configure Clock Source0 and clock source1
    //
    DCC_setCounter0ClkSource(base, clock0);
    DCC_setCounter1ClkSource(base, clock1);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(base, count0, valid, count1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(base, DCC_MODE_COUNTER_ZERO);

    //
    // Enable Error Signal and Done signal
    //
    DCC_enableErrorSignal(base);
    DCC_enableDoneSignal(base);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(base);

    //
    // Wait until Error or Done Flag is generated
    //
    while((DCC_getSingleShotStatus(base) | DCC_getErrorStatus(base)) == 0);

    //
    // Returns true if DCC completes without error
    //
    if(DCC_getSingleShotStatus(base) == 1U)
    {
        return(true);
    }
    else
    {
        return(false);
    }
}

//*****************************************************************************
//
// DCC_measureClockFrequency
//
//*****************************************************************************
float32_t
DCC_measureClockFrequency(uint32_t base,
                          DCC_Count1ClockSource clock1,
                          DCC_Count0ClockSource clock0,
                          float32_t freq0,
                          float32_t tolerance,
                          float32_t freqerr,
                          float32_t freq_sysclk)
{
    uint32_t total_error;
    uint32_t window;
    uint32_t count0;
    uint32_t valid;
    uint32_t count1;
    uint32_t actual_cnt1;
    float32_t freq1;

    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // Async. Error = 2 + 2*(Fsysclk/Fclk0)
    // Digitization error = 8 Clock0 cycles
    // Total DCC error = Async Error + Digitization error
    //
    total_error = (uint32_t)(2.0F + 2.0F * (freq_sysclk / freq0));
    total_error += 8U;

    //
    // Window (in Cycles) = (Total Error) / (0.01 * Tolerance)
    //
    window = total_error / (0.01F * tolerance);

    //
    // Error due to variance in clock frequency =
    // window * (Allowable Frequency Tolerance (in %) / 100)
    //
    total_error += (float32_t)window * (freqerr / 100.0F);

    //
    // DCC counter configuration :
    // Counter0 = Window - Total Error
    // Valid0 = 2 * Total Error
    // Counter1 = Maximum counter value (0xFFFFF)
    //
    count0 = window - total_error;
    valid  = 2 * total_error;
    count1 = 0xFFFFFU;

    //
    // Clear Error & Done Flag
    //
    DCC_clearErrorFlag(base);
    DCC_clearDoneFlag(base);

    //
    // Disable DCC and the error and done signals
    //
    DCC_disableModule(base);
    DCC_disableErrorSignal(base);
    DCC_disableDoneSignal(base);

    //
    // Configure Clock Source0 and clock source1
    //
    DCC_setCounter0ClkSource(base, clock0);
    DCC_setCounter1ClkSource(base, clock1);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(base, count0, valid, count1);

    //
    // Enable Single Shot mode
    //
    DCC_enableSingleShotMode(base, DCC_MODE_COUNTER_ZERO);

    //
    // Enable Error Signal and Done signal
    //
    DCC_enableErrorSignal(base);
    DCC_enableDoneSignal(base);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(base);

    //
    // Wait until Error or Done Flag is generated
    //
    while((DCC_getSingleShotStatus(base) | DCC_getErrorStatus(base)) == 0);

    //
    // Calculate the difference of the current counter
    // values with the initial fed counter values. This
    // will give the counts corresponding to the frequency
    // of each clock source
    //
    actual_cnt1 = count1 - DCC_getCounter1Value(base);

    //
    // Compute the frequency using relation F1 = (F0 * Actual C1)/Window.
    //
    freq1 =  (freq0 * actual_cnt1) / (count0 + valid);

    return(freq1);
}

//*****************************************************************************
//
// DCC_continuousMonitor
//
//*****************************************************************************
void
DCC_continuousMonitor(uint32_t base,
                      DCC_Count1ClockSource clock1,
                      float32_t freq1,
                      DCC_Count0ClockSource clock0,
                      float32_t freq0,
                      float32_t tolerance,
                      float32_t freqerr,
                      float32_t freq_sysclk)
{
    uint32_t total_error;
    uint32_t window;
    uint32_t count0;
    uint32_t valid;
    uint32_t count1;

    //
    // Check the arguments.
    //
    ASSERT(DCC_isBaseValid(base));

    //
    // If Fclk1 > Fclk0,
    //     Async. Error (In Clock0 cycles) = 2 + 2*(Fsysclk/Fclk0)
    //
    // If Fclk1 < Fclk0,
    //     Async. Error (In Clock0 cycles) = 2*(Fclk0/Fclk1) + 2*(Fsysclk/Fclk0)
    //
    if(freq1 > freq0)
    {
        total_error = (uint32_t)(2.0F + 2.0F * (freq_sysclk / freq0));
    }
    else
    {
        total_error = (uint32_t)(2.0F * (freq0 / freq1) +
                                 2.0F * (freq_sysclk / freq0));
    }

    //
    // Digitization error = 8 Clock0 cycles
    //
    total_error += 8U;

    //
    // Window (in Cycles) = (Total Error) / (0.01 * Tolerance)
    //
    window = (float32_t)total_error / (0.01F * tolerance);

    //
    // Error due to variance in clock frequency =
    // window * (Allowable Frequency Tolerance (in %) / 100)
    //
    total_error += (float32_t)window * (freqerr / 100.0F);

    //
    // DCC counter configuration :
    // Counter0 = Window - Total Error
    // Valid0 = 2 * Total Error
    // Counter1 = Window * (Fclk1 / Fclk0)
    //
    count0 = window - total_error;
    valid  = 2 * total_error;
    count1 = window * freq1 / freq0;

    //
    // Clear Error & Done Flag
    //
    DCC_clearErrorFlag(base);
    DCC_clearDoneFlag(base);

    //
    // Disable DCC and the error and done signals
    //
    DCC_disableModule(base);
    DCC_disableErrorSignal(base);
    DCC_disableDoneSignal(base);

    //
    // Configure Clock Source0 and clock source1
    //
    DCC_setCounter0ClkSource(base, clock0);
    DCC_setCounter1ClkSource(base, clock1);

    //
    // Configure COUNTER-0, COUNTER-1 & Valid Window
    //
    DCC_setCounterSeeds(base, count0, valid, count1);

    //
    // Enable Continuous mode
    //
    DCC_disableSingleShotMode(base);

    //
    // Enable Error Signal
    //
    DCC_enableErrorSignal(base);

    //
    // Enable DCC to start counting
    //
    DCC_enableModule(base);
}
