

#include "ControllerMain.h"
uint16_t adcHeatSinkTemp2SensorDataBuffer[RESULTS_BUFFER_SIZE];

EPWM_SignalParams pwmSignal =
            {100, 1.0f, 0.5f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
            EPWM_COUNTER_MODE_UP_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};

uint8_t gTimer0Var;
uint8_t gTimer1Var;
uint8_t gTimer2Var;

bool RS485_interruptReceived;
bool CM_IPC_DataReceived;

gControlStruct *gControlStruct_t;

gControlStruct gStructPtr;

gControlStruct* GetStructPtr()
{
    return (&gStructPtr);
}

void Ipc_init()
{
    //CPU1 - CM IPC 1
    IPC_registerInterrupt(IPC_CPU1_L_CM_R, IPC_INT1, CM_CPU1_IPC_ISR1);

    IPC_clearFlagLtoR(IPC_CPU1_L_CM_R, IPC_FLAG_ALL);

//    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);

    //CPU1 - CPU2 IPC1
    IPC_registerInterrupt(IPC_CPU1_L_CPU2_R, IPC_INT2, CPU2_CPU1_IPC_ISR1);

    IPC_clearFlagLtoR(IPC_CPU1_L_CPU2_R, IPC_FLAG_ALL);

}

//
// configureADC - Write ADC configurations and power up the ADC for the
// selected ADC
//
void configureADC(uint32_t adcBase)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(adcBase, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
    ADC_setMode(adcBase, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);

    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(adcBase, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADCs and then delay for 1 ms
    //
    ADC_enableConverter(adcBase);

    //
    // Delay for 1ms to allow ADC time to power up
    //
    DEVICE_DELAY_US(1000);
}
void initADCSOC_B(void)
{

    //
    // - NOTE: A longer sampling window will be required if the ADC driving
    //   source is less than ideal (an ideal source would be a high bandwidth
    //   op-amp with a small series resistance). See TI application report
    //   SPRACT6 for guidance on ADC driver design.
    // - NOTE: SOCs need not use the same S+H window duration, but SOCs
    //   occurring in parallel (in this example, SOC0 on both ADCs occur in
    //   parallel, as do the SOC1s on both ADCs, etc.) should usually
    //   use the same value to ensure simultaneous samples and synchronous
    //   operation.

    //
    // Select the channels to convert and the configure the ePWM trigger
    //
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_SW_ONLY,
                 ADC_CH_ADCIN2, 512);


    //
    // Selec SOC2 on ADCA as the interrupt source.  SOC2 on ADCC will end at
    // the same time, so either SOC2 would be an acceptable interrupt triggger.
    //
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER2, ADC_SOC_NUMBER2);
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER2);
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER2);
}
void LEDsInit()
{

    GPIO_setDirectionMode(DBG_GREEN_LED_GPIO36, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DBG_GREEN_LED_GPIO36, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(DBG_GREEN_LED_GPIO36, GPIO_CORE_CM);
    GPIO_setQualificationMode(DBG_GREEN_LED_GPIO36, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_36_GPIO36);

    GPIO_setDirectionMode(DBG_BLUE_LED_GPIO37, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DBG_BLUE_LED_GPIO37, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(DBG_BLUE_LED_GPIO37, GPIO_CORE_CPU1);
    GPIO_setQualificationMode(DBG_BLUE_LED_GPIO37, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_37_GPIO37);

    GPIO_setDirectionMode(OCP_RED_LED_GPIO38, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(OCP_RED_LED_GPIO38, GPIO_PIN_TYPE_STD);
    GPIO_setMasterCore(OCP_RED_LED_GPIO38, GPIO_CORE_CPU2);
    GPIO_setQualificationMode(OCP_RED_LED_GPIO38, GPIO_QUAL_SYNC);
    GPIO_setPinConfig(GPIO_38_GPIO38);
}
void UART_FIFO_init()
{
    //
     // Interrupts that are used in this example are re-mapped to
     // ISR functions found within this file.
     //
     Interrupt_register(INT_SCIA_RX, sciaRXFIFOISR);
     Interrupt_register(INT_SCIA_TX, sciaTXFIFOISR);

     initSCIAFIFO();

     Interrupt_enable(INT_SCIA_RX);
     Interrupt_enable(INT_SCIA_TX);
     Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

}

void UART_Init()
{
    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO85_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO85_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO84_PIN_SCITXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO84_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO84_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO84_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO84_PIN_SCITXDA, GPIO_QUAL_ASYNC);

}
//
// initSCIAFIFO - Configure SCIA FIFO
//
void initSCIAFIFO()
{
    //
    // 8 char bits, 1 stop bit, no parity. Baud rate is 9600.
    //
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 115200, (SCI_CONFIG_WLEN_8 |
                                                        SCI_CONFIG_STOP_ONE |
                                                        SCI_CONFIG_PAR_NONE));
    SCI_resetChannels(SCIA_BASE);

    SCI_enableModule(SCIA_BASE);
    SCI_enableFIFO(SCIA_BASE);

    //
    // RX and TX FIFO Interrupts Enabled
    //
    SCI_enableInterrupt(SCIA_BASE, (SCI_INT_RXFF));
    SCI_disableInterrupt(SCIA_BASE, (SCI_INT_RXERR | SCI_INT_TXFF) );

    //
    // The transmit FIFO generates an interrupt when FIFO status
    // bits are less than or equal to 2 out of 16 words
    // The receive FIFO generates an interrupt when FIFO status
    // bits are greater than equal to 2 out of 16 words
    //
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX1, SCI_FIFO_RX3);
    SCI_performSoftwareReset(SCIA_BASE);

    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
}
//
// configurePhase - Configure ePWMx Phase
//
void configurePhase(uint32_t base, uint32_t masterBase, uint16_t phaseVal)
{
    uint32_t readPrdVal, phaseRegVal;

    //
    // Read Period value to calculate value for Phase Register
    //
    readPrdVal = EPWM_getTimeBasePeriod(masterBase);

    //
    // Caluclate phase register values based on Time Base counter mode
    //
    if((HWREGH(base + EPWM_O_TBCTL) & 0x3U) == EPWM_COUNTER_MODE_UP_DOWN)
    {
        phaseRegVal = (2U * readPrdVal * phaseVal) / 360U;
    }
    else if((HWREGH(base + EPWM_O_TBCTL) & 0x3U) < EPWM_COUNTER_MODE_UP_DOWN)
    {
        phaseRegVal = (readPrdVal * phaseVal) / 360U;
    }

    EPWM_selectPeriodLoadEvent(base, EPWM_SHADOW_LOAD_MODE_SYNC);
    EPWM_setPhaseShift(base, phaseRegVal);
    EPWM_setTimeBaseCounter(base, phaseRegVal);

}

void EPWM_init()
{
    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    EPWM_configureSignal(EPWM2_BASE, &pwmSignal);


    EPWM_disablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setPhaseShift(EPWM2_BASE, 0U);

    EPWM_enableSyncOutPulseSource(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_SOFTWARE);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

//    configurePhase(EPWM2_BASE,EPWM1_BASE,120);

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_EPWM1);
}

/**
 * main.c
 */
int main(void)
{

    //
    // Initialize device clock and peripherals
    //
//    Device_init();

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);

#ifdef _FLASH
//    Device_bootCM(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCM(BOOTMODE_BOOT_TO_S0RAM);
#endif

    //
    // Boot CPU2 core
    //
#ifdef _FLASH
    Device_bootCPU2(BOOTMODE_BOOT_TO_FLASH_SECTOR0);
#else
    Device_bootCPU2(BOOTMODE_BOOT_TO_M0RAM);
#endif
    DINT;

//    MemCfg_setGSRAMMasterSel( (MEMCFG_SECT_GS6 | MEMCFG_SECT_GS7 | MEMCFG_SECT_GS8 | MEMCFG_SECT_GS9 | MEMCFG_SECT_GS10  ),MEMCFG_GSRAMMASTER_CPU2);
    UART_Init();

    //
    // Disable pin locks and enable internal pullups.
    //
//    Device_initGPIO();
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    ControllerGpioInit_CPU1();
    ControllerGpioInit_CPU2();

    LEDsInit();

    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 1, SYSCTL_CPUSEL_CPU2 );
//    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL11_ADC, 2, SYSCTL_CPUSEL_CPU2 );

//    SysCtl_lockCPUSelectRegs(SYSCTL_CPUSEL14_DAC);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL14_DAC, 2, SYSCTL_CPUSEL_CPU2 );
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL14_DAC, 1, SYSCTL_CPUSEL_CPU2 );

    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL7_I2C, 1, SYSCTL_CPUSEL_CPU2 );
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL7_I2C, 2, SYSCTL_CPUSEL_CPU2 );

    //
    // Initialize the SPI
    //
    initSPI();

    configureADC(ADCB_BASE);
    initADCSOC_B();

    //configures the GPIOs and Allocates the shared peripherals
    CM_EthernetIOMuxHandler();

    Ipc_init();

//    IPC_sync(IPC_CPU1_L_CM_R, IPC_FLAG31);

    INIT_CPU_TIMERS();//Initialize only CPU0 and CPU1 because CPU2 is being reserved for RTOS Idle task operation, Ensure no CPU2 timer will be created

    setupMsgTimer0();

    setupMsgTimer1();

    setupMsgTimer2();

    InitFWInstances();

    UART_FIFO_init();

    //
    // Enables CPU interrupts
    //
    EINT;
    ERTM;

    lcd_init();
    grllcddisplay(1, "Booting...."); 
    GPIO_writePin(FAN1_GPIO3, 1);
    GPIO_writePin(FAN2_GPIO4, 1);

    EPWM_init();

    GPIO_writePin(FAN1_PWM_GPIO2, 1);
    GPIO_writePin(FAN2_PWM_GPIO5, 1);



    while(1)
    {
//        GPIO_writePin(DBG_BLUE_LED_GPIO37, 0);
//
////        if(RS485_interruptReceived)
////        {
////            RS485_interruptReceived = false;
////            grlRs485Rx_CmTx_DataHandler();
////        }
////        if(CM_IPC_DataReceived)
////        {
////            CM_IPC_DataReceived = false;
////            RecvDataHandler(CmDataRxbuf);
////        }
//        DEVICE_DELAY_US(100000);
//        GPIO_writePin(DBG_BLUE_LED_GPIO37, 1);
//        DEVICE_DELAY_US(100000);


    }

}
