//###########################################################################
//
// FILE:   enet_lwip.c
//
// TITLE:  lwIP based Ethernet Example.
//
// Example to demonstrate UDP socket (for daikin customer)
// buf_rx,buf_tx are the watch variables which can be used or updated in the
// main application based on the requirement.
//###########################################################################
// $TI Release:   $
// $Release Date:   $
// $Copyright:
// Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.co
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################


#include <GrlPDComm.h>
//#include <dhcp.h>

static int g_tcp_active_cnt = 0;    /* tcp activity status to close the in-active connection */
uint16_t cont_rx_udp;
extern uint8_t  gTimer0Var;
extern uint8_t  gTimer1Var;
extern uint32_t IP_address;

#pragma DATA_SECTION(buf_rx, "MSGRAM_CM_TO_CPU1")

uint16_t PollingRxBuf[128];

uint16_t CPU1RxBuf[CPU1_RXBUFSIZE];
uint16_t CPU2RxBuf[CPU2_RXBUFSIZE];

u8_t buf_rx[PAYLOAD];
u8_t buf_tx[PAYLOAD];

volatile bool flag_TX_frame_UDP=true;

#define MAKE_IP_ADDRESS(a0,a1,a2,a3) (((a0<<24) & 0xFF000000) | ((a1<<16) & 0x00FF0000) | ((a2<<8)  & 0x0000FF00) | (a3 & 0x000000FF) )

bool Connected_udp_28000 = false;

uint16_t cnt_Connected_udp_28000 = 0;
//
// Defines
//
#define PACKET_LENGTH 132

struct udp_pcb *g_upcb;
struct tcp_pcb *tcp_echoserver_pcb;
struct tcp_pcb *tcp_echoserver_pcb_5003;

/* ECHO protocol states */
enum tcp_echoserver_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

/* structure for maintaing connection infos to be passed as argument
   to LwIP callbacks*/
struct tcp_echoserver_struct
{
  u8_t state;             /* current connection state */
  struct tcp_pcb *pcb;    /* pointer on the current tcp_pcb */
  struct pbuf *p;         /* pointer on the received/to be transmitted pbuf */
};


uint32_t IPAddr_dhcp = 0xffffffff;

// These are defined by the linker (see device linker command file)
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;

extern uint16_t constLoadStart;
extern uint16_t constLoadEnd;
extern uint16_t constLoadSize;
extern uint16_t constRunStart;
extern uint16_t constRunEnd;
extern uint16_t constRunSize;

#define DEVICE_FLASH_WAITSTATES 2

//*****************************************************************************
//
// Driver specific initialization code and macro.
//
//*****************************************************************************

#define ETHERNET_NO_OF_RX_PACKETS   2U
#define ETHERNET_MAX_PACKET_LENGTH 1538U
//#define ETHERNET_MAX_PACKET_LENGTH 16384U//grl-edit

#define NUM_PACKET_DESC_RX_APPLICATION PBUF_POOL_SIZE //8 - same as PBUF_POOL_SIZE


Ethernet_Handle emac_handle;
Ethernet_InitConfig *pInitCfg;
uint32_t Ethernet_numRxCallbackCustom = 0;
uint32_t releaseTxCount = 0;
uint32_t genericISRCustomcount = 0;
uint32_t genericISRCustomRBUcount = 0;
uint32_t genericISRCustomROVcount = 0;
uint32_t genericISRCustomRIcount = 0;

uint32_t systickPeriodValue = 125000; //15000000;
Ethernet_Pkt_Desc  pktDescriptorRXCustom[NUM_PACKET_DESC_RX_APPLICATION];
extern uint32_t Ethernet_numGetPacketBufferCallback;
extern Ethernet_Device Ethernet_device_struct;
uint8_t Ethernet_rxBuffer[ETHERNET_NO_OF_RX_PACKETS *
                          ETHERNET_MAX_PACKET_LENGTH];

uint32_t sendPacketFailedCount = 0;

uint8_t mac_custom[6] = {0xA8, 0x63, 0xF2, 0x00, 0x1D, 0x98};

extern Ethernet_Pkt_Desc*
lwIPEthernetIntHandler(Ethernet_Pkt_Desc *pPacket);

void CM_init(void)
{
    //
    // Disable the watchdog
    //
//    SysCtl_disableWatchdog();

#ifdef _FLASH
    //
    // Copy time critical code and flash setup code to RAM. This includes the
    // following functions: InitFlash();
    //
    // The RamfuncsLoadStart, RamfuncsLoadSize, and RamfuncsRunStart symbols
    // are created by the linker. Refer to the device .cmd file.
    // Html pages are also being copied from flash to ram.
    //
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
    memcpy(&constRunStart, &constLoadStart, (size_t)&constLoadSize);
    //
    // Call Flash Initialization to setup flash waitstates. This function must
    // reside in RAM.
    //
//    Flash_initModule(FLASH0CTRL_BASE, FLASH0ECC_BASE, DEVICE_FLASH_WAITSTATES);
#endif

    //
    // Sets the NVIC vector table offset address.
    //
#ifdef _FLASH
    Interrupt_setVectorTableOffset((uint32_t)vectorTableFlash);
#else
    Interrupt_setVectorTableOffset((uint32_t)vectorTableRAM);
#endif

}
//*****************************************************************************
//
// HTTP Webserver related callbacks and definitions.
//
//*****************************************************************************
//
// Currently, this implemented as a pointer to function which is called when
// corresponding query is received by the HTTP webserver daemon. When more
// features are needed to be added, it should be implemented as a separate
// interface.
//
void httpLEDToggle(void);
void(*ledtoggleFuncPtr)(void) = &httpLEDToggle;

//*****************************************************************************
//
// The interrupt handler for the SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Call the lwIP timer handler.
    //
 //   lwIPTimer(systickPeriodValue);
	   lwIPTimer(1);
}

//*****************************************************************************
//
//  This function is a callback function called by the example to
//  get a Packet Buffer. Has to return a ETHERNET_Pkt_Desc Structure.
//  Rewrite this API for custom use case.
//
//*****************************************************************************
Ethernet_Pkt_Desc* Ethernet_getPacketBufferCustom(void)
{
    //
    // Get the next packet descriptor from the descriptor pool
    //
    uint32_t shortIndex = (Ethernet_numGetPacketBufferCallback + 3)
                % NUM_PACKET_DESC_RX_APPLICATION;

    //
    // Increment the book-keeping pointer which acts as a head pointer
    // to the circular array of packet descriptor pool.
    //
    Ethernet_numGetPacketBufferCallback++;

    //
    // Update buffer length information to the newly procured packet
    // descriptor.
    //
    pktDescriptorRXCustom[shortIndex].bufferLength =
                                  ETHERNET_MAX_PACKET_LENGTH;

    //
    // Update the receive buffer address in the packer descriptor.
    //
    pktDescriptorRXCustom[shortIndex].dataBuffer =
                                      &Ethernet_device_struct.rxBuffer [ \
               (ETHERNET_MAX_PACKET_LENGTH*Ethernet_device_struct.rxBuffIndex)];

    //
    // Update the receive buffer pool index.
    //
    Ethernet_device_struct.rxBuffIndex += 1U;
    Ethernet_device_struct.rxBuffIndex  = \
    (Ethernet_device_struct.rxBuffIndex%ETHERNET_NO_OF_RX_PACKETS);

    //
    // Receive buffer is usable from Address 0
    //
    pktDescriptorRXCustom[shortIndex].dataOffset = 0U;

    //
    // Return this new descriptor to the driver.
    //
    return (&(pktDescriptorRXCustom[shortIndex]));
}

//*****************************************************************************
//
//  This is a hook function and called by the driver when it receives a
//  packet. Application is expected to replenish the buffer after consuming it.
//  Has to return a ETHERNET_Pkt_Desc Structure.
//  Rewrite this API for custom use case.
//
//*****************************************************************************
Ethernet_Pkt_Desc* Ethernet_receivePacketCallbackCustom(
        Ethernet_Handle handleApplication,
        Ethernet_Pkt_Desc *pPacket)
{

	Ethernet_Pkt_Desc* temp_eth_pkt;
    //
    // Book-keeping to maintain number of callbacks received.
    //
#ifdef ETHERNET_DEBUG
    Ethernet_numRxCallbackCustom++;
#endif



      Ethernet_disableRxDMAReception(EMAC_BASE,0);


    //
    // This is a placeholder for Application specific handling
    // We are replenishing the buffer received with another buffer
    //
  //  return lwIPEthernetIntHandler(pPacket);

      temp_eth_pkt=lwIPEthernetIntHandler(pPacket);


      Ethernet_enableRxDMAReception(EMAC_BASE,0);

      return temp_eth_pkt;
}

void Ethernet_releaseTxPacketBufferCustom(
        Ethernet_Handle handleApplication,
        Ethernet_Pkt_Desc *pPacket)
{
    //
    // Once the packet is sent, reuse the packet memory to avoid
    // memory leaks. Call this interrupt handler function which will take care
    // of freeing the memory used by the packet descriptor.
    //
    lwIPEthernetIntHandler(pPacket);

    //
    // Increment the book-keeping counter.
    //
#ifdef ETHERNET_DEBUG
    releaseTxCount++;
#endif
}

Ethernet_Pkt_Desc *Ethernet_performPopOnPacketQueueCustom(
            Ethernet_PKT_Queue_T *pktQueuePtr)
{
    Ethernet_Pkt_Desc *pktDescHdrPtr;

    pktDescHdrPtr = pktQueuePtr->head;

    if(0U != pktDescHdrPtr)
    {
        pktQueuePtr->head = pktDescHdrPtr->nextPacketDesc;
        pktQueuePtr->count--;
    }

    return(pktDescHdrPtr);
}
void Ethernet_performPushOnPacketQueueCustom(
        Ethernet_PKT_Queue_T *pktQueuePtr,
        Ethernet_Pkt_Desc *pktDescHdrPtr)
{
    pktDescHdrPtr->nextPacketDesc = 0U;

    if(0U == pktQueuePtr->head)
    {
        //
        // Queue is empty - Initialize it with this one packet
        //
        pktQueuePtr->head = pktDescHdrPtr;
        pktQueuePtr->tail = pktDescHdrPtr;
    }
    else
    {
        //
        // Queue is not empty - Push onto END
        //
        pktQueuePtr->tail->nextPacketDesc = pktDescHdrPtr;
        pktQueuePtr->tail        = pktDescHdrPtr;
    }
    pktQueuePtr->count++;
}
void Ethernet_setMACConfigurationCustom(uint32_t base, uint32_t flags)
{
    HWREG(base + ETHERNET_O_MAC_CONFIGURATION) |= flags;
}
void Ethernet_clearMACConfigurationCustom(uint32_t base, uint32_t flags)
{
    HWREG(base + ETHERNET_O_MAC_CONFIGURATION) &= ~flags;

}

interrupt void Ethernet_genericISRCustom(void)
{
    genericISRCustomcount++;
    Ethernet_RxChDesc *rxChan;
    Ethernet_TxChDesc *txChan;
    Ethernet_HW_descriptor    *descPtr;
    Ethernet_HW_descriptor    *tailPtr;
    uint16_t i=0;
    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_TE);
    for(i = 0U;i < Ethernet_device_struct.initConfig.numChannels;i++)
     {
         Ethernet_disableRxDMAReception(
               Ethernet_device_struct.baseAddresses.enet_base,
               i);
     }
    if(((ETHERNET_DMA_CH0_STATUS_AIS |
                         ETHERNET_DMA_CH0_STATUS_RBU) ==
                       (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                              ETHERNET_O_DMA_CH0_STATUS) &
                              (uint32_t)(ETHERNET_DMA_CH0_STATUS_AIS |
                                         ETHERNET_DMA_CH0_STATUS_RBU))) ||
          (ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS) ==
                                   (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                          ETHERNET_O_MTL_Q0_INTERRUPT_CONTROL_STATUS) &
                                          (uint32_t)(ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS
                                                     )))
      {
          if((ETHERNET_DMA_CH0_STATUS_AIS |
                             ETHERNET_DMA_CH0_STATUS_RBU) ==
                           (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                  ETHERNET_O_DMA_CH0_STATUS) &
                                  (uint32_t)(ETHERNET_DMA_CH0_STATUS_AIS |
                                             ETHERNET_DMA_CH0_STATUS_RBU)))
          {
          genericISRCustomRBUcount++;
          }
          if((ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS) ==
                  (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                         ETHERNET_O_MTL_Q0_INTERRUPT_CONTROL_STATUS) &
                         (uint32_t)(ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS
                                    )))
          {
              genericISRCustomROVcount++;
              Ethernet_enableMTLInterrupt(Ethernet_device_struct.baseAddresses.enet_base,0,
                                          ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOVFIS);
          }

        /*
             * Clear the AIS and RBU status bit. These MUST be
             * cleared together!
             */
            Ethernet_clearDMAChannelInterrupt(
                    Ethernet_device_struct.baseAddresses.enet_base,
                    ETHERNET_DMA_CHANNEL_NUM_0,
                    ETHERNET_DMA_CH0_STATUS_AIS |
                    ETHERNET_DMA_CH0_STATUS_RBU);

            /*
           *Recover from Receive Buffer Unavailable (and hung DMA)
         *
         * All descriptor buffers are owned by the application, and
         * in result the DMA cannot transfer incoming frames to the
         * buffers (RBU condition). DMA has also entered suspend
         * mode at this point, too.
         *
         * Drain the RX queues
         */

            /* Upon RBU error, discard all previously received packets */
            if(Ethernet_device_struct.initConfig.pfcbDeletePackets != NULL)
                (*Ethernet_device_struct.initConfig.pfcbDeletePackets)();

            rxChan =
               &Ethernet_device_struct.dmaObj.rxDma[ETHERNET_DMA_CHANNEL_NUM_0];
            txChan=
               &Ethernet_device_struct.dmaObj.txDma[ETHERNET_DMA_CHANNEL_NUM_0];

    /*
     * Need to disable multiple interrupts, so protect the code to do so within
     * a global disable block (to prevent getting interrupted in between)
     */

            if(NULL!= Ethernet_device_struct.ptrPlatformInterruptDisable)
            {
                (*Ethernet_device_struct.ptrPlatformInterruptDisable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_RX_INTR_CH0 + rxChan->chInfo->chNum]);

                (*Ethernet_device_struct.ptrPlatformInterruptDisable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_GENERIC_INTERRUPT]);
            }
            /* verify we have full capacity in the descriptor queue */
            if(rxChan->descQueue.count < rxChan->descMax) {
              /* The queue is not at full capacity due to OOM errors.
              Try to fill it again */
                Ethernet_addPacketsIntoRxQueue(rxChan);
            }
            Ethernet_initRxChannel(
                    &Ethernet_device_struct.initConfig.chInfo[ETHERNET_CH_DIR_RX][0]);

            Ethernet_writeRxDescTailPointer(
                Ethernet_device_struct.baseAddresses.enet_base,
                0,
                (&Ethernet_device_struct.rxDesc[
                 ((uint32_t)ETHERNET_DESCRIPTORS_NUM_RX_PER_CHANNEL) *
                  (0 + (uint32_t)1U)]));

            if(NULL!= Ethernet_device_struct.ptrPlatformInterruptEnable)
            {
                (*Ethernet_device_struct.ptrPlatformInterruptEnable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_RX_INTR_CH0 + rxChan->chInfo->chNum]);
                (*Ethernet_device_struct.ptrPlatformInterruptEnable)(
                    Ethernet_device_struct.interruptNum[
                        ETHERNET_GENERIC_INTERRUPT]);
            }


    }
    if(0U != (HWREG(Ethernet_device_struct.baseAddresses.enet_base +
                                 ETHERNET_O_DMA_CH0_STATUS) &
                           (uint32_t) ETHERNET_DMA_CH0_STATUS_RI))
    {
        genericISRCustomRIcount++;
        Ethernet_clearDMAChannelInterrupt(
                        Ethernet_device_struct.baseAddresses.enet_base,
                        ETHERNET_DMA_CHANNEL_NUM_0,
                        ETHERNET_DMA_CH0_STATUS_NIS | ETHERNET_DMA_CH0_STATUS_RI);
    }

    for(i = 0U;i < Ethernet_device_struct.initConfig.numChannels;i++)
     {
         Ethernet_enableRxDMAReception(
               Ethernet_device_struct.baseAddresses.enet_base,
               i);
     }
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_TE);
}

void
Ethernet_init(const unsigned char *mac)
{
    Ethernet_InitInterfaceConfig initInterfaceConfig;
    uint32_t macLower;
    uint32_t macHigher;
    uint8_t *temp;

    initInterfaceConfig.ssbase = EMAC_SS_BASE;
    initInterfaceConfig.enet_base = EMAC_BASE;
    initInterfaceConfig.phyMode = ETHERNET_SS_PHY_INTF_SEL_MII;


    //
    // Assign SoC specific functions for Enabling,Disabling interrupts
    // and for enabling the Peripheral at system level
    //
    initInterfaceConfig.ptrPlatformInterruptDisable =
                                                    &Platform_disableInterrupt;
    initInterfaceConfig.ptrPlatformInterruptEnable =
                                                     &Platform_enableInterrupt;
    initInterfaceConfig.ptrPlatformPeripheralEnable =
                                                    &Platform_enablePeripheral;
    initInterfaceConfig.ptrPlatformPeripheralReset =
                                                     &Platform_resetPeripheral;

    //
    // Assign the peripheral number at the SoC
    //
    initInterfaceConfig.peripheralNum = SYSCTL_PERIPH_CLK_ENET;

    //
    // Assign the default SoC specific interrupt numbers of Ethernet interrupts
    //
    initInterfaceConfig.interruptNum[0] = INT_EMAC;
    initInterfaceConfig.interruptNum[1] = INT_EMAC_TX0;
    initInterfaceConfig.interruptNum[2] = INT_EMAC_TX1;
    initInterfaceConfig.interruptNum[3] = INT_EMAC_RX0;
    initInterfaceConfig.interruptNum[4] = INT_EMAC_RX1;

    //Grl-Edit
//    initInterfaceConfig.clockSel = ETHERNET_SS_CLK_SRC_EXTERNAL;

    pInitCfg = Ethernet_initInterface(initInterfaceConfig);

    Ethernet_getInitConfig(pInitCfg);
    pInitCfg->dmaMode.InterruptMode = ETHERNET_DMA_MODE_INTM_MODE2;

    //
    // Assign the callbacks for Getting packet buffer when needed
    // Releasing the TxPacketBuffer on Transmit interrupt callbacks
    // Receive packet callback on Receive packet completion interrupt
    //
    pInitCfg->pfcbRxPacket = &Ethernet_receivePacketCallbackCustom;
    pInitCfg->pfcbGetPacket = &Ethernet_getPacketBuffer;    //custom
    pInitCfg->pfcbFreePacket = &Ethernet_releaseTxPacketBufferCustom;

    //
    //Assign the Buffer to be used by the Low level driver for receiving
    //Packets. This should be accessible by the Ethernet DMA
    //
    pInitCfg->rxBuffer = Ethernet_rxBuffer;

    //
    // The Application handle is not used by this application
    // Hence using a dummy value of 1
    //
    Ethernet_getHandle((Ethernet_Handle)1, pInitCfg , &emac_handle);

    //
    // Disable transmit buffer unavailable and normal interrupt which
    // are enabled by default in Ethernet_getHandle.
    //
    Ethernet_disableDmaInterrupt(Ethernet_device_struct.baseAddresses.enet_base,
                                 0, (ETHERNET_DMA_CH0_INTERRUPT_ENABLE_TBUE |
                                     ETHERNET_DMA_CH0_INTERRUPT_ENABLE_NIE));

    //
    // Enable the MTL interrupt to service the receive FIFO overflow
    // condition in the Ethernet module.
    //
    Ethernet_enableMTLInterrupt(Ethernet_device_struct.baseAddresses.enet_base,0,
                                ETHERNET_MTL_Q0_INTERRUPT_CONTROL_STATUS_RXOIE);

    //
    // Disable the MAC Management counter interrupts as they are not used
    // in this application.
    //
    HWREG(Ethernet_device_struct.baseAddresses.enet_base + ETHERNET_O_MMC_RX_INTERRUPT_MASK) = 0xFFFFFFFF;
    HWREG(Ethernet_device_struct.baseAddresses.enet_base + ETHERNET_O_MMC_IPC_RX_INTERRUPT_MASK) = 0xFFFFFFFF;
    //
    //Do global Interrupt Enable
    //
    (void)Interrupt_enableInProcessor();

    //
    //Assign default ISRs
    //
    Interrupt_registerHandler(INT_EMAC_TX0, Ethernet_transmitISR);
    Interrupt_registerHandler(INT_EMAC_RX0, Ethernet_receiveISR);
    Interrupt_registerHandler(INT_EMAC, Ethernet_genericISRCustom);

    //
    // Convert the mac address string into the 32/16 split variables format
    // that is required by the driver to program into hardware registers.
    // Note: This step is done after the Ethernet_getHandle function because
    //       a dummy MAC address is programmed in that function.
    //
    temp = (uint8_t *)&macLower;
    temp[0] = mac[0];
    temp[1] = mac[1];
    temp[2] = mac[2];
    temp[3] = mac[3];

    temp = (uint8_t *)&macHigher;
    temp[0] = mac[4];
    temp[1] = mac[5];

    //
    // Program the unicast mac address.
    //
    Ethernet_setMACAddr(EMAC_BASE,
                        0,
                        macHigher,
                        macLower,
                        ETHERNET_CHANNEL_0);

    Ethernet_clearMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
    Ethernet_setMACConfigurationCustom(Ethernet_device_struct.baseAddresses.enet_base,ETHERNET_MAC_CONFIGURATION_RE);
}



/**
  * @brief  This functions closes the tcp connection
  * @param  tcp_pcb: pointer on the tcp connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_connection_close(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{

  /* remove all callbacks */
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);

  /* delete es structure */
  if (es != NULL) {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);
}

/**
  * @brief  This function is used to send data for tcp connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  es: pointer on echo_state structure
  * @retval None
  */
static void tcp_echoserver_send(struct tcp_pcb *tpcb, struct tcp_echoserver_struct *es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  while ((wr_err == ERR_OK) &&
         (es->p != NULL) &&
         (es->p->len <= tcp_sndbuf(tpcb)))
  {
    /* get pointer on pbuf from es structure */
    ptr = es->p;

    /* enqueue data for transmission */
	//Pranay,20Oct'22, Handling Echoback for commands that re required to be echoed back only, 
	//else remianing all buffer clearing functionalities shall be remained same, so handling here
    if( isEchobackReq )//Set && PGM mode APIs
        wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    else//For Get Mode commands
        wr_err = ERR_OK;

    if (wr_err == ERR_OK) {
      u16_t plen;
      u8_t freed;
      plen = ptr->len;

      /* continue with next pbuf in chain (if any) */
      es->p = ptr->next;

      if (es->p != NULL) {
        /* increment reference count for es->p */
        pbuf_ref(es->p);
      }

      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      /* chop first pbuf from chain */
       do
       {
         /* try hard to free pbuf */
         freed = pbuf_free(ptr);
       }
       while(freed == 0);
      /* Update tcp window size to be advertized : should be called when received
      data (with the amount plen) has been processed by the application layer */
      tcp_recved(tpcb, plen);
   } else if(wr_err == ERR_MEM) {
      /* we are low on memory, try later / harder, defer to poll */
     es->p = ptr;
   } else {
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This function implements the tcp_poll LwIP callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
  * @retval err_t: error code
  */
static err_t tcp_echoserver_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  es = (struct tcp_echoserver_struct *)arg;
//  if( (es->pcb->state == CLOSED) ||
//          ( es->pcb->state == CLOSE_WAIT) ||
//              ( es->pcb->state == CLOSING)
//      )
  {
      g_tcp_active_cnt++;
//      main();
//      __asm("   bkpt #0");//GRL-EDIT debugging
//      if(gVar == 1)
//          grl_tcp_lwip_re_init();

//      CM_reset();
//      unsigned long ulUser0, ulUser1;
//
      /* Polling for the tcp/ip activity */
      if(g_tcp_active_cnt >= 5)
      {
          g_tcp_active_cnt = 0;
//          tcp_close(tpcb);//Sending FIN
//          tcp_abort(tpcb);//Sending RST
      }
//
//      ulUser0 = GRL_MAC_0 ;
//      ulUser1 = GRL_MAC_1 | (gSystemID & 0xFFF);
//
//      pucMACArray[0] = ((ulUser0 >> 16) & 0xff);
//      pucMACArray[1] = ((ulUser0 >> 8) & 0xff);
//      pucMACArray[2] = ((ulUser0 >> 0) & 0xff);
//      pucMACArray[3] = ((ulUser1 >>  16) & 0xff);
//      pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
//      pucMACArray[5] = ((ulUser1 >> 0) & 0xff);
//
//      //
//      // Initialize ethernet module.
//      //
//      Ethernet_init(pucMACArray);
//      //
//      // Initialze the lwIP library, using DHCP.
//      //
//      IPAddr &= 0xFFFF0000;
//      IPAddr |= gSystemID;
//
//      IP_address=IPAddr;
//      //Re init TCP stack
//
//      lwIPInit(0, pucMACArray, IPAddr, NetMask, GWAddr, IPADDR_USE_STATIC);
//
//      // Initialize the UDP server
//      //
////      netif_init();
//      dhcp_start(&g_sNetIF);
//
//      if( 0 ==  my_tcp_init() )
//      {
//          return 0;
//      }

  }
//  if (es != NULL) {
//    if (es->p != NULL) {
//      /* there is a remaining pbuf (chain) , try to send data */
//      tcp_echoserver_send(tpcb, es);
//    } else {
//      /* no remaining pbuf (chain)  */
//      if (es->state == ES_CLOSING) {
//        /*  close tcp connection */
//        tcp_echoserver_connection_close(tpcb, es);
//      }
//    }
//    ret_err = ERR_OK;
//  } else {
//    /* nothing to be done */
//    tcp_abort(tpcb);
//    ret_err = ERR_ABRT;
//  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_err callback function (called
  *         when a fatal tcp_connection error occurs.
  * @param  arg: pointer on argument parameter
  * @param  err: not used
  * @retval None
  */
static void tcp_echoserver_error(void *arg, err_t err)
{
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(err);

  es = (struct tcp_echoserver_struct *)arg;
  if (es != NULL) {
    /*  free es structure */
    mem_free(es);
  }
}
/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data)
  * @param  None
  * @retval None
  */
u8_t pbuf_free_var = 0;
static err_t tcp_echoserver_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct tcp_echoserver_struct *es;
    pbuf_free_var =0;
    LWIP_UNUSED_ARG(len);

//    pbuf_free_var = pbuf_free(pbuf1_tx);

  es = (struct tcp_echoserver_struct *)arg;

   pbuf_free(es->p);//Pranay,20Oct'22, As per dicussion with Km, adding this as part of the memory management after ACK received from Client

//  if (es->p != NULL) {
//    /* still got pbufs to send */
//    tcp_echoserver_send(tpcb, es);
//  } else {
//    /* if no more data to send and client closed connection*/
//    if (es->state == ES_CLOSING) {
//      tcp_echoserver_connection_close(tpcb, es);
//    }
//  }
    /* if no more data to send and client closed connection*/
//    if (es->state == ES_CLOSING) {
//      tcp_echoserver_connection_close(tpcb, es);
//    }
  return ERR_OK;
}

#if 0

/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t tcp_echoserver_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  struct tcp_echoserver_struct *es;
  err_t ret_err;

  LWIP_ASSERT("arg != NULL",arg != NULL);

//  Rxtcb = tpcb;

  es = (struct tcp_echoserver_struct *)arg;

  /* if we receive an empty tcp frame from client => close connection */
  if (p == NULL) {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if (es->p == NULL) {
       /* we're done sending, close connection */
       tcp_echoserver_connection_close(tpcb, es);
    } else {
      /* we're not done yet */
      /* acknowledge received packet */
      tcp_sent(tpcb, tcp_echoserver_sent);

      /* send remaining data*/
      tcp_echoserver_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }
  /* else : a non empty frame was received from client but for some reason err != ERR_OK */
  else if(err != ERR_OK) {
    /* free received pbuf*/
    if (p != NULL) {
      es->p = NULL;
      pbuf_free(p);
    }
    ret_err = err;
  } else if(es->state == ES_ACCEPTED) {
    /* first data chunk in p->payload */
    es->state = ES_RECEIVED;

    /* store reference to incoming pbuf (chain) */
    es->p = p;
    grl_tcp_recv_data_pop(tpcb,p);
    /* initialize LwIP tcp_sent callback function */
    tcp_sent(tpcb, tcp_echoserver_sent);

    /* send back the received data (echo) */
    tcp_echoserver_send(tpcb, es);

    ret_err = ERR_OK;
  } else if (es->state == ES_RECEIVED) {
    /* more data received from client and previous data has been already sent*/
    if (es->p == NULL) {
      es->p = p;

      /* send back received data */
      tcp_echoserver_send(tpcb, es);
    } else {
      struct pbuf *ptr;

      /* chain pbufs to the end of what we recv'ed previously  */
      ptr = es->p;
      pbuf_chain(ptr,p);
    }
    ret_err = ERR_OK;
  }

  /* data received when connection already closed */
  else {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);

    /* free pbuf and do nothing */
    es->p = NULL;
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}
#endif

/**
  * @brief  This function is the implementation for tcp_recv LwIP callback
  * @param  arg: pointer on a argument for the tcp_pcb connection
  * @param  tpcb: pointer on the tcp_pcb connection
  * @param  pbuf: pointer on the received pbuf
  * @param  err: error information regarding the reveived pbuf
  * @retval err_t: error code
  */
static err_t grltcpServerRecv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct tcp_echoserver_struct *es;
    err_t ret_err;

    LWIP_ASSERT("arg != NULL",arg != NULL);
    es = (struct tcp_echoserver_struct *)arg;

    /* if we receive an empty tcp frame from client => close connection */
    if (p == NULL) {
      /* remote host closed connection */
      es->state = ES_CLOSING;
      //Grl-Edit,19ul'22
//      mem_free_count++;
//      pbuf_free(p);//GRL_EDIT 19Jul'22
//      tcp_recved(tpcb, p->tot_len);
      tcp_sent(tpcb, tcp_echoserver_sent);

      tcp_echoserver_connection_close(tpcb, es);
      ret_err = ERR_OK;
    }
    else {
        /* first data chunk in p->payload */
        es->state = ES_RECEIVED;

        /* store reference to incoming pbuf (chain) */
        es->p = p;

        tcp_sent(tpcb, tcp_echoserver_sent);

        grl_tcp_recv_data_pop(tpcb,p);

//        tpcb->so_options |= SOF_KEEPALIVE;
//        tpcb->keep_intvl = 5000;
//        tpcb->keep_idle = 60000;
//        g_tcp_active_cnt = 0; /* Re-init the tcp activity status to close the in-active connection */

            tcp_echoserver_send(tpcb, es);
//        else//For Get Mode commands
//        {
//            //Pranay,20Sep'22
//            tcp_recved(tpcb, p->tot_len);
//            //Pranay,18March'22
//            pbuf_free(p);
//        }


        ret_err = ERR_OK;
    }
    return ret_err;

}
/**
  * @brief  This function is the implementation of tcp_accept LwIP callback
  * @param  arg: not used
  * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
  * @param  err: not used
  * @retval err_t: error status
  */
static err_t tcp_echoserver_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  err_t ret_err;
  struct tcp_echoserver_struct *es;

  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(err);

  tcp_nagle_disable(newpcb);

  /* set priority for the newly accepted tcp connection newpcb */
  tcp_setprio(newpcb, TCP_PRIO_MIN);

  /* allocate structure es to maintain tcp connection informations */
  es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
  if (es != NULL) {
    es->state = ES_ACCEPTED;
    es->pcb = newpcb;
    es->p = NULL;

    /* pass newly allocated es structure as argument to newpcb */
    tcp_arg(newpcb, es);

    /* initialize lwip tcp_recv callback function for newpcb  */
//    tcp_recv(newpcb, tcp_echoserver_recv);
    tcp_recv(newpcb, grltcpServerRecv);

    /* initialize lwip tcp_err callback function for newpcb  */
    tcp_err(newpcb, tcp_echoserver_error);

    /* initialize lwip tcp_poll callback function for newpcb */
    tcp_poll(newpcb, tcp_echoserver_poll, 120);//For every 40 -50Secs of nterval keep polling

    ret_err = ERR_OK;
  } else {
    /* return memory error */
    ret_err = ERR_MEM;
  }
  return ret_err;
}

void udp_rx_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p,
               struct ip_addr *addr, u16_t port)

{
    char *cad_rx;
    uint16_t long_actual = 0;
    uint16_t long_UDP_complete = 0;
    uint16_t long_total = 0;
    uint8_t cnt_lee = 0;

    httpLEDToggle();

    memset(buf_rx, 0x00, 50);

    long_total = p->tot_len;

    cont_rx_udp++;

    while ((long_UDP_complete < long_total)
            && (long_UDP_complete < 1800) || (cnt_lee == 0))
    {
        cnt_lee++;
        long_actual = p->len;

        cad_rx = p->payload;

        int i = 0;
        for (i = 0; i < long_actual; i++)
        {

            buf_rx[0 + i + long_UDP_complete] = *cad_rx++;
        }
        long_UDP_complete = long_actual + long_UDP_complete;
        buf_rx[long_UDP_complete + 1] = '\n';
//        buf_rx[long_UDP_complete ] = '\n';//GRL-EDIT

        if (long_UDP_complete == long_total)
        {
            pbuf_free(p); /* don't leak the pbuf!*/
        }
        else
        {
            if ( p->next != NULL)
            p = p->next;
            //  pbuf_free(a);                                   /* don't leak the pbuf!*/
        }

//        memset(buf_rx_cm, 0x00, 150);
//        grlAsciitoHexConvertion(buf_rx,buf_rx_cm,long_actual);

//        IPC_sendCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
//                               IPC_CMD_READ_MEM, (uint32_t)buf_rx_cm, 150);

//        IPC_waitForAck(IPC_CM_L_CPU1_R, IPC_FLAG1);

        if ((buf_rx[0] == 'S') && (buf_rx[1] == 'T') && (buf_rx[2] == 'O') && (buf_rx[3] == 'P'))
        {
            //Disconnect
            //udp_disconnect(upcb);

            Connected_udp_28000 = false;
            cnt_Connected_udp_28000 = 0;
        }
        else if ((buf_rx[0] == 'S') && (buf_rx[1] == 'T') && (buf_rx[2] == 'A') && (buf_rx[3] == 'R') && (buf_rx[4] == 'T'))
        {                //Connect

            Connected_udp_28000 = true;
            cnt_Connected_udp_28000 = 0;

            /* process the payload in p->payload */
//            udp_connect(upcb, addr, port); /* connect to the remote host */

        }
    }

    pbuf_free(p);

    if (!Connected_udp_28000)
    {
//        udp_disconnect(upcb);
    }

}

/* UDP initialization ......................................................*/
void my_udp_init(void)
{

    g_upcb = udp_new();
    udp_bind(g_upcb, IP_ADDR_ANY, 28000);
    udp_bind(g_upcb, IP_ADDR_ANY, 29000);
//    udp_recv(g_upcb, &udp_rx_callback, (void*) 0);

}
//*****************************************************************************
//
// This example demonstrates the use of the Ethernet Controller.
//
//*****************************************************************************

unsigned long IPAddr =  0xC0A80009; // 0xC0A80004; //192.168.0.4
unsigned long IPAddr_dup =  0x0400A8C0; //192.168.0.4
unsigned long NetMask = 0xFFFFFF00;
unsigned long GWAddr = 0x00000000;
unsigned char pucMACArray[8];

int my_tcp_init()
{

    /* create new tcp pcb */
    tcp_echoserver_pcb = tcp_new();
    bool retVal = 0;
    if (tcp_echoserver_pcb != NULL)
    {
          err_t err;
          /* bind echo_pcb to port 7 (ECHO protocol) */
          err = tcp_bind(tcp_echoserver_pcb, IP_ADDR_ANY, 5002);

          if (err == ERR_OK)
          {
            /* start tcp listening for echo_pcb */
            tcp_echoserver_pcb = tcp_listen(tcp_echoserver_pcb);

            /* initialize LwIP tcp_accept callback function */
            tcp_accept(tcp_echoserver_pcb, tcp_echoserver_accept);
            retVal = 1;
          }
          else
          {
            retVal = 0;
          }
    }
    else
    {
        retVal = 0;
    }
    return retVal;
}

int my_tcp_5003_init()
{
    /* create new tcp pcb */
    tcp_echoserver_pcb_5003 = tcp_new();

    bool retVal = 0;

    if (tcp_echoserver_pcb_5003 != NULL)
    {
          err_t err;
          /* bind echo_pcb to port 7 (ECHO protocol) */
          err = tcp_bind(tcp_echoserver_pcb_5003, IP_ADDR_ANY, 5003);

          if (err == ERR_OK)
          {
            /* start tcp listening for echo_pcb */
              tcp_echoserver_pcb_5003 = tcp_listen(tcp_echoserver_pcb_5003);

            /* initialize LwIP tcp_accept callback function */
            tcp_accept(tcp_echoserver_pcb_5003, tcp_echoserver_accept);
            retVal = 1;
          }
          else
          {
            retVal = 0;
          }
    }
    else
    {
        retVal = 0;
    }
    return retVal;
}

/**Sending data to UDP Client*/
void grlUdpDataTransfer(uint32_t * aTxBuf, uint16_t aDataLength)
{
    uint8_t i = 0;

    memset(buf_tx, 0x00, PAYLOAD);

    for (i = 0; i < aDataLength; i++)
    {
        buf_tx[i] = (uint8_t)aTxBuf[i];
    }

    ////////////////////////////////////////////

    pbuf1_tx = pbuf_alloc(PBUF_TRANSPORT, PAYLOAD, PBUF_RAM);
    if (pbuf1_tx!= NULL)
    {
        pbuf1_tx->payload = (void*) buf_tx;
        pbuf1_tx->tot_len = PAYLOAD;  //17        // long_UDP_complete+4;
        pbuf1_tx->len = PAYLOAD;   //17       // long_UDP_complete+4;

//        udp_send(g_upcb, pbuf1_tx);
    }

    if (pbuf1_tx!= NULL)
        pbuf_free(pbuf1_tx);

}
/*
err_t grlTcpDataTx_5003(uint16_t * aTxBuf, uint16_t aDataLength)
{

    gReadAPI = false;
    gReadPollingData = false;
    err_t err;
    u16_t len = aDataLength,max_len = 0;

    max_len = tcp_sndbuf(Rxtcb);
    if (max_len < len) {
      len = max_len;
    }
    err = tcp_write(Rxtcb_5003, aTxBuf,len, 1);

    tcp_output (Rxtcb_5003);

    if(err != ERR_OK)
          return err;

    return err;
}
*/

err_t grlTcpDataTx(uint16_t * aTxBuf, uint16_t aDataLength)
{
    err_t err;

#if 0
    gReadAPI = false;
    gReadPollingData = false;
    gDatalength = 0;
    gDatalength = aDataLength;

    pbuf1_tx = pbuf_alloc(PBUF_TRANSPORT, aDataLength, PBUF_RAM);
    if (pbuf1_tx!= NULL)
    {
        pbuf1_tx->payload = (void*) aTxBuf;
        pbuf1_tx->tot_len = aDataLength;  //17        // long_UDP_complete+4;
        pbuf1_tx->len = aDataLength;   //17       // long_UDP_complete+4;


        struct tcp_echoserver_struct *es;

        /* allocate structure es to maintain tcp connection informations */
        es = (struct tcp_echoserver_struct *)mem_malloc(sizeof(struct tcp_echoserver_struct));
        if (es != NULL)
        {
          es->pcb = Rxtcb;
          es->p = pbuf1_tx;
          es->p->len = aDataLength;

          /* pass newly allocated es structure as argument to newpcb */
          tcp_arg(Rxtcb, es);

          llen = tcp_sndbuf(Rxtcb);

          err = tcp_write(Rxtcb, aTxBuf,aDataLength, 1);
          if(ERR_OK != err )
          {
              __asm("   bkpt #0");//GRL-EDIT debugging

          }
          err = tcp_output (Rxtcb);
          if(ERR_OK != err )
          {
              __asm("   bkpt #0");//GRL-EDIT debugging
          }

//          while( aDataLength <= tcp_sndbuf(Rxtcb) )
//          tcp_echoserver_send(Rxtcb,es);

          /* initialize LwIP tcp_sent callback function */
          tcp_sent(Rxtcb, tcp_echoserver_sent);
        }
        else
        {
            __asm("   bkpt #0");//GRL-EDIT debugging
        }

    }
    else
    {
        memset(aTxBuf, 0x00, aDataLength);
        //Filling error condition
        aTxBuf[0] = 0xAA;
        aTxBuf[1] = 0xAA;
        aTxBuf[2] = 0xAA;
        aTxBuf[3] = 0xAA;
        aDataLength = 4;
        err = tcp_write(Rxtcb, aTxBuf,aDataLength, 1);
        err = tcp_output (Rxtcb);
    }

#endif


//#if 0
//    u16_t len = 1024,max_len = 0;

//    uint16_t TCPTxBuf[1078] = {0};
//
//    memcpy(TCPTxBuf, aTxBuf, (aDataLength*2));

//    max_len = tcp_sndbuf(Rxtcb);
//    if (max_len < len) {
//      len = max_len;
//    }

    err = tcp_write(Rxtcb, aTxBuf,aDataLength, 1);
    if(ERR_OK != err )
    {
      // __asm("   bkpt #0");//GRL-EDIT debugging
    }
    else
    {
        err = tcp_output (Rxtcb);
        if(ERR_OK != err )
        {

         //  __asm("   bkpt #0");//GRL-EDIT debugging
        }
    }


    return err;
//#endif
}

//
//Pranay, 11March,22
//IPC ISR for communication between Cm and CPU2,
// THis will get executed when CM receives data from CPU2
//
__interrupt void IPC_CPU2_CM_ISR1()
{
    uint32_t command, addr, data;

    IPC_readCommand(IPC_CM_L_CPU2_R, IPC_FLAG3, IPC_ADDR_CORRECTION_ENABLE,
                &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CM_L_CPU2_R, IPC_FLAG3);

    int i =0,index = 0;

    uint32_t lVar[286] = {0};
    uint16_t lTxBuf[4] = {0};

    memset(CPU2RxBuf, 0x00, CPU2_RXBUFSIZE);

    //
    //Pranay,11March'22
    //Data will be received in 32bit format,
    // i.e., If we're sending AA,BB,CC,DD from Remote core, it will be received as
    // 00BB00AA, 00DD00CC,
    //So receiving data here in a 32 bit variable
    //
    for (i = 0; i < (data/2); i++)
    {
        lVar[0 + i] = (*((uint32_t *)addr + i) );
    }

    //
    //Pranay,11March'22
    // Here in Ti we only have a min of 16bit addressing, there is not 8 bit addressing even though you decalre as uint8, it will create a 16 bit variable
    // Once the whole data is received in to lVar buffer further go for decoding
    // First we're decoding Each byte data from lVar and storing into lTxBuf
    // then further forming a pair of Byte(1 word) with 2 bytes LSB first and MSB next
    // we have other alternate way of writing this logic aswell, in whichwe can track only 0th Byte and 2nd Byte directly into CPU2RxBuf
    //
    // in looping condition,
    // +1 for rounding off ending condition,
    // * 2 when zeroes included as MSB ByteCount increases by 2
    //
    for (i = 0; i < ( ( (data/4) + 1) * 2); i++)
    {
        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        //Storing the neighbouring Bytes
//        CPU2RxBuf[index++] = (lTxBuf[0] | (lTxBuf[1] << 8)) ;
//        CPU2RxBuf[index++] = (lTxBuf[2] | (lTxBuf[3] << 8)) ;

        CPU2RxBuf[index++] = (lTxBuf[0] | (lTxBuf[2] << 8)) ;

    }

    if((CPU2RxBuf[0] & 0x0F) == 0x0d)
       {
         grlTcpDataTx(&CPU2RxBuf[0], 10);
       }
    if(CPU2RxBuf[0] == 0xC2C1)//Data from C2 representing System details (s.no and MFD month,year)
    {
        DecodeSystemDetails(CPU2RxBuf);
    }
//    grlUdpDataTransfer(buf_rx_cm,data);
    else if(gReadAPI)
        grlTcpDataTx(CPU2RxBuf,data);

}
__interrupt void IPC_CPU1_CM_POLLING_ISR()
{
    uint32_t command, addr, data;

    IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG1, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG1);
//    memset(PollingRxBuf, 0x00, 28);
    int i =0,index = 0;

    uint32_t lVar[286] = {0};
    uint16_t lTxBuf[4] = {0};

    memset(PollingRxBuf, 0x00, 128);

    //
    //Pranay,11March'22
    //Data will be received in 32bit format,
    // i.e., If we're sending AA,BB,CC,DD from Remote core, it will be received as
    // 00BB00AA, 00DD00CC,
    //So receiving data here in a 32 bit variable
    //
    for (i = 0; i < (data); i++)
    {
        lVar[0 + i] = (*((uint32_t *)addr + i) );
    }

    for (i = 0; i < ( ( (data/4) + 1) * 2); i++)//+1 for rounding off, *2 when zeroes included as MSB ByteCount increases
    {

        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        //Here instead of forming a word with neighbouring bytes, we're forming a word with bytes in which data is actually present
//        PollingRxBuf[index++] = (lTxBuf[0] | (lTxBuf[2] << 8)) ;
        PollingRxBuf[index++] = (lTxBuf[0] | (lTxBuf[2] << 8)) ;
    }

    if(gReadAPI )
        grlTcpDataTx(&PollingRxBuf[1],data);

//    if( ((PollingRxBuf[0] & 0x0F) == 0x0F) && (gReadPollingData == true) )
//    {
//        grlTcpDataTx_5003(&PollingRxBuf[1],data);
//    }
}
void grlGetPhyRegInfo(uint16_t aGetRegVal)
{
    uint16_t phyRegContent=0;
    uint8_t luArtTxBuf[16] = {0};
    luArtTxBuf[0] = 0xBB;
    luArtTxBuf[1] = 0x01;
    phyRegContent= Ethernet_readPHYRegister(EMAC_BASE,aGetRegVal);

}
//
// Pranay, 11March,22
// IPC ISR for communication between CM and CPU1,
// THis will get executed when CM receives data from CPU1
//

__interrupt void IPC_CPU1_CM_ISR0()
{
    uint32_t command, addr, data;

    IPC_readCommand(IPC_CM_L_CPU1_R, IPC_FLAG0, IPC_ADDR_CORRECTION_ENABLE,
                    &command, &addr, &data);

    IPC_ackFlagRtoL(IPC_CM_L_CPU1_R, IPC_FLAG0);

    memset(CPU1RxBuf, 0x00, MAX_DATA_TX_SIZE);

    int i =0,index = 0;

    uint32_t lVar[286] = {0};
    uint16_t lTxBuf[4] = {0};

    //
    //Pranay,11March'22
    //Data will be received in 32bit format,
    // i.e., If we're sending AA,BB,CC,DD from Remote core, it will be received as
    // 00BB00AA, 00DD00CC,
    //So receiving data here in a 32 bit variable
    //
    for (i = 0; i < (data); i++)
    {
        lVar[0 + i] = (*((uint32_t *)addr + i) );
    }

    //
    //Pranay,11March'22
    // Here in Ti we only have a min of 16bit addressing, there is not 8 bit addressing even though you decalre as uint8, it will create a 16 bit variable
    // Once the whole data is received in to lVar buffer further go for decoding
    // First we're decoding Each byte data from lVar and storing into lTxBuf
    // then further forming a pair of Byte(1 word) with 2 bytes LSB first and MSB next
    // we have other alternate way of writing this logic aswell, in whichwe can track only 0th Byte and 2nd Byte directly into CPU2RxBuf
    //
    // in looping condition,
    // +1 for rounding off ending condition,
    // * 2 when zeroes included as MSB ByteCount increases by 2
    //
    for (i = 0; i < ( ( (data/4) + 1) * 2); i++)//+1 for rounding off, *2 when zeroes included as MSB ByteCount increases
    {

        lTxBuf[0] = lVar[i] & 0xFF;
        lTxBuf[1] = (lVar[i] & 0xFF00) >> 8;
        lTxBuf[2] = (lVar[i] & 0xFF0000) >> 16;
        lTxBuf[3] = (lVar[i] & 0xFF000000) >> 24;

        //Here instead of forming a word with neighbouring bytes, we're forming a word with bytes in which data is actually present
        CPU1RxBuf[index++] = (lTxBuf[0] | (lTxBuf[2] << 8)) ;

    }

//    if( ((CPU1RxBuf[0] & 0x0F) == 0x0F) && (gReadPollingData == true) )
//    {
//        memcpy(&PollingRxBuf[0], &CPU1RxBuf[1], data);
//        grlTcpDataTx_5003(PollingRxBuf,data);
//    }
//    grlUdpDataTransfer(buf_rx_cm,data);
//    if( (CPU1RxBuf[0] ) == 0xBB)
 //   {
//        grlGetPhyRegInfo(CPU1RxBuf[2]);
//    }
    if( (CPU1RxBuf[0] & 0x0F) == 0x02)//if related to PGM
    {
        uint8_t lBuf[24] = {0};
        uint8_t FWAPILength = 0;
        index = 0;
        FWAPILength = CPU1RxBuf[0] >> 8;

        for(i =0; i < FWAPILength; i++)
        {
            lBuf[index++] = CPU1RxBuf[i];
            lBuf[index++] = CPU1RxBuf[i] >> 8;
        }
        tcpRxdataHandler(lBuf);
    }
    else if((CPU1RxBuf[0] & 0x0F) == 0x0d)
        {
            grlTcpDataTx(&CPU1RxBuf[0], 10);
        }

    if(gReadAPI )
        grlTcpDataTx(CPU1RxBuf,data);

}

void ipc_init()
{

    //CM- CPU1
    IPC_clearFlagLtoR(IPC_CM_L_CPU1_R, IPC_FLAG_ALL);
    IPC_registerInterrupt(IPC_CM_L_CPU1_R, IPC_INT0, IPC_CPU1_CM_ISR0);

    //CM- CPU1 -polling
    IPC_clearFlagLtoR(IPC_CM_L_CPU1_R, IPC_FLAG_ALL);
    IPC_registerInterrupt(IPC_CM_L_CPU1_R, IPC_INT1, IPC_CPU1_CM_POLLING_ISR);

    //CM -CPU2
    IPC_clearFlagLtoR(IPC_CM_L_CPU2_R, IPC_FLAG_ALL);
    IPC_registerInterrupt(IPC_CM_L_CPU2_R, IPC_INT3, IPC_CPU2_CM_ISR1);

}



void led_toggle()
{
    int i;
    GPIO_writePin(36, 0);
  //        DEVICE_DELAY_US(100000);//it was 10us
          for (i=0;i<100000;++i)
          {

          }
          GPIO_writePin(37, 1);
  //        DEVICE_DELAY_US(100000);//it was 10us
          for (i=0;i<100000;++i)
          {

          }
}



void TxIPAddr()
{
//    delay();
    IPAddr_dup = IP_address & 0xFFFF0000;
    IPAddr_dup |= gSystemID;

    if(IP_address != IPAddr_dup)
    {
        buf_rx[0]=0x01;
        buf_rx[1]=8;
        buf_rx[2]=0x05;
        buf_rx[3]= IP_address;
        buf_rx[4]=(IP_address)>>8;
        buf_rx[5]=(IP_address )>>16;
        buf_rx[6]=(IP_address )>>24;
        tcpRxdataHandler(buf_rx);
//        delay();

    }
    else
    {
        buf_rx[0]=0x01;
        buf_rx[1]=8;
        buf_rx[2]=0x05;
        buf_rx[6]= IP_address;
        buf_rx[5]=(IP_address)>>8;
        buf_rx[4]=(IP_address )>>16;
        buf_rx[3]=(IP_address )>>24;
        tcpRxdataHandler(buf_rx);
//        delay();
    }
}

void sys_id()
{
//    delay();
    buf_rx[0]=0x01;
    buf_rx[1]=8;
    buf_rx[2]=0x06;
    buf_rx[3]=gMFDYear>>8;//Change this to LSB First amd MSB nxt
    buf_rx[4]=gMFDYear;
    buf_rx[5]=gSystemID>>8;//Change this to LSB First amd MSB nxt
    buf_rx[6]=gSystemID;
    tcpRxdataHandler(buf_rx);
//delay();
}


void dectohex(uint16_t num)
{
    uint16_t hex=0,temp=0;
    while(num!=0)
    {
        temp=num%16;
        num=num/16;
        hex=hex|temp<<4;
    }

}
void delay()
{
    int i , j =0;
        //
        // Turn on LED
        //
        GPIO_writePin(36, 0);
        //
        // Delay for 500000uS.
        //
        for(j=0;j<50;j++)
            for(i = 0; i<50000;++i )
                ;
//        for(i = 0; i<500000;++i );
        //
        // Turn off LED
        //
        GPIO_writePin(36, 1);
        for(j=0;j<50;j++)
            for(i = 0; i<50000;++i )
                ;
}
void grlFWVarInitializations()
{
    gReadAPI = false;

    /**Pranay,03Sept'22, Handling the echoback and msg ID sequence during FW udpates, If prev and present MSG ID is same then ignore received FW packet
     * initializing gFWupdPrevMsgID to 0x0F to handle first FW update command after bootup,if it 0x00 then first packet will be ignored
     * */
    gFWupdPresentRxMsgID = 0x00;
    gFWupdPrevMsgID = 0x0F;
    isEchobackReq = false;
}

int main(void)
{
    unsigned long ulUser0, ulUser1;

    //  ////////////////////////////////////////
    // Initializing the CM. Loading the required functions to SRAM.
    //
    CM_init();

    SYSTICK_setPeriod(systickPeriodValue);
    SYSTICK_enableCounter();
    SYSTICK_registerInterruptHandler(SysTickIntHandler);
    SYSTICK_enableInterrupt();

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    // Enable processor interrupts.
    //
    Interrupt_enableInProcessor();

    ipc_init();

    IPC_sync(IPC_CM_L_CPU2_R, IPC_FLAG30);

    INIT_CPU_TIMERS();//Initialize only CPU0 and CPU1 because CPU2 is being reserved for RTOS Idle task operation, Ensure no CPU2 timer will be created

    setupMsgTimer0();

    setupMsgTimer1();
    // Set user/company specific MAC octets
    // (for this code we are using A8-63-F2-00-00-80)
    // 0x00 MACOCT3 MACOCT2 MACOCT1
    ulUser0 = GRL_MAC_0 ;
    // 0x00 MACOCT6 MACOCT5 MACOCT4
    ulUser1 = GRL_MAC_1 | (gSystemID & 0xFFF);

    //
    // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
    // address needed to program the hardware registers, then program the MAC
    // address into the Ethernet Controller registers.
    //

//    pucMACArray[0] = ((ulUser0 >>  0) & 0xff);
//    pucMACArray[1] = ((ulUser0 >>  8) & 0xff);
//    pucMACArray[2] = ((ulUser0 >> 16) & 0xff);
//    pucMACArray[3] = ((ulUser1 >>  0) & 0xff);
//    pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
//    pucMACArray[5] = ((ulUser1 >> 16) & 0xff);

    pucMACArray[0] = ((ulUser0 >> 16) & 0xff);
    pucMACArray[1] = ((ulUser0 >> 8) & 0xff);
    pucMACArray[2] = ((ulUser0 >> 0) & 0xff);
    pucMACArray[3] = ((ulUser1 >>  16) & 0xff);
    pucMACArray[4] = ((ulUser1 >>  8) & 0xff);
    pucMACArray[5] = ((ulUser1 >> 0) & 0xff);

    //
    // Initialize ethernet module.
    //
    Ethernet_init(pucMACArray);

    //
    // Initialze the lwIP library, using DHCP.
    //
    IPAddr &= 0xFFFF0000;
    IPAddr |= gSystemID;

    IP_address=IPAddr;
    lwIPInit(0, pucMACArray, IPAddr, NetMask, GWAddr, IPADDR_USE_STATIC);

    // Initialize the UDP server
    //
    netif_init();
    my_udp_init();
    dhcp_start(&g_sNetIF);

    if( 0 ==  my_tcp_init() )
    {
        return 0;
    }

    if(0 == my_tcp_5003_init() )
    {
        return 0;
    }

    grlFWVarInitializations();

    //
    // Loop forever. All the work is done in interrupt handlers.
    //
//    Interrupt_setPriority(INT_EMAC_TX0, 2);
//    Interrupt_setPriority(INT_EMAC_RX0, 1);

//    Pranay,14Jul'22, As per Ti Forum input
//    ->https://e2e.ti.com/support/microcontrollers/c2000-microcontrollers-group/c2000/f/c2000-microcontrollers-forum/1011110/tms320f28388d-lwip-stack-crash
    Interrupt_setPriority(INT_EMAC_TX0, 0);
    Interrupt_setPriority(INT_EMAC_RX0, 2);
    Interrupt_setPriority(INT_EMAC, 4);

    Interrupt_enable(INT_EMAC_TX0);
    Interrupt_enable(INT_EMAC_RX0);
    Interrupt_enable(INT_EMAC);

    delay();
    sys_id();
    delay();
    TxIPAddr();
    GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);
    MsgTimerStart(1000, 0, TIMER0);

    while (1)
    {
        sys_check_timeouts();
    }
}

//*****************************************************************************
//
// Called by lwIP Library. Toggles the led when a command is received by the
// HTTP webserver.
//
//*****************************************************************************
void httpLEDToggle(void)
{
    //
    // Toggle the LED D1 on the control card.
    //
    GPIO_togglePin(DEVICE_GPIO_PIN_GREENLED36);
}

//*****************************************************************************
//
// Called by lwIP Library. Could be used for periodic custom tasks.
//
//*****************************************************************************

uint32_t cnt_ms_lwip_Htimer=0;
uint32_t cnt_ms_TX_Htimer=0;
void lwIPHostTimerHandler(void)
{
//	msTime++;

	cnt_ms_lwip_Htimer++;
}
