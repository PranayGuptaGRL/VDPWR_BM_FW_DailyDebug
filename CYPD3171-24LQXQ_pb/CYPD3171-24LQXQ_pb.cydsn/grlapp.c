/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

/* [] END OF FILE */

#include <grlapp.h>
#include <grlabstraction.h>
#include <pdo.h>
#include <battery_charging.h>


uint8_t gI2cTransBuffer[512] = {0};
uint8_t gLogBuffer[256] = {0};
uint8_t greadConfigBuf[16] = {0};

uint16_t gEventlogBufIndex;
uint8_t gEventlogBuffer[EVNT_LOG_BUF_SIZE];
uint8_t gBattStatBuf[16];

#if ONLY_PD_SNK_FUNC_EN

#define BYTELENGTH  8

uint8_t gSOP1AckBuf[64];

#endif/*ONLY_PD_SNK_FUNC_EN*/

void DrpConfiguration()
{
    ccg_status_t aRetstat;
  /*  pd_typec_en_rp(0,CC_CHANNEL_1,RP_TERM_RP_CUR_3A);
    pd_typec_en_rp(0,CC_CHANNEL_2,RP_TERM_RP_CUR_3A);
    
    pd_typec_en_rd(0,CC_CHANNEL_1);
    pd_typec_en_rd(0,CC_CHANNEL_2);
    CyDelay(1);
    pd_hal_config_auto_toggle(0,1);
    CyDelay(1);*/
    
    aRetstat = dpm_update_port_config (TYPEC_PORT_0_IDX, PRT_DUAL, PRT_ROLE_SOURCE, true, false);
    CyDelay(1);
    dpm_update_port_config (TYPEC_PORT_0_IDX, PRT_DUAL, PRT_ROLE_SOURCE, true, false);
    
    gBufLog(false,0xAD);
    gBufLog(false,aRetstat);
}
void SrcConfiguration()
{
    ccg_status_t aRetstat;
    pd_typec_en_rp(0,CC_CHANNEL_1,RP_TERM_RP_CUR_3A);
    pd_typec_en_rp(0,CC_CHANNEL_2,RP_TERM_RP_CUR_3A);
    CyDelay(1);
    aRetstat = dpm_update_port_config (TYPEC_PORT_0_IDX, PRT_ROLE_SOURCE, PRT_ROLE_SOURCE, false, false);
    CyDelay(1);
    dpm_update_port_config (TYPEC_PORT_0_IDX, PRT_ROLE_SOURCE, PRT_ROLE_SOURCE, false, false);
    
    gBufLog(false,0xAE);
    gBufLog(false,aRetstat);
}
void SinkConfiguration()
{
    ccg_status_t aRetstat;
    pd_typec_en_rd(0,CC_CHANNEL_1);
    pd_typec_en_rd(0,CC_CHANNEL_2);
    CyDelay(1);
    aRetstat = dpm_update_port_config (G_PORT0, PRT_ROLE_SINK, PRT_ROLE_SINK, false, false);
    CyDelay(1);
    dpm_update_port_config (G_PORT0, PRT_ROLE_SINK, PRT_ROLE_SINK, false, false);
    
    gBufLog(false,0xAF);
    gBufLog(false,aRetstat);
}
void toggle_PDNegLed()
{
    static int lVar = 0;
    
    gpio_set_value (GPIO_PORT_2_PIN_1, (lVar&1) );
}
static uint8_t gRetryCount;
static void timer_expiry_callback(uint8_t instance, timer_id_t id)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(id)
    {
#if ONLY_PD_SNK_FUNC_EN
        case GRL_APP_SOP1_TIMER:
            g_Struct_Ptr->gPDSSConfigCtrl.CableIntrRx = false;
            g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
            gInitSOP1DiscID();
        break;
#endif
        case GRL_SRC_PSRDY_TIMER:
            psrc_select_voltage(0);
        break;  
        case GRL_SRC_ATTACH_INTR_TIMER:
            psrc_select_voltage(0);
        break;
        case GRL_INIT_GET_STATUS:
            dpm_pd_command(0, DPM_CMD_GET_STATUS, NULL, NULL);
        break;
        case GRL_INIT_OCP_ALERT:
            g_Init_AlertDataMsg();
            g_Struct_Ptr->gLogData.isOCPOccurred = false;
        break;
        case GRL_INIT_GET_BATT_CAPS:
            g_Init_BattCapsMsg();
        break;
        case GRL_INIT_GET_BATT_STATUS:
            memset(greadConfigBuf, 0, 16);

            greadConfigBuf[0] = 0x21;
            greadConfigBuf[1] = 0x0A;
            greadConfigBuf[2] = 0x02;
            greadConfigBuf[3] = 0x03;
            greadConfigBuf[4] = 0xF2;//Custom config
            greadConfigBuf[5] = 0x0D;//GetBatterystatus Extnded msg
            greadConfigBuf[6] = 0x01;
            greadConfigBuf[7] = 0x00;
            greadConfigBuf[8] = 0x00;
            greadConfigBuf[9] = 0x00;
            
            InitGetBattStatusExtndMsg(greadConfigBuf);
        break;
        case GRL_PORT_ROLE_SRC:
                SrcConfiguration();
                schedule_task(200,GRL_PORT_ATTACH);
            break;
        case GRL_PORT_ROLE_SNK:
                SinkConfiguration();
                schedule_task(200,GRL_PORT_ATTACH);
            break;
        case GRL_PORT_ROLE_DRP:
                DrpConfiguration();
                schedule_task(200,GRL_PORT_ATTACH);
            break;
                    
        case GRL_PORT_ATTACH:

                gpio_set_value (GPIO_PORT_2_PIN_1, 1);
                CableConnectionSimulation(GRL_ATTACH_NO_CHECK_PDC);
            break;
        case GRL_PORT_DETACH:
                g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag = GRL_PD_MODE_SET;
                CableConnectionSimulation(GRL_DETACH_WITH_RP_RD_DISABLE);
                schedule_task(60,GRL_ATTACH_STATE_POLL);
                break;
        case GRL_ATTACH_STATE_POLL:
                
                if(dpm_get_info(G_PORT0)->attach != true)//Poll if transitioned in to detach state or not for 3 times * 640mS
                {
                    //Pranay,If transitioned into Detach state then only set mode to sink/src/DRp based on API received
                    switch(g_Struct_Ptr->gCustomConfig.gReceivedSetPortRoleCmd)
                    {
                        case PRT_ROLE_SINK:
                            schedule_task(640,GRL_PORT_ROLE_SNK);//650mS is tDetach as per TypeC Cable Spec
                        break;
                        case PRT_ROLE_SOURCE:
                            schedule_task(640,GRL_PORT_ROLE_SRC);
                        break;
                        case PRT_DUAL:
                            schedule_task(640,GRL_PORT_ROLE_DRP);
                        break;
                    }
                }
                else if(gRetryCount++ >= 2)//Retrying for 3 times
                {
                    schedule_task(2,GRL_PORT_DETACH);
                }
                else
                {
                    //TBD, what to do if not been to detach state even after 3 times retrying
                }
                break;
                /*venkat 6Jul'2022,D+ line from 0.6v to 3.3v by considering current and required pulse count criteria*/
        case  GRL_DP_PULSE : 
          if(g_Struct_Ptr->gPDSSConfigCtrl.gCurrent_Pulsecnt < g_Struct_Ptr->gPDSSConfigCtrl.gReqPulse_Count)
            {
                PDSS->bch_det_0_ctrl[0] |=  PDSS_BCH_DET_0_CTRL_RDP_PU_EN;
                PDSS->bch_det_0_ctrl[0] &= ~(PDSS_BCH_DET_0_CTRL_VDP_SRC_EN);
                schedule_task(2,GRL_DPSTAB);
                g_Struct_Ptr->gPDSSConfigCtrl.gCurrent_Pulsecnt ++;
            }
            break;
        /*venkat 6Jul'2022, D- line from 3.3v to 0.6v by considering current and required pulse count criteria*/
        case GRL_DM_PULSE: 
            if(g_Struct_Ptr->gPDSSConfigCtrl.gCurrent_Pulsecnt < g_Struct_Ptr->gPDSSConfigCtrl.gReqPulse_Count)
            {
                PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_VDM_SRC_EN;
                PDSS->bch_det_0_ctrl[0] &= ~(PDSS_BCH_DET_0_CTRL_RDM_PU_EN );
                schedule_task(2,GRL_DMSTAB);
                g_Struct_Ptr->gPDSSConfigCtrl.gCurrent_Pulsecnt ++;
            }
            
            break;
        /*venkat 6Jul'2022,bring back pulse of D+ line from 3.3v to 0.6v*/
        case GRL_DPSTAB:  
            PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            PDSS->bch_det_0_ctrl[0] &= ~(PDSS_BCH_DET_0_CTRL_RDP_PU_EN);
            schedule_task(1,GRL_DP_PULSE);
        
            break;
        /*venkat 6Jul'2022,make D- high again from 0.6v to 3.3v*/
        case  GRL_DMSTAB:  
              PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_RDM_PU_EN ;
              PDSS->bch_det_0_ctrl[0] &= ~(PDSS_BCH_DET_0_CTRL_VDM_SRC_EN);
              schedule_task(1,GRL_DM_PULSE);
            break;
        /*venkat 11Jul'2022,QC contineouse mode task,  used to make QC3.0 mode change automatic both in Attach state and Detach state */    
        case GRL_QC_CONT_MODE:
            /* 0.6 on D+, D- 3.3 */
            PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            
            PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_RDM_PU_EN ;
            
            /* Remove other terms */
            PDSS->bch_det_0_ctrl[0] &= ~(PDSS_BCH_DET_0_CTRL_RDP_PU_EN|PDSS_BCH_DET_0_CTRL_VDM_SRC_EN);
            break;
        /*venkat 6Jul'2022,Enumeration task, used to make QC mode change automatic both in Attach state and Detach state*/
        case GRL_QCENUMERATION:
            /* 0.6 on D+, D- HiZ */
            PDSS->bch_det_0_ctrl[0] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            
            /* Remove other terms */
            PDSS->bch_det_0_ctrl[0] &= ~ (PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
                                    PDSS_BCH_DET_0_CTRL_IDM_SNK_EN);
            
            if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag == QC3MODE)
            {
                schedule_task(2000,GRL_QC_CONT_MODE);
            }
            
            break;

        case GRL_QC_DETACH:
    
            break;
        /*venkat 6Jul'2022,Attach task, Specifically used for QC mode change from PD mode*/
        case GRL_QC_ATTACH:
            CableConnectionSimulation(GRL_CABLE_ATTACH);
            //dpm_start (G_PORT0);
            if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag != PDMODE)
                schedule_task(500,GRL_QCENUMERATION);
            
            break;
    }
  
}
/*Soft timers are identified using a single byte timer ID, and the caller should ensure that the timer ID used does not collide with
timers used elsewhere. This is facilitated by reserving the timer ID range from 0xE0 to 0xFF for use by user application code.
These timer IDs are not used internally within the CCGx firmware stack and are safe for use**/
/* Use a timer to schedule task to be run delay_ms milliseconds later. */
void schedule_task(uint16_t adelay_ms,timer_id_t aTimerID)
{
    /* Start an application timer to wait for delay_ms.
    Devices with two USB-PD ports support two sets of timers, and
    the set to be used is selected using the first parameter. */
    timer_start(0, aTimerID, adelay_ms, timer_expiry_callback);
}
#if ONLY_PD_SNK_FUNC_EN
//    static uint8_t var;
#if 0
    
/**This function can be used if needed to decode every PDO and push only specific required info*/    
void g_fill_DiscID_PDO_info(uint8_t aPresentPDOIndex, const pd_do_t* ACK_VDO, uint8_t *aBuffer)
{
    switch(aPresentPDOIndex)
    {
        case 0:
        
        break;
        case 1:
        
        break;
        
    }
}
#endif

void g_Decode_Rx_Ack_Hdr(pd_packet_extd_t* rx_pd_packet,uint8_t *aBuffer, uint8_t * aBufIndex)
{
    /**decoding header and pushing into buffer*/
    aBuffer[*aBufIndex] = rx_pd_packet->hdr.val; 
    *aBufIndex = *aBufIndex + 1;
    
    aBuffer[*aBufIndex] = rx_pd_packet->hdr.val >> 8; 
    *aBufIndex = *aBufIndex + 1;
}

void g_Decode_Rx_PayloadData(pd_packet_extd_t* rx_pd_packet,uint8_t *aBuffer, uint8_t * aBufIndex,uint8_t aPDOCount)
{
    /**Filling received PDOs into Buffer*/
    for(uint8_t index = 0 ; index < aPDOCount; ++index)
    {
        //g_fill_DiscID_PDO_info(index , &rx_pd_packet->dat[index], aBuffer);
        /**Taking Each PDO data and filling data into buffer by looping */
        for(uint8_t i =0,j=0; i < 4; j+=8,++i )
        {
            aBuffer[*aBufIndex] |= (rx_pd_packet->dat[index].val >> j); 
            *aBufIndex = *aBufIndex + 1;
        }
    }
}

uint8_t g_Decode_Rx_Ack()
{
    static pd_packet_extd_t* rx_pd_packet ;
    rx_pd_packet = pd_phy_get_rx_packet(0);
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount = 0;
    
    uint8_t lBufIndex=0, lPDOLength = 0;
  
    memset(&gSOP1AckBuf[0], 0x00, 64);
    
    /**soptype into Upper nibble*//**Cmd Type to lower nibble*/
    gSOP1AckBuf[lBufIndex++] = ((rx_pd_packet->sop << 4) | (rx_pd_packet->dat[0].std_vdm_hdr.cmd_type & 0x03) | ((g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC & 0x01) << 2)) ;
    
   /* gSOP1AckBuf[lBufIndex] |= (rx_pd_packet->dat[0].std_vdm_hdr.cmd_type & 0x03);
    gSOP1AckBuf[lBufIndex++] |= (g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC << 2);*/
    
    gSOP1AckBuf[lBufIndex++] = lPDOLength = rx_pd_packet->len;
    
    gSOP1AckBuf[lBufIndex++] =  rx_pd_packet->hdr.val;
    gSOP1AckBuf[lBufIndex++] =  (rx_pd_packet->hdr.val >> BYTELENGTH);
    
    for(uint8_t i =0; i < lPDOLength; ++i )
    {
        for(uint8_t j=0; j<4; j++)
            gSOP1AckBuf[lBufIndex++] |= (rx_pd_packet->dat[i].val >> (BYTELENGTH*j) );
    }
    /**Decoding Received ACK Header*/
    //g_Decode_Rx_Ack_Hdr(rx_pd_packet,gSOP1AckBuf, &lBufIndex);
    
    /**storing received PDOs length*/
    
    
    /**Decoding received Payload data*/
    //g_Decode_Rx_PayloadData(rx_pd_packet,gSOP1AckBuf,&lBufIndex,lPDOLength);
    
    return lBufIndex;
}
#endif /*ONLY_PD_SNK_FUNC_EN*/

/*
*Pranay 07Jun'19, CCG3PA PD subsystem status related controls will be handled here
*/
void g_PDSS_Status(uint8_t * RecvBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
#if ONLY_PD_SNK_FUNC_EN    
//    if(RecvBuffer[3] == GRL_BC12_SCOPE)
//        gBC12_DataManager(RecvBuffer);
//    else
#endif /*ONLY_PD_SNK_FUNC_EN*/
    { 
        g_Get_PDNegotiationInfo(RecvBuffer, gLogBuffer);   
        memcpy(&gI2cTransBuffer[0], &gLogBuffer[0], 128);
    }
    
}
void grl_Src_Select_PDO(pd_do_t *new_pdo,uint8_t aPdoCount)
{
    const dpm_status_t *dpm_stat = dpm_get_info (0);
    uint8_t index;
    bool pdo_found = false;
    
//    dpm_update_src_cap(0, aPdoCount, new_pdo);
 //   dpm_update_src_cap_mask(0,(dpm_stat->src_pdo_mask | (1 << index)));
    
    /* See if the new_pdo is already part of the list. */
    for (index = 0; index < aPdoCount; index++)
    {
        if (dpm_stat->src_pdo[index].val == new_pdo[index].val)
        {
            pdo_found = true;
            //break;
        }
    
        if (pdo_found)
        {
            /* PDO found. Just enable it. */
            dpm_update_src_cap_mask(0,(dpm_stat->src_pdo_mask | (1 << index)));
        }
        else
        {
            
            //dpm_update_src_cap(0, aPdoCount, new_pdo);
            //dpm_update_src_cap_mask(0, 1);
            //dpm_update_src_cap_mask(0,0x7F);
            dpm_update_src_cap_mask(0,(dpm_stat->src_pdo_mask | (1 << index)));
        }
    }

}

void InitGetBattStatusExtndMsg(uint8_t * aBuffer)
{
    uint32_t get_bat_status[1];
    dpm_pd_cmd_buf_t cmd;
    cmd.cmd_sop = SOP;
    cmd.extd_hdr.val = 0x1;
    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
    cmd.extd_type = EXTD_MSG_GET_BAT_STATUS;
       
    get_bat_status[0] = (aBuffer[6] | (aBuffer[7] << 8) | (aBuffer[8] << 16) | (aBuffer[9] << 24));
    cmd.dat_ptr = (uint8_t*)&get_bat_status[0];

    dpm_pd_command(0, DPM_CMD_SEND_EXTENDED, &cmd, NULL);  
}

void InitGetBattCapsExtndMsg(uint8_t * aBuffer)
{
    uint32_t get_bat_Caps[1];
    dpm_pd_cmd_buf_t cmd;
    cmd.cmd_sop = SOP;
    cmd.extd_hdr.val = 0x1;
    cmd.timeout = PD_SENDER_RESPONSE_TIMER_PERIOD;
    cmd.extd_type = EXTD_MSG_GET_BAT_CAP;
       
    get_bat_Caps[0] = (aBuffer[6] | (aBuffer[7] << 8) | (aBuffer[8] << 16) | (aBuffer[9] << 24));
    cmd.dat_ptr = (uint8_t*)&get_bat_Caps[0];

    dpm_pd_command(0, DPM_CMD_SEND_EXTENDED, &cmd, NULL);  
}

void g_typec_Pd_Extd_Msg_Send(uint8_t * aBuffer)
{
    switch( (aBuffer[5] & 0x0F) )
    {
        case DPM_CMD_SEND_EXTENDED:/**< 0x0D: Get_Battery_Status message. */
            InitGetBattStatusExtndMsg(aBuffer);
            g_Struct_Ptr->gLogData.isGetBattStatusInited = true;
        break;
        case 0x0E:/**< 0x0E: Get_Battery_Caps message. */
            InitGetBattCapsExtndMsg(aBuffer);
            g_Struct_Ptr->gLogData.isGetBattStatusInited = true;
        break;
        case DPM_CMD_GET_STATUS:/**< 0x0F: Send Get_Status message. */
            dpm_pd_command(0, DPM_CMD_GET_STATUS, NULL, NULL);
            g_Struct_Ptr->gLogData.isGetStatusInited = true;
            break;
    }
}

void g_update_srcs_caps(uint8_t * aBuffer)
{
    pd_do_t new_pdo[MAX_NO_OF_PDO];
    
    uint8_t aBufIndex = 6;
    
    uint8_t lPDOCount = aBuffer[5];
    //gBufLog(false,lPDOCount);
    uint32_t lPDOPayload = 0;
    uint8_t lSupplytype = 0;
    uint8_t lApilength = aBuffer[1];
    
    if(!lPDOCount)/**Pranay,14Dec'22, If no pdos are configured for SrcCaps then dont execute that API call*/
        return;
    //gBufLog(false,lApilength);
    
    //dpm_pd_cmd_buf_t lSrcCapsBuf;
    
   // lSrcCapsBuf.cmd_sop = SOP;
   // lSrcCapsBuf.no_of_cmd_do = lPDOCount;
    dpm_update_src_cap_mask(0,0x00);
    if(aBuffer[lApilength+2] == 0xCD)
    {
        for(uint8_t i = 0; i < lPDOCount; i++)
        {
            lPDOPayload = aBuffer[aBufIndex++];
            lPDOPayload |= (aBuffer[aBufIndex++] << 8 );
            lPDOPayload |= (aBuffer[aBufIndex++] << 16 );
            lPDOPayload |= (aBuffer[aBufIndex++] << 24 );
            
            lSupplytype = ((lPDOPayload & SRCSNK_CAPS_SUPPLYTYPE_OFFSET) >> SRCSNK_CAPS_SUPPLYTYPE_BYTEINDEX) ;
            
            //lSrcCapsBuf.cmd_do[i].val = lPDOPayload;
            
            new_pdo[i].val = lPDOPayload;
            switch(lSupplytype)
            {
                case PDO_FIXED_SUPPLY:
                
                    new_pdo[i].fixed_src.max_current = (lPDOPayload & 0x3FF);
                    new_pdo[i].fixed_src.voltage = ((lPDOPayload >> 10) & 0x3FF);
                    new_pdo[i].fixed_src.pk_current = ((lPDOPayload >> 20) & 0b11);
                    new_pdo[i].fixed_src.unchunk_sup = ((lPDOPayload >> 24) & 0x01);
                    new_pdo[i].fixed_src.dr_swap = ((lPDOPayload >> 25) & 0x01);
                    new_pdo[i].fixed_src.usb_comm_cap = ((lPDOPayload >> 26) & 0x01);
                    new_pdo[i].fixed_src.ext_powered = ((lPDOPayload >> 27) & 0x01);
                    new_pdo[i].fixed_src.usb_suspend_sup = ((lPDOPayload >> 28) & 0x01);
                    new_pdo[i].fixed_src.dual_role_power = ((lPDOPayload >> 29) & 0x01);
                    new_pdo[i].fixed_src.supply_type = ((lPDOPayload >> 30) & 0x03);
                break;
                case PDO_BATTERY:
                    new_pdo[i].bat_src.max_power = (lPDOPayload & 0x3FF);
                    new_pdo[i].bat_src.min_voltage = ((lPDOPayload >> 10) & 0x3FF);
                    new_pdo[i].bat_src.max_voltage = ((lPDOPayload >> 20) & 0x3FF);
                    new_pdo[i].bat_src.supply_type = ((lPDOPayload >> 30) & 0x03);
                break;
                case PDO_VARIABLE_SUPPLY:
                    new_pdo[i].var_src.max_current = (lPDOPayload & 0x3FF);
                    new_pdo[i].var_src.max_voltage = ((lPDOPayload >> 10) & 0x3FF );
                    new_pdo[i].var_src.min_voltage = ((lPDOPayload >> 20) & 0x3FF );
                    new_pdo[i].var_src.supply_type = ((lPDOPayload >> 30) & 0x03);
                break;
                case PDO_AUGMENTED:
                    new_pdo[i].pps_src.max_cur = (lPDOPayload & 0x3F);
                    new_pdo[i].pps_src.min_volt = ((lPDOPayload >> 8) & 0xFF);
                    new_pdo[i].pps_src.max_volt = ((lPDOPayload >> 17) & 0xFF);
                    new_pdo[i].pps_src.pps_pwr_limited = ((lPDOPayload >> 27 ) & 0x01);
                    new_pdo[i].pps_src.apdo_type = ((lPDOPayload >> 28) & 0x03);
                    new_pdo[i].pps_src.supply_type = ((lPDOPayload >> 30) & 0x03);
                    
                break;
            }

        }
        
        grl_Src_Select_PDO(new_pdo, lPDOCount);
        dpm_update_src_cap(0, lPDOCount, new_pdo);
        CyDelay(2);
        dpm_pd_command(0, DPM_CMD_SRC_CAP_CHNG, NULL, NULL); //Send PD command about the changes in source capabilities
        //dpm_pd_command(0, DPM_CMD_SRC_CAP_CHNG, &lSrcCapsBuf, NULL); //Send PD command about the changes in source capabilities  
    }
}
/*
*Prasanna 15May'19; CCG3PA PD Subsystem Block releated control will be handled 
*/
void g_PDSS_Config(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
   int lApilength = aBuffer[1];
    switch(aBuffer[5])
    {
        case DATA_MSG_REQUEST:
        
            if(aBuffer[lApilength+2] == 0xCD)
            {
               g_typec_Pd_Data_Msg_Send(aBuffer);
               pd_prot_send_ctrl_msg(G_PORT0, 0x00, CTRL_MSG_GET_SOURCE_CAP);//initiating GetSrcCaps here, so that the configured request packet will be initiated.
            }
            
        break;
#if ONLY_PD_SNK_FUNC_EN   
        case DATA_MSG_VDM:
        default:
        if(aBuffer[lApilength+2] == 0xCD)
        {
            g_typec_Pd_Data_Msg_Send(aBuffer);
        }
     
        break;
#else
        default:
            if(aBuffer[lApilength+2] == 0xCD)
            {
                g_typec_Pd_Data_Msg_Send(aBuffer);
            }
        break;
#endif/*ONLY_PD_SNK_FUNC_EN*/              
    }
}

#if ONLY_PD_SNK_FUNC_EN
/*
*Prasanna 15May'19; Controlling PD Submodules of CCG3PA for the GRL Application
*/
void gPDSS_Control(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint8_t CmdType = (aBuffer[6]);
    switch(CmdType)
    {
        case CableConnection_Attach:
            typec_start(G_PORT0);
            break;
        case CableConnection_Detach:
            typec_stop(G_PORT0);
            
            break;
            
        case ControlTestAutomation_Start:
            
            break;
        case ControlTestAutomation_Stop:
            //should add Start/Stop TestAutomation
            break;
        case SoftReset :
             pd_prot_send_ctrl_msg(G_PORT0, 1, CTRL_MSG_SOFT_RESET);
            break;
        case HardReset:
            //Second arg. is reason for sending Hard reset.
            dpm_send_hard_reset(G_PORT0, 1);
            break;
        
        default:
        
            break;
    }    
}
#if BC
bool gBC12_DataManager(uint8_t * lAppBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    bool lRetStatus = false;  
//    bool bc_Status;
    //const bc_status_t *bc_stat;
    bc_status_t gl_bc_status[NO_OF_BC_PORTS];
    bc_status_t *bc_stat = &gl_bc_status[BC_PORT_0_IDX];
    switch(lAppBuffer[4])
    {       
        case GRL_APP_CONFIG:
        /* In BC1.2 we will only tell FX3 that what kind of Provider is connected, which will be tracked in bc_stat->cur_mode
         * whether is it DCP SDP or CDP, based on that will set write 1,2,3 respc. 
         * in to I2C Buf. which will be read in the FX3 and hence Eload'ing(drawing current)
         * needs to be handled according to the DUT type.
         * for DCP Max Current is 1.5A, 
         * for CDP and SDP Max. Current is 500mA if configured. 
         */ 
//         bc_start(BC_PORT_0_IDX, BC_PORT_SINK);
       // bc_stat  = bc_get_status(0);
        var = bc_stat->bc_fsm_state;
         gI2cTransBuffer[1] = 7;//ByteCount
         gI2cTransBuffer[2] = 0xFC;
         gI2cTransBuffer[3] = bc_stat->bc_fsm_state;
         gI2cTransBuffer[4] = bc_stat->bc_evt ;
         gI2cTransBuffer[5] = bc_stat->connected ;
         gI2cTransBuffer[6] = bc_stat->attach ;
         gI2cTransBuffer[7] = bc_stat->cur_mode ;
       // gGet_BC12DUTConfig(lAppBuffer,var,gI2cTransBuffer);
        memset(&gI2cTransBuffer[8], 0x00, (250-8));
        break;
            
        case CableConnection_Attach:
//        bc_start(BC_PORT_0_IDX, BC_PORT_SINK);
        break; 
        case CableConnection_Detach:   
//        bc_stop(BC_PORT_0_IDX);  
        break;
        
        case GRL_APP_STATUS:
//        bc_Status = bc_is_active(0);
        gI2cTransBuffer[2] = 0xFE;
//        gI2cTransBuffer[3] = bc_Status;
        
        break;
        case 0x07:
  
        break;
        default:
        break;
    }
          lRetStatus = true;

        return lRetStatus;
}
#endif
#if 0
void gMiscHandler(uint8_t * lAppBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch (lAppBuffer[5])
    {
        case 0x01: /**Initiate UVDM message to get DUT FW version, applicable only to cypress chips, VDM_GET_DEVICE_VERSION **/
                prepareGetDevVersionPkt();
            break;
        case 0x02:/**clearing log buffer and index*/
            gBufLog(true,0);
            
        break;
        case 0x03:
            dpm_downgrade_pd_port_rev(G_PORT0);
            break;
        case 0x04:
            g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC = lAppBuffer[6];
            if( !g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC )
            {
                memset(gSOP1AckBuf,0x00,64);
                g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount = 0;
                g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_IGNORE;
                pd_phy_refresh_roles(0);
                g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
            }
            break;
        case 0x05:
            g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = lAppBuffer[6];
            break;
        case 0x06:
//            bc_init(0);
            CyDelay(10);
//            chgb_enable(0);
            break;
            
        case 0x07:
            
            #if 0
            CHGB_SINK_TERM_SPD = 0,             /**< Standard port detect */
            CHGB_SINK_TERM_PCD,                 /**< Primary charger detect. */
            CHGB_SINK_TERM_SCD,                 /**< Secondary charger detect. */
            CHGB_SINK_TERM_AFC,                 /**< AFC detect */
            CHGB_SINK_TERM_APPLE                /**< Apple detect */
                
                
            #endif
            //grl_chgb_apply_sink_term(0,lAppBuffer[6]);
            //chgb_apply_sink_term (0, lAppBuffer[6]);
            
            break;
        case 0x08:
//            chgb_remove_term(0);
            break;
        case 0x09:
//            chgb_apply_dp_pd(0);
            break;
        case 0x0A:
//            chgb_apply_sink_term (0, lAppBuffer[6]);
            break;
            
 
    }
}
#endif //0
#endif/*ONLY_PD_SNK_FUNC_EN*/

void CableConnectionSimulation(uint8_t isAttach)
{
    ccg_status_t aRetStatus;
    const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();

    switch(isAttach)
    {
        case GRL_CABLE_DETACH:
        
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            typec_stop(G_PORT0);
            CyDelay(1);
            dpm_stop (G_PORT0);
            CyDelay(1);
            dpm_typec_command(G_PORT0, DPM_CMD_PORT_DISABLE,NULL);
            CyDelay(1);
            typec_stop(G_PORT0);
            CyDelay(1);
            dpm_stop (G_PORT0);
            CyDelay(1);
            dpm_typec_command(G_PORT0, DPM_CMD_PORT_DISABLE,NULL);
            g_Struct_Ptr->RequestPacketConfig.gCustomConfig = 0;
            
            /**Pranay,22Feb21, When we detach ourselves we wont be getting any interrupt, so reverting back to def PDC after explicit detach**/        
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = false;
           
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = 1;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = 0x32;
            g_Struct_Ptr->RequestPacketConfig.gOperating_I = 0x0A;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Type = PDO_FIXED_SUPPLY;
        break;
        case GRL_CABLE_ATTACH:
            if( dpm_stat->contract_exist != PD_CONTRACT_NEGOTIATION_SUCCESSFUL)//Pranay,20Feb'2020, if already attached no need of attaching again,so if api received neglecting it.
            {
                gpio_set_value (GPIO_PORT_1_PIN_1,1);
                g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
                g_Struct_Ptr->RequestPacketConfig.gDetachFlag = false;
                //dpm_update_port_config (G_PORT0, PRT_DUAL, PRT_ROLE_SINK, true, false);
                typec_start(G_PORT0);
                CyDelay(2);
                aRetStatus = dpm_start (G_PORT0);
                
                gBufLog(false,0xA4);
                gBufLog(false,aRetStatus);
            }
        break;
        case GRL_DETACH_WITH_RP_RD_DISABLE:

            //Disable state set
            aRetStatus = dpm_typec_command(G_PORT0, DPM_CMD_PORT_DISABLE,NULL);
            CyDelay(1);
            dpm_typec_command(G_PORT0, DPM_CMD_PORT_DISABLE,NULL);
            
            gBufLog(false,0xA1);
            gBufLog(false,aRetStatus);
            CyDelay(1);
            //TypeC and dmp stop
            typec_stop(G_PORT0);
            CyDelay(1);
            aRetStatus = dpm_stop (G_PORT0);
            
            gBufLog(false,0xA2);
            gBufLog(false,aRetStatus);
            
            CyDelay(1);
            //Disable Rd on both CCs
            pd_typec_dis_rd(0,CC_CHANNEL_2);
            pd_typec_dis_rd(0,CC_CHANNEL_1);
            CyDelay(1);
            //Disable Rp on both CCs
            pd_typec_dis_rp(0,CC_CHANNEL_2);
            pd_typec_dis_rp(0,CC_CHANNEL_1);
            CyDelay(1);
            //De asserting RpRd
            dpm_typec_deassert_rp_rd(0,CC_CHANNEL_2);
            dpm_typec_deassert_rp_rd(0,CC_CHANNEL_1);
            
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            g_Struct_Ptr->RequestPacketConfig.gCustomConfig = 0;
            /**Pranay,22Feb21, When we detach ourselves we wont be getting any interrupt, so reverting back to def PDC after explicit detach**/        
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = false;
           
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = 1;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = 0x32;
            g_Struct_Ptr->RequestPacketConfig.gOperating_I = 0x0A;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Type = PDO_FIXED_SUPPLY;
            
            break;
        case GRL_ATTACH_NO_CHECK_PDC:
                gpio_set_value (GPIO_PORT_1_PIN_1,1);
                g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
                g_Struct_Ptr->RequestPacketConfig.gDetachFlag = false;
                //dpm_update_port_config (G_PORT0, PRT_DUAL, PRT_ROLE_SINK, true, false);
                typec_start(G_PORT0);
                CyDelay(2);
                aRetStatus = dpm_start (G_PORT0);
                
                gBufLog(false,0xA3);
                gBufLog(false,aRetStatus);
            break;
    }
}
void g_grl_custom_config_handler(uint8_t * lAppBuffer)
{
    switch(lAppBuffer[5])
    {
#if ONLY_PD_SNK_FUNC_EN
        case 0x04:
            g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC = lAppBuffer[6];
            if( !g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC )
            {
                memset(gSOP1AckBuf,0x00,64);
                g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount = 0;
                g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_SNK_ONLY;
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_IGNORE;
                pd_phy_refresh_roles(0);
                g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
            }
        break;
        case 0x05:
            g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = lAppBuffer[6];
        break;
#endif
        case 0x0A://OCP Hardreset
            g_Struct_Ptr->gLogData.isOCPOccurred = true;
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = true;
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            dpm_send_hard_reset(0, PD_HARDRES_REASON_VBUS_OCP);
        break;
                
        case 0x0B:
            grl_chgb_apply_sink_term(0,lAppBuffer[6],lAppBuffer);
    
        break;
            
        case GET_BATTERY_SOC_TEMP_DETAILS://0x0D
            //InitGetBattStatusExtndMsg(lAppBuffer);
            memset(&gBattStatBuf[0], 0x00, 16);
            g_Struct_Ptr->gLogData.gCustomConfig = GET_BATTERY_SOC_TEMP_DETAILS;
            dpm_pd_command(0, DPM_CMD_GET_STATUS, NULL, NULL);
        break;
            
        case 0xF1://If we want to update sink caps in runtime based on DUT caps
            g_Struct_Ptr->RequestPacketConfig.isRuntime_SnkCapsConfigEnabled = lAppBuffer[6];
        break;
        case 0xF2:
            dpm_downgrade_pd_port_rev(G_PORT0);
        break;
    }
}

/*
Pranay,07Jun'19, Adding PD related setting/getting info here.
lRetStatus :: should be false for every case other than GRL_APP_STATUS in which we'll be reading DUTs data.
*/
bool gPD_DataManager(uint8_t * lAppBuffer)
{
   // const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    bool lRetStatus = false;
   
    switch(lAppBuffer[4])
    {
        case GRL_CABLE_DETACH :
            
            CableConnectionSimulation(GRL_CABLE_DETACH);

        break;
        case GRL_CABLE_ATTACH :
            
            CableConnectionSimulation(GRL_CABLE_ATTACH);

        break;
        case GRL_TesterMode_SINK ://0x03
            grl_qc4_confighandler(0,lAppBuffer);
         
        break;
            
        case GRL_TesterMode_SRC://0x04
            gBufLog(true,0);
            gBufLog(false,0xA5);
            gBufLog(false,lAppBuffer[4]);
            g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag = GRL_PD_MODE_SET;// Venkat 16'NOV'2022 ,setting PD mode flag
            gpio_set_value (GPIO_PORT_2_PIN_1,0);
            gRetryCount = 0;
            g_Struct_Ptr->gCustomConfig.gReceivedSetPortRoleCmd = PRT_ROLE_SOURCE;
            
            schedule_task(2,GRL_PORT_DETACH);
            
        break;
            
        case GRL_TesterMode_DRP://0x06

            gpio_set_value (GPIO_PORT_2_PIN_1,0);
            gRetryCount = 0;
            g_Struct_Ptr->gCustomConfig.gReceivedSetPortRoleCmd = PRT_DUAL;
            g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag = GRL_PD_MODE_SET;// Venkat 16'NOV'2022 ,setting PD mode flag
            
            schedule_task(2,GRL_PORT_DETACH);
      
            break;

#if ONLY_PD_SNK_FUNC_EN            

        //case GRL_CCLines_Handle :
         //g_typec_Resistance_Set(lAppBuffer);
        
        //break;

        case GRL_CAPABILITY_MISMATCH_HANDLE://0xF1
         g_CapabilityMismatchHandler(lAppBuffer);
        break;
#endif /*ONLY_PD_SNK_FUNC_EN*/
        case GRL_APP_CONFIG ://0x05
            g_PDSS_Config(lAppBuffer);
        break;
        case GRL_SRC_CAPS_UPDATE://0x20
            g_update_srcs_caps(lAppBuffer);
        break;
        
        case GRL_APP_CMD_INIT ://0x10
            
            if(((lAppBuffer[5] & 0xF0) >> 4 ) == 0x01)//If Ctrlmsg needs to be initated
            {
               g_typec_Pd_Ctrl_Msg_Send(lAppBuffer); 
            }
            else if(((lAppBuffer[5] & 0xF0)>>4)== 0x02)//If Extnd msg needs to be initated
            {
                g_typec_Pd_Extd_Msg_Send(lAppBuffer);
            }
            break;
        case GRL_CUSTOM_CONFIG_HANDLE://0xF2
            g_grl_custom_config_handler(lAppBuffer);
        break;
        
#if ONLY_PD_SNK_FUNC_EN        
/*        case GRL_APP_CMD_INIT ://0x10
        
            if(((lAppBuffer[5] & 0xF0)>>4) == 0x01)//If Ctrlmsg needs to be initated
            {
               g_typec_Pd_Ctrl_Msg_Send(lAppBuffer); 
            }
            else if(((lAppBuffer[5] & 0xF0)>>4)== 0x02)//If Extnd msg needs to be initated
            {
//                g_typec_Pd_Extd_Msg_Send(lAppBuffer);
            }
            else if(((lAppBuffer[5] & 0xF0)>>4)== 0x00)//If data msg needs to be initated
            {    
            }
            else//If need to Initiate any other commands
            {
               // gPDSS_Control(lAppBuffer);
            }
         break;   


        case GRL_MISC_HANDLE ://0xF2
            gMiscHandler(lAppBuffer);
        break;*/
#endif/*ONLY_PD_SNK_FUNC_EN*/    
    }    
    return lRetStatus;
}
/**/

bool gDataManager(uint8_t * lAppBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    bool lRetStatus = false;
#if ONLY_PD_SNK_FUNC_EN
    /**Pranay, 16Jul'21,Restarting timer if we get any API from user/application/TC expecting that received API will be handled in 200mS max
    * if Detach/Attach cmd is received - this timer will be handled in appevt handler itself.  
    */
    if( (g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType == CCG_AS_CABLE_TESTER )&&
        (!g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady) && 
        (g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC)
       )
    {
        timer_stop(0,GRL_APP_SOP1_TIMER);
        schedule_task(200, GRL_APP_SOP1_TIMER);
    }
#endif /**ONLY_PD_SNK_FUNC_EN*/    

    if((lAppBuffer[0] & 0x0F)== 0x07)//Get
    {

       g_PDSS_Status(lAppBuffer);
            
            lRetStatus = true;
                   
            gI2cTransBuffer[0] = 0xFA;
    }
    else if((lAppBuffer[0] & 0x0F) == 0x01)//Set
    {
        switch(lAppBuffer[3])
        {
            case GRL_PD_SCOPE:
            
            lRetStatus = gPD_DataManager(lAppBuffer);
              
            break;
#if ONLY_PD_SNK_FUNC_EN
            case GRL_BC12_SCOPE:
            
//            lRetStatus =  gBC12_DataManager(lAppBuffer);
            
            break;
#endif /**ONLY_PD_SNK_FUNC_EN*/    

            default:
                
                break;
            
        }
    }
    else if((lAppBuffer[0] & 0x0F) == 0x02)//Programming
    {
        switch(lAppBuffer[3])
        {
            case 0x0F://Switch back to the Boot loader
            
            Bootloadable_1_Load();
              
            break;  
            
            /**Pranay,29Sept'20, For updating DUTs FW through CC lines using uVDMs, sequence will come under FW update section so implementing here aswell */
            case GRL_PD_SCOPE :/**0x03*/
            
            lRetStatus = gPD_DataManager(lAppBuffer);
            
            break;
        }
    }
    return lRetStatus;
}
/*
*Prasanna 15May'19;I2c Communication task handle, I2C Data received from the master will be handled here
*/
void gI2cHandle_task()
{   
    bool isRead = false;
   
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    if((0 != gI2cTransBuffer[0]) && (0xFA != gI2cTransBuffer[0]))
    {
        isRead = gDataManager(gI2cTransBuffer);
        if(!isRead)
            memset(&gI2cTransBuffer[0], 0x00, 250);
    }  
}

/*
*Prasanna 15May'19; i2c Slave SCB initialization for communication between FX3 and CCG3PA
*/
void i2cDevice_Init()
{    
    EZI2C_1_Start();
    
    EZI2C_1_EzI2CSetBuffer1(512, 250, gI2cTransBuffer);
    
    
}

/*
*Prasanna 15May'19; GRL Application Initializtion, SCB's, GPIO's are initialized and the system default configuration states are initialized here
*/
void grlapp_Initialization()
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    i2cBuf[0] = 0x00;
    i2cBuf[1] = 0x00;
    g_Struct_Ptr->RequestPacketConfig.gDetachFlag  = true;
     
    g_Struct_Ptr->RequestPacketConfig.gPDO_Index = 1;
    g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = 0x32;
    g_Struct_Ptr->RequestPacketConfig.gOperating_I = 0x0A;
    g_Struct_Ptr->RequestPacketConfig.gPDO_Type = PDO_FIXED_SUPPLY;
    
    g_Struct_Ptr->RequestPacketConfig.isRuntime_SnkCapsConfigEnabled = false;
#if ONLY_PD_SNK_FUNC_EN

    g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
    g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = false;
    g_Struct_Ptr->RequestPacketConfig.gCustomConfig = 0;
    
    g_Struct_Ptr->gPDSSConfigCtrl.gDrSwapSent = false;
    
    g_Struct_Ptr->gPDSSConfigCtrl.gFromBufIndex = 0;
    
    g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_IGNORE;
  
    g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC = false;
    g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
    
    
    memset(&gBattStatBuf[0], 0x00, 16);
      /*PDflag initialy set ,for PD contract negotiation when attach is observed*/
    g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag = PDMODE;

    bc_init(0);
    CyDelay(200);
    chgb_enable(0);
    CyDelay(200);
    chgb_remove_term(0);
    /**Pranay,14Dec'22,Variables related to APDO handling to be set to default state**/
    g_Struct_Ptr->RequestPacketConfig.gReqSupplyType = 0x00;
    g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusVoltage = 0x00;
    g_Struct_Ptr->RequestPacketConfig.gPrevReqAPDOVbusCurrent = 0x00;
    g_Struct_Ptr->RequestPacketConfig.isNewAPDORequest = false;
#endif /**ONLY_PD_SNK_FUNC_EN*/

}

void grlSystem_init()
{   
    i2cDevice_Init();
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    grlapp_Initialization();
    /**Pranay,22Aug'19, Changed default drive mode to Pull-up from pull-down beacause FX3 is configured for Active Low interrupt*/
    gpio_hsiom_set_config (GPIO_PORT_1_PIN_2, HSIOM_MODE_GPIO, GPIO_DM_RES_UP, true);
    
    gpio_hsiom_set_config (GPIO_PORT_1_PIN_1, HSIOM_MODE_GPIO, GPIO_DM_RES_UP, true); /**Pranay,19Sept'19,Gpio Added for checking the throughput of API's */
    gpio_hsiom_set_config (GPIO_PORT_2_PIN_1, HSIOM_MODE_GPIO, GPIO_DM_RES_UP, true); 
}

/*GRL Custom QC implementations*/
/* venkat 6Jul'22 
   GRL custom QC3 D+/D- pulse generation function , From current voltage anf required voltage from API
   no.of pulses needed is calculated and task is scheduled to generate pulse*/
ccg_status_t grl_chgb_QC3(uint8_t *lAppBuffer)  
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint16_t tvol;
    g_Struct_Ptr->gPDSSConfigCtrl.gReqPulse_Count = 0;
    g_Struct_Ptr->gPDSSConfigCtrl.gCurrent_Pulsecnt = 0;
    uint16_t current_vbus;    
    uint16_t diffVoltage;
    current_vbus = vbus_get_value(0);
    tvol = ( (lAppBuffer[8] << 8) | lAppBuffer[7] );
    /*inc vbus by calculating required pulse count from present 
    and target voltage and schediling task for pulse generation*/
    if(current_vbus < tvol)
    {   
        g_Struct_Ptr->gPDSSConfigCtrl.gTarget_Voltage  = tvol - current_vbus;
        diffVoltage  = tvol - current_vbus;
      

        g_Struct_Ptr->gPDSSConfigCtrl.gReqPulse_Count = (diffVoltage/GRL_QCpulse_voltage) ;
        schedule_task(1, GRL_DP_PULSE);
        
    }
    /*dec vbus by calculating required pulse count from present
    and target voltage and schediling task for pulse generation*/
    else if(current_vbus > tvol)
    {
        g_Struct_Ptr->gPDSSConfigCtrl.gTarget_Voltage = current_vbus - tvol;
        diffVoltage = current_vbus - tvol;

        g_Struct_Ptr->gPDSSConfigCtrl.gReqPulse_Count = (diffVoltage/GRL_QCpulse_voltage) ;
        schedule_task(1, GRL_DM_PULSE);
    }
    else
    {
        //no pulses
    }
    
   return CCG_STAT_SUCCESS;     
}

/*  venkat 11Jul'22
    This is GRL Custom function for setting QC2.0 voltages as per spec changing D+ and D- lines therfore source will provide 5V,9V,12V,20V.
*/
ccg_status_t grl_qc2_sink(uint8_t cport,uint8_t *lAppBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(lAppBuffer[7])
    {
        case GRL_QC_REMOVE_TERM:
            chgb_remove_term(cport);
        break;
        case GRL_QC_5V:

            /* 0.6 on D+, D- HiZ */
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            
            /* Remove other terms */
            PDSS->bch_det_0_ctrl[cport] &= ~ (PDSS_BCH_DET_0_CTRL_RDP_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
                                    PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
                                    PDSS_BCH_DET_0_CTRL_IDM_SNK_EN);
           
        break;
        case GRL_QC_9V:

            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDP_PU_EN;
            
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDM_SRC_EN;
            

            /* Remove other terms */
            PDSS->bch_det_0_ctrl[cport] &= ~ ( PDSS_BCH_DET_0_CTRL_RDM_PU_EN |
                                PDSS_BCH_DET_0_CTRL_VDP_SRC_EN);
        break;
        case GRL_QC_12V:

            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDP_SRC_EN;
            
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_VDM_SRC_EN;
            
             /* Remove other terms */
            PDSS->bch_det_0_ctrl[cport] &= ~(PDSS_BCH_DET_0_CTRL_RDP_PU_EN|
                                    PDSS_BCH_DET_0_CTRL_RDM_PU_EN);
        break;
        case GRL_QC_20V:
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDP_PU_EN;
            
            PDSS->bch_det_0_ctrl[cport] |= PDSS_BCH_DET_0_CTRL_RDM_PU_EN;
            
            /* Remove other terms */
            PDSS->bch_det_0_ctrl[cport] &= ~ ( PDSS_BCH_DET_0_CTRL_VDM_SRC_EN |
                                PDSS_BCH_DET_0_CTRL_VDP_SRC_EN);
        break;
            
    }
    return CCG_STAT_SUCCESS;
}

/* venkat 6Jul'22
   TaskID - V-1-T314 - QC4.0 implementation,
   Switch for user from PD mode to qc mode,if Qc mode we can switch for QC2.0 or Qc3.0 modes*/
ccg_status_t grl_qc4_confighandler(uint8_t cport,uint8_t *lAppBuffer) 
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();            
    
    switch(lAppBuffer[5])
    {
        case GRL_PD_MODE_SET://only PD,0x01
        default:
            /* Pranay,23Nov'22,DpDm terminations shall be removed 
             * before disabling QC mode to make state machine to 
             * default state so that DUT also considers that there is a detach 
             */
            chgb_remove_term(cport);
            gRetryCount = 0;

            g_Struct_Ptr->gCustomConfig.gReceivedSetPortRoleCmd = PRT_ROLE_SINK;
            /* Pranay,23Nov'22, 
            *  After removing terminations, it is taking 1.5Secs for terminations 
            *  to be properly removed and for source to detect respective changes
            *  (based on observation from scope captures with duracell DUT)
            *  4.5sec Timer shall be started only if QC mode was enabled earlier 
            *  else start 2ms timer just to detach and handle rest scenario as similar to SRC/DRP mode set
            */
            
            if(g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag != GRL_PD_MODE_SET)
                schedule_task(1500,GRL_PORT_DETACH);
            else
                schedule_task(2,GRL_PORT_DETACH);
        break;
            
        case GRL_QC_MODE_SET://Only QC Enable,0x02
            
            /*   venkat 11Jul'2022,Detach task, 
             *   Specifically used for QC mode change from PD mode 
             *   where sink need to get detached in order to undergo BC enumeration sequence
             *   where it is the first sequence in QC mode as per the spec
             **/
            g_Struct_Ptr->gPDSSConfigCtrl.gQC4_3_ConfigFlag = lAppBuffer[6];
            CableConnectionSimulation(GRL_CABLE_DETACH);
            CyDelay(5);
            SinkConfiguration();//Venkat,18Nov'22, Inorder to handle transiting from PD Src mode to QC Sink mode fist we need to change CCG3 mode to Sinkmode then only QC mode can be set
            schedule_task(650,GRL_QC_ATTACH);
            
        break;
 
    }
    return CCG_STAT_SUCCESS; 
}
const grl_Struct_t * get_grl_struct_ptr()
{
    return (&gStruct_t);
}
