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

#include <grlabstraction.h>
#include <grlapp.h>

#define ATTRIBUTES

static uint16_t glogBufIndex;
dpm_pd_cmd_buf_t vdm_cmd_buf;

uint8_t glFirmwareID[24] __attribute__ ((aligned(32))) = "1.1.1";
#if ONLY_PD_SNK_FUNC_EN
void g_CapabilityMismatchHandler(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(aBuffer[5])
    {
        case 0://Disable
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = false;
            g_Struct_Ptr->RequestPacketConfig.gCustomConfig = 0;
        break;
        case 1://enable
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = true;
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = true;
            g_Struct_Ptr->RequestPacketConfig.gCustomConfig = gAptivDetachOverride;
        break;
    }
    
}
#endif/*ONLY_PD_SNK_FUNC_EN*/

void g_fill_datalog_buffer_2Bytes( uint16_t lvalue, uint8_t * aBuffer)
{
    aBuffer[glogBufIndex++] = lvalue;
    aBuffer[glogBufIndex++] = lvalue >> 8;
}

/* Pranay 07Jun'19, Function to know the Active CC line */
uint8_t g_ActiveCC(uint8_t * aBuffer)
{   
    uint8_t ret =0;
    ret = dpm_get_polarity(G_PORT0); 
    return ret;
}

/*
Pranay,07June'19, Function for Sending CtrlMsgs as per listed Enums,
G_PORT0 ::Port no. (Rightnow Hardcoded to G_PORT0),
g_Sop_type :: Sop type with which the msg is to be initiated,(Lower nibble as SOP type)
g_CtrlMsg_type :: CtrlMsgType to be initiated,(Upper nibble as MsgType)
*/
void g_typec_Pd_Ctrl_Msg_Send(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint8_t g_Sop_type = (aBuffer[5] & 0x0F); //b[3:0] as SOP type.
    uint8_t g_CtrlMsg_type = (aBuffer[6]);
    switch(g_CtrlMsg_type)
    {

        default:/**All other commands will be initiated using g_CtrlMsg_type, by passing respective Enum as per structure**/
            /** Pranay,27Sept'19,to resolve issue i.e., after requesting PDOx if we send GetSrcCaps data msg request packet was being sent with 
            PDO1 so Added this flag here to avoid requesting PDO1 by default afer requesting PDOx particularly */
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = true;
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
            pd_prot_send_ctrl_msg(G_PORT0, g_Sop_type, g_CtrlMsg_type);
        break;
    }

  
}
#if ONLY_PD_SNK_FUNC_EN
void g_typec_Resistance_Set(uint8_t * aBuffer)
{ 
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    switch(aBuffer[5])
    {
        case 1://typec_Disable Rd on CC1
            pd_typec_en_rd(G_PORT0, CC_CHANNEL_2);//Enabling Rd on only CC2
            pd_typec_dis_rd(G_PORT0, CC_CHANNEL_1);
            pd_typec_dis_rp(G_PORT0, CC_CHANNEL_1);
            break;
        
        case 2://typec_Disable Rd on CC2
            pd_typec_en_rd(G_PORT0, CC_CHANNEL_1);//Enabling Rd on only CC1
            pd_typec_dis_rd(G_PORT0, CC_CHANNEL_2);
            pd_typec_dis_rp(G_PORT0, CC_CHANNEL_2);
            break;
    }

}

/**Pranay,01March'21, This function initiates uVDM packet for fetching VDM_GET_DEVICE_VERSION(DUT FW version)*/
void prepareGetDevVersionPkt()
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    static dpm_pd_cmd_buf_t vdm_cmd_buf ;
    vdm_cmd_buf.cmd_sop = SOP;
    vdm_cmd_buf.timeout = 120;
    vdm_cmd_buf.no_of_cmd_do = 1;
    
    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR;
    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.cmd = VDM_CMD_DSC_IDENTITY;
    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.vdm_ver = 0x02;
    vdm_cmd_buf.cmd_do[0].ustd_vdm_hdr.vdm_type = VDM_TYPE_UNSTRUCTURED;
    
    
    vdm_cmd_buf.cmd_do[0].val = 0x04B40002;/** 04B4:VID, VDM_GET_DEVICE_VERSION code word = 0x02*/
    
    g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit = true;
    dpm_pd_command(G_PORT0, DPM_CMD_SEND_VDM,&vdm_cmd_buf, NULL);
    
}

void g_VDMPacketBuffFill(uint8_t * aBuffer)
{
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    static dpm_pd_cmd_buf_t vdm_cmd_buf ;
    uint8_t VdmType = (aBuffer[6] & 0x0F);/**VdmType; 0 :: Unstructured, 1 :: structured **/
    
    vdm_cmd_buf.cmd_sop = (aBuffer[6] & 0xF0) >> 4;/*SOP TYPE*/
    
    uint8_t lDataObjectsCnt = aBuffer[7];/*No of Dataobjects*/
    
    vdm_cmd_buf.no_of_cmd_do = lDataObjectsCnt;
    vdm_cmd_buf.timeout = 120;

    uint8_t lBufIndex = 8;
    uint32_t vdoVal;
 
    switch(VdmType)
    {
        case VDM_TYPE_UNSTRUCTURED:
           
            for(uint8_t index = 0; index < lDataObjectsCnt; ++index)
            {
                vdm_cmd_buf.cmd_do[index].ustd_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR;
                vdm_cmd_buf.cmd_do[index].ustd_vdm_hdr.cmd = VDM_CMD_DSC_IDENTITY;
                vdm_cmd_buf.cmd_do[index].ustd_vdm_hdr.vdm_ver = 0x02;
                vdm_cmd_buf.cmd_do[index].ustd_vdm_hdr.vdm_type = VDM_TYPE_UNSTRUCTURED;

                vdoVal = 0;

                for(uint8_t i = 0,j = 0 ; i < 4; ++i,++lBufIndex)
                {
                    vdoVal |= (aBuffer[lBufIndex] << j); 
                    vdm_cmd_buf.cmd_do[index].val = vdoVal;
                    j += 8;
                }
            }
            g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit = true;
            dpm_pd_command(G_PORT0, DPM_CMD_SEND_VDM,&vdm_cmd_buf, NULL);
            
        break;
            
        case VDM_TYPE_STRUCTURED:
            /**Decdoing configured header from Application*/
            if(vdm_cmd_buf.cmd_sop != SOP)
            {   
                /**Incrementing message ID for internal tracking*/
                ++g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID;
                g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader = (aBuffer[lBufIndex]);lBufIndex++;
                
                /**Configuring message ID based on received and prev. initiated count, Header'ss [11:9] bits indicates Message ID*/
                aBuffer[lBufIndex] |= (g_Struct_Ptr->gPDSSConfigCtrl.gSOP1MsgID << 1);
                
                g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader |= ((aBuffer[lBufIndex] << 8)); lBufIndex++;

            }
            /**decoding received D O payload from application*/
            for(uint8_t index = 0; index < lDataObjectsCnt; ++index)
            {
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.cmd_type = CMD_TYPE_INITIATOR;
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.cmd = VDM_CMD_DSC_IDENTITY;
                vdm_cmd_buf.cmd_do[0].std_vdm_hdr.vdm_type = VDM_TYPE_STRUCTURED;
                vdm_cmd_buf.cmd_do[index].std_vdm_hdr.obj_pos = index;
             
                vdoVal = 0;

                for(uint8_t i = 0,j = 0 ; i < 4; ++i,++lBufIndex)
                {
                    vdoVal |= (aBuffer[lBufIndex] << j); 
                    vdm_cmd_buf.cmd_do[index].val = vdoVal;
                    j += 8;
                }
            }
            /**Initiating packet*/
            if(vdm_cmd_buf.cmd_sop != SOP)
            {
                /**Pranay,16Jul;21,Stop this timer if we have to send SOP1 packet explicitly so that ACK response will be tracked and stored in a local buffer*/
                if(g_Struct_Ptr->gPDSSConfigCtrl.gStartSOP1DiscIDAfterPDC)
                    timer_stop(0,GRL_APP_SOP1_TIMER);
                    
                /**Here we're using PHY level APIs so Header needs to configured carefully. If phy level APIs are used interrupts are not passed to app layer so decode them in pdss_mx_hal.c itself and then handle accordingly in main while loop*/
                /**Shall set this variable appropriately to get and decode interrupt*/
                g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_CABLE_TESTER;
                g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady = false;
                g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = RES_TYPE_ACK;
                
                pd_phy_load_msg(0,vdm_cmd_buf.cmd_sop,1,vdm_cmd_buf.no_of_cmd_do, g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader , false, (uint32_t *)vdm_cmd_buf.cmd_do);
                CyDelayUs(1000);/**neccessary delay as suggested by cypress*/
                pd_phy_send_msg(0);
                pd_phy_refresh_roles(0);/**Should call this if not we'll not receive any SOP1 responses*/
            }
            else
            {
                g_Struct_Ptr->gPDSSConfigCtrl.gVdmInit = true;/**If Vdms are initiated using dpm_pd_commnd, events are received in app_evt_handler() else not. So VdmInit Flag is used to decode packets when received*/
                dpm_pd_command(G_PORT0, DPM_CMD_SEND_VDM,&vdm_cmd_buf, NULL);
            }
        break;
    }

}
static uint8_t fromIndex; /**Actual VDOsinfo starts from  index 9 */

void FillVdmInfo4Bytes(pd_do_t * vdm_cmd_buf, uint8_t * aBuffer )
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    vdm_cmd_buf -> val =  (aBuffer[fromIndex++]);
    vdm_cmd_buf -> val |=  (aBuffer[fromIndex++] << 8);
    
    vdm_cmd_buf -> val |= (aBuffer[fromIndex++] << 16);
    vdm_cmd_buf -> val |= (aBuffer[fromIndex++] << 24);
    
}
void ConfigCableResponse(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    /**Shall set this variable appropriately to get and decode interrupt*/
    g_Struct_Ptr->gPDSSConfigCtrl.gVUPStateMachineType = CCG_AS_CABLE_AND_SNK;
    
    g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig = aBuffer[6] & 0x0F ;/**0:ignore, 1:ACK,2:NAK**/

    g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs = ((aBuffer[6] & 0xF0) >> 4);
    
    /***tracking configured header from application*/
    g_Struct_Ptr->gPDSSConfigCtrl.gVDMHeader = ( aBuffer[7] | (aBuffer[8] << 8)) ;
    
    fromIndex = 9; /**Actual VDOsinfo starts from  index 9 */
    
    if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig == RES_TYPE_NAK)
    {
        memset(&vdm_cmd_buf, 0x00, sizeof(dpm_pd_cmd_buf_t));
        CyDelay(2);
        FillVdmInfo4Bytes(&vdm_cmd_buf.cmd_do[0], aBuffer);/** 9-12 : 4Bytes */
    }
    else if(g_Struct_Ptr->gPDSSConfigCtrl.Cable_ResponseConfig == RES_TYPE_ACK)
    {
        for(uint8_t i =0; i < g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs; ++i)
        {
            FillVdmInfo4Bytes(&vdm_cmd_buf.cmd_do[i], aBuffer);/***/
        }
    }
    else
    {
        memset(&vdm_cmd_buf, 0x00, sizeof(dpm_pd_cmd_buf_t));
    }
    
}
#endif /*ONLY_PD_SNK_FUNC_EN*/
void ConfigSnkCaps(uint8_t * aBuffer)
{
    pd_do_t snk_pdo[MAX_NO_OF_PDO];
    
    uint8_t aBufIndex = 7;
    
    uint8_t lPDOCount = aBuffer[6];
    
    uint32_t lPDOPayload = 0;
    uint8_t lSupplytype = 0;

    const dpm_status_t *dpm_stat = dpm_get_info (0);
    
    dpm_update_snk_cap_mask(0x00, 0x00);
    
    for(uint8_t i = 0; i < lPDOCount; i++)
    {
        lPDOPayload = aBuffer[aBufIndex++];
        lPDOPayload |= (aBuffer[aBufIndex++] << 8 );
        lPDOPayload |= (aBuffer[aBufIndex++] << 16 );
        lPDOPayload |= (aBuffer[aBufIndex++] << 24 );
        
        lSupplytype = ((lPDOPayload & SRCSNK_CAPS_SUPPLYTYPE_OFFSET) >> SRCSNK_CAPS_SUPPLYTYPE_BYTEINDEX) ;
        snk_pdo[i].val = lPDOPayload;
        //lCurrentPDOSupplytype = src_cap->dat[i].fixed_src.supply_type;
        switch(lSupplytype)
        {
           case PDO_FIXED_SUPPLY:
            
            
            snk_pdo[i].fixed_snk.op_current = (lPDOPayload & 0x3FF);//10mA units, B9..0
            snk_pdo[i].fixed_snk.voltage = ((lPDOPayload >> 10) & 0x3FF);//50mV units, b19...10
            //20..22 Reserved
            snk_pdo[i].fixed_snk.fr_swap = ((lPDOPayload >> 23) & 0b11);//B24...23
            snk_pdo[i].fixed_snk.dr_swap = ((lPDOPayload >> 25) & 0x01);//B25
            snk_pdo[i].fixed_snk.usb_comm_cap = ((lPDOPayload >> 26) & 0x01);//B26
            snk_pdo[i].fixed_snk.ext_powered = ((lPDOPayload >> 27) & 0x01);//B27
            snk_pdo[i].fixed_snk.high_cap = ((lPDOPayload >> 28) & 0x01);//B28
            snk_pdo[i].fixed_snk.dual_role_power = ((lPDOPayload >> 29) & 0x01);//B29
            snk_pdo[i].fixed_snk.supply_type = ((lPDOPayload >> 30) & 0x03);//B31..30
            
            //snk_pdo[i].fixed_snk.voltage = src_cap->dat[i].fixed_src.voltage;
            break;
            
           case PDO_VARIABLE_SUPPLY:
            
            snk_pdo[i].var_snk.op_current = (lPDOPayload & 0x3FF);//in 10mA units, B9..0
            snk_pdo[i].var_snk.min_voltage = ((lPDOPayload >> 10) & 0x3FF );//in 50mV units, B19...10 
            snk_pdo[i].var_snk.max_voltage = ((lPDOPayload >> 20) & 0x3FF );//in 50mV units, B29...20
            snk_pdo[i].var_snk.supply_type = ((lPDOPayload >> 30) & 0x03);//B31..30
            break;
            
           case PDO_BATTERY:
            
            snk_pdo[i].bat_snk.op_power = (lPDOPayload & 0x3FF);//in 250mW units, B9..0
            snk_pdo[i].bat_snk.min_voltage = ((lPDOPayload >> 10) & 0x3FF);//in 50mV units, B19...10
            snk_pdo[i].bat_snk.max_voltage = ((lPDOPayload >> 20) & 0x3FF);//in 50mV units, B29...20
            snk_pdo[i].bat_snk.supply_type = ((lPDOPayload >> 30) & 0x03);//B31...30
            break;
            
           case PDO_AUGMENTED:
            
            snk_pdo[i].pps_snk.op_cur = (lPDOPayload & 0x3F);//50mA incr., B6..0
            snk_pdo[i].pps_snk.min_volt = ((lPDOPayload >> 8) & 0xFF);//100mV incr, B15...8
            snk_pdo[i].pps_snk.max_volt = ((lPDOPayload >> 17) & 0xFF);//100mV incr, B24...17
            snk_pdo[i].pps_snk.apdo_type = ((lPDOPayload >> 28) & 0x03);//B29...28-> should be 00b-PPS
            snk_pdo[i].pps_snk.supply_type = ((lPDOPayload >> 30) & 0x03);//B31...30-> should be 11b-APDO
            break;
            
        }        
        
        dpm_update_snk_cap_mask(0,(dpm_stat->snk_pdo_mask | (1 << i)));
        
        dpm_update_snk_cap(0,lPDOCount, snk_pdo);
    }
}
/*
Pranay,07June'19, Function for Sending DataMsgs as per listed Enums,
G_PORT0 ::Port no. (Rightnow Hardcoded to G_PORT0),
g_Sop_type :: Sop type with which the msg is to be initiated,(Lower nibble as SOP type)
g_DataMsg_type :: CtrlMsgType to be initiated,(Upper nibble as MsgType)
*/
void g_typec_Pd_Data_Msg_Send(uint8_t * aBuffer)
{
    #if 0
    DATA_MSG_SRC_CAP = 1,               /**< 0x01: Source_Capabilities message. */
    DATA_MSG_REQUEST,                   /**< 0x02: Request message. */
    DATA_MSG_BIST,                      /**< 0x03: BIST message. */
    DATA_MSG_SNK_CAP,                   /**< 0x04: Sink_Capabilities message. */
    DATA_MSG_BAT_STATUS,                /**< 0x05: Battery_Status message. */
    DATA_MSG_ALERT,                     /**< 0x06: Alert message. */
    DATA_MSG_GET_COUNTRY_INFO,          /**< 0x0F: Vendor_Defined message. */
    DATA_MSG_VDM = 15                   /**< 0x0F: Vendor_Defined message. */
    #endif

    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
        
    uint8_t l_PDO_Type = ((aBuffer[6] & 0xF0) >> 4); //Upper nibble as PDO type.
    uint8_t l_PDO_Index = (aBuffer[6] & 0x0F); //Lower nibble as PDO Index.

    uint8_t l_DataMsg_type = aBuffer[5];

    uint32_t lMax_Op_I = ((aBuffer[8] << 8) | (aBuffer[7]));
    uint32_t lOperating_I = ((aBuffer[10] << 8) | (aBuffer[9]));
    uint8_t lparameter = 10;//default case

    
   switch(l_DataMsg_type)
    {
        case DATA_MSG_SRC_CAP: /**< 0x01: Source_Capabilities message. */
        
        break;
        case DATA_MSG_REQUEST:/**< 0x02: Request message. */
          g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
          // gPrepareRequestPkt(g_PDO_type, false, false, Operating_I, Max_Op_I,g_DataMsg_type/*, app_resp_handler*/);//app_resp_handler   
         // grl_RequestPacketConfig(l_PDO_Index,l_PDO_Type, lMax_Op_I, lOperating_I,lparameter);
          g_Struct_Ptr->RequestPacketConfig.gOperating_I = lOperating_I;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = lMax_Op_I;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = l_PDO_Index;
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = 1;
            g_Struct_Ptr->RequestPacketConfig.gPDO_Type = l_PDO_Type;

            grl_RequestPacketConfig(g_Struct_Ptr->RequestPacketConfig.gPDO_Index,
                                    g_Struct_Ptr->RequestPacketConfig.gPDO_Type, 
                                    g_Struct_Ptr->RequestPacketConfig.gMax_Op_I, 
                                    g_Struct_Ptr->RequestPacketConfig.gOperating_I,
                                    lparameter);
      
        break;
        case DATA_MSG_SNK_CAP: /**< 0x04: Sink_Capabilities message. */
            ConfigSnkCaps(aBuffer);
        break;
#if ONLY_PD_SNK_FUNC_EN
        case DATA_MSG_BIST:/**< 0x03: BIST message. */
        
        break;
 
        case DATA_MSG_ALERT: /**< 0x06: Alert message. */
        
        break;
        case DATA_MSG_BAT_STATUS:/**< 0x05: Battery_Status message. */
        
        break;
        case DATA_MSG_GET_COUNTRY_INFO:/**< 0x0F: Vendor_Defined message. */
        
        break;

        case DATA_MSG_VDM:/**< 0x0F: Vendor_Defined message. */
 
            g_VDMPacketBuffFill(aBuffer);
         //set_disc_param(G_PORT0, g_Sop_type,  VDM_CMD_DSC_IDENTITY);
        break;
            
        case DATA_MSG_ACK : /**< 0xF1: Vendor_Defined Ack message. */
            /**V-UP as cable + Snk emulation*/
            /**By Default if SPL cable is used and VUP received any DiscID SOp1/SOP11 packets, VUP will IGNORE packets untill configured explictly from API-SW as NAK/ACK**/
            ConfigCableResponse(aBuffer);
            
        break;
#endif /*ONLY_PD_SNK_FUNC_EN*/
        default:
        
        break;      
    }

}
#if 0
void g_typec_Pd_Extd_Msg_Send(uint8_t * aBuffer)
{
    
    switch(aBuffer[3])
    {
        
    }

}



void g_typec_Pd_Msg_Send(uint8_t * aBuffer)
{
    switch(aBuffer[2])
    {
        case Data_Msg://1
             g_typec_Pd_Data_Msg_Send(aBuffer);
            break;
        case Ctrl_Msg://2
           g_typec_Pd_Ctrl_Msg_Send(aBuffer);
            break;
        case Extd_Msg://3
            g_typec_Pd_Extd_Msg_Send(aBuffer);
            break;
        default:
            
            break;
    }
    
}
#endif
/*
* Copying Every PDO's Max. and Min. Voltages,Currents into the buffer based on the Supply type,
*   Currents and voltages are of 2 bytes each.
*/
void g_decode_SRC_PDOs(uint8_t src_pdo_index,const pd_do_t* pdo_src, uint8_t * aBuffer)
{
   g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint32_t fix_volt = 0, max_volt =0, min_volt =0;
    uint32_t fix_current =0 ,max_current =0;
    
    /* copying Supply type in Upper nibble and PDO index in lower nibble to byte [3] index*/
     aBuffer[glogBufIndex++] = ((pdo_src->fixed_src.supply_type << 4) | ( src_pdo_index+1 ));
  
   
    switch(pdo_src->fixed_src.supply_type)
    {
        case PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
           
            fix_volt = pdo_src->fixed_src.voltage;
            g_fill_datalog_buffer_2Bytes( fix_volt, aBuffer);
           
            fix_current = pdo_src->fixed_src.max_current;
             g_fill_datalog_buffer_2Bytes(fix_current, aBuffer);
            
            g_fill_datalog_buffer_2Bytes(0x00, aBuffer);//Extra 2 bytes filling to keep the Payload length for each PDO constant as 6 bytes each.
             
            break;
        case  PDO_BATTERY: /* Battery supply PDO */
        
            max_volt = pdo_src->bat_src.max_voltage;
             g_fill_datalog_buffer_2Bytes( max_volt, aBuffer);
           
            min_volt = pdo_src->bat_src.min_voltage;
             g_fill_datalog_buffer_2Bytes( min_volt, aBuffer);
            
            g_fill_datalog_buffer_2Bytes(0x00, aBuffer);//Extra 2 bytes filling to keep the Payload length for each PDO constant as 6 bytes each.
            
            break;
        case PDO_VARIABLE_SUPPLY :/* Variable supply PDO */
           
            max_volt = pdo_src->var_src.max_voltage;
             g_fill_datalog_buffer_2Bytes( max_volt, aBuffer);
         
            min_volt = pdo_src->var_src.min_voltage;
             g_fill_datalog_buffer_2Bytes( min_volt, aBuffer);
            
            max_current = pdo_src->var_src.max_current;
             g_fill_datalog_buffer_2Bytes( max_current, aBuffer);
          
            break;
          case PDO_AUGMENTED:
             max_volt = pdo_src->pps_src.max_volt;/**< Maximum voltage in 100 mV units. */
             g_fill_datalog_buffer_2Bytes( max_volt, aBuffer);
         
            min_volt = pdo_src->pps_src.min_volt;/**< Maximum voltage in 100 mV units. */
             g_fill_datalog_buffer_2Bytes( min_volt, aBuffer);
            
            max_current = pdo_src->pps_src.max_cur;/**< Maximum current in 50 mA units. */
             g_fill_datalog_buffer_2Bytes( max_current, aBuffer);
            break;
         default :
         
            break;
    }
    
}
void g_get_ccg_src_cap_details(uint8_t * aBuffer)
{
    const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
    
    
    uint8_t i = 0 , aPrSrcCapsCount = dpm_stat -> src_pdo_count;
    
    /*No. of PDO's in [2]index-2*/
    aBuffer[glogBufIndex++] = aPrSrcCapsCount;
    
    for(i = 0; i < aPrSrcCapsCount; i++)
    {
        g_decode_SRC_PDOs(i, &dpm_stat ->src_pdo[i], aBuffer);
    }
    
}

/**/
void g_get_dut_src_cap_details(const pd_packet_t* src_cap, uint8_t * aBuffer)
{
    uint8_t index = 0, num_src_pdo = src_cap->len;
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    /*No. of PDO's in [2]index-2*/
    aBuffer[glogBufIndex++] = src_cap->len;
    
    for(index = 0; index < num_src_pdo; index++)
    {
        g_decode_SRC_PDOs(index,&src_cap->dat[index], aBuffer);
    }
}
/**/
void g_get_snk_cap_details(const pd_do_t * Snkrdo, uint8_t *aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint32_t  min_max_power_cur=0,op_power_cur=0;  
    uint8_t obj_pos=0;
   
        obj_pos = Snkrdo->val;
         aBuffer[glogBufIndex++] =obj_pos;    
        
        op_power_cur = Snkrdo->rdo_gen.op_power_cur;
        g_fill_datalog_buffer_2Bytes(op_power_cur, aBuffer);
    
        min_max_power_cur = Snkrdo->rdo_gen.min_max_power_cur;
        g_fill_datalog_buffer_2Bytes(min_max_power_cur, aBuffer);
    
  
    
}

//Pranay,16July'19, Filling I2c BUffer with Present PDC details when requested.
void g_PresentPDCInfo(const dpm_status_t *dpm_stat,/*contract_t contract,*/ uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint32_t lvariable = 0;
   
    if(dpm_stat->cur_port_role == PRT_ROLE_SINK)
    {
        aBuffer[glogBufIndex++] = ((dpm_stat->snk_sel_pdo.src_gen.supply_type << 4) | dpm_stat->snk_rdo.rdo_gen.obj_pos);
    }
    else
    {
        aBuffer[glogBufIndex++] = ((dpm_stat->src_sel_pdo.src_gen.supply_type << 4) | dpm_stat->src_rdo.rdo_gen.obj_pos);
    }
    lvariable = dpm_stat->contract.cur_pwr;//Current = 10mA pu
    g_fill_datalog_buffer_2Bytes(lvariable, aBuffer);
    lvariable = dpm_stat->contract.max_volt;//max. voltage in mV p.u
    g_fill_datalog_buffer_2Bytes(lvariable, aBuffer);
    lvariable = dpm_stat->contract.min_volt;//min. voltage in mV p.u
    g_fill_datalog_buffer_2Bytes(lvariable, aBuffer);
    
}
#if ONLY_PD_SNK_FUNC_EN
void g_fill_datalog_buffer_4Bytes( uint32_t lvalue, uint8_t * aBuffer)
{
     //aBuffer[glogBufIndex++] = 0xFE;
    aBuffer[glogBufIndex++] = lvalue;
    aBuffer[glogBufIndex++] = lvalue >> 8;
    aBuffer[glogBufIndex++] = lvalue >> 16;
    aBuffer[glogBufIndex++] = lvalue >> 24;
    // aBuffer[glogBufIndex++] = 0xFF;
}
/***/
void g_Fetch_uVDMData(uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    aBuffer[glogBufIndex++] = g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs;
    
    for(uint8_t i = 0; i < g_Struct_Ptr->gPDSSConfigCtrl.gNoOfVDOs; ++i)
        g_fill_datalog_buffer_4Bytes(g_Struct_Ptr->gPDSSConfigCtrl.gVdmSVID[i],aBuffer);
        
    aBuffer[2] = glogBufIndex; /**Replacing 2nd Bytes with ByteCount**/
 
}
void g_get_Src_caps_extnd_details(uint8_t * aBuffer)
{
        g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
        
        uint8_t sys_max_pdp = 0;
        /* If PD3.0 is supported, the PDP value will be read from the extended source cap */        
        const dpm_status_t *dpm_stat = dpm_get_info(0);
        
        sys_max_pdp = dpm_stat->ext_src_cap[CCG_PD_EXT_SRCCAP_PDP_INDEX];

        aBuffer[glogBufIndex++] = sys_max_pdp;
}
#endif/* ONLY_PD_SNK_FUNC_EN*/

#ifdef EVT_LOG
void g_EventlogBufFetch(uint8_t * aBuffer)
{
    
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    aBuffer[glogBufIndex++] = gEventlogBufIndex; /**2 Bytes for total Bytes of data in Even log Buffer**/
    aBuffer[glogBufIndex++] = gEventlogBufIndex >> 8; 
    aBuffer[glogBufIndex++] = g_Struct_Ptr->gLogData.gFromBufIndex;/**1Byte for From which event Buf index data is being filled */

    
    if(gEventlogBufIndex<= 32)
    {
        aBuffer[glogBufIndex++] = gEventlogBufIndex;/***1Byte for No. of Bytes being filled in present read operation*/
        
        memcpy(&aBuffer[glogBufIndex],&gEventlogBuffer[g_Struct_Ptr->gLogData.gFromBufIndex],gEventlogBufIndex);
        g_Struct_Ptr->gLogData.gFromBufIndex += gEventlogBufIndex; /**From Buf index will be incremented by no. of bytes being filled everytime unless overwritten it to zero*/
        glogBufIndex += gEventlogBufIndex;/** response buffer index should be tracked with no. of bytes being reposnded with, so incrementingby No of bytes being filled*/
        gEventlogBufIndex = 0;
        g_Struct_Ptr->gLogData.gFromBufIndex = 0;
    }
    else
    {
        aBuffer[glogBufIndex++] = 32;/***1Byte for No. of Bytes being filled in present read operation*/
        
        memcpy(&aBuffer[glogBufIndex],&gEventlogBuffer[g_Struct_Ptr->gLogData.gFromBufIndex],32);
        g_Struct_Ptr->gLogData.gFromBufIndex += 32; /**From Buf index will be incremented by no. of bytes being filled everytime unless overwritten it to zero*/
        glogBufIndex += 32;/** response buffer index should be tracked with no. of bytes being reposnded with, so incrementingby No of bytes being filled*/
    }
}
void EvtBufCopy(uint8_t * aBuffer)
{
    memcpy(&aBuffer[glogBufIndex],&gEventlogBuffer[0],gEventlogBufIndex);
    glogBufIndex += gEventlogBufIndex;
}
#endif
/*
Pranay,07Jun'19,Function to get details of DUT packet details
*/
void g_Get_PDNegotiationInfo(uint8_t * RecvBuffer,uint8_t * aBuffer)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    uint16_t lRetvalue = 0;
    
    // uint32_t op_cur = 0;
   // pd_config_t* Configptr = get_pd_config();
   
    /*should assign this index to zero before starting data retrieving*/
    glogBufIndex = 0;
    
    /*Should not change this variable as this is the keyword for reading*/
    aBuffer[glogBufIndex++] = 0xFA;
    
    const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
   
    if(RecvBuffer[2] == 0x01)
    {
        aBuffer[glogBufIndex++] = 0xA3;//Keyword
        aBuffer[glogBufIndex++] = 0xFE;//will be replaced by bytecount
        memcpy(&aBuffer[3], glFirmwareID, 6);
        glogBufIndex = 3+6;
    }
    else 
    {
    switch(RecvBuffer[3]) 
    {
        case 1:
            /*1st index will be filled with 0xD1 as DUT packets SrcCaps keyword*/ 
            aBuffer[glogBufIndex++] = 0xD1;
            aBuffer[glogBufIndex++] = 0xFD;
             /*getting no.of PDO's and there details and copying into buffer starting from 3rd index*/
            
            if( dpm_get_info (TYPEC_PORT_0_IDX)->cur_port_role == PRT_ROLE_SINK )
            {
                if(dpm_stat->contract_exist)//Pranay,16July'19,Filling i2C Buffer only if PDC is success
                { 	
                	g_get_dut_src_cap_details(dpm_stat->src_cap_p, aBuffer);
                }
            }
            else if(dpm_get_info (TYPEC_PORT_0_IDX)->cur_port_role == PRT_ROLE_SOURCE )
            {
                if(dpm_stat->contract_exist)//Pranay,16July'19,Filling i2C Buffer only if PDC is success
                {
                    g_get_ccg_src_cap_details(aBuffer);
                }
            }
        break;
        
        case 2://Getting Snkcapabilities
            aBuffer[glogBufIndex++] = 0xD2;
            aBuffer[glogBufIndex++] = 0xFD;
            g_get_snk_cap_details(&dpm_stat->snk_rdo, aBuffer);
        break;
#if ONLY_PD_SNK_FUNC_EN 
        case 0x04://getSrcCapsextnd
            aBuffer[glogBufIndex++] = 0xD8;//Keyword
            aBuffer[glogBufIndex++] = 0xFD;
            
            g_get_Src_caps_extnd_details(aBuffer);
        
        break;

        case 0xF0://Active CC line
            aBuffer[glogBufIndex++] = 0xCC;
            aBuffer[glogBufIndex++] = 0xFD;
            aBuffer[glogBufIndex++] = g_ActiveCC(aBuffer);
            aBuffer[glogBufIndex++] = dpm_stat->contract_exist;//PDC status
        break;
#endif/*ONLY_PD_SNK_FUNC_EN*/        
        case 0xF1://PDC info
            aBuffer[glogBufIndex++] = 0xD4;
            aBuffer[glogBufIndex++] = 0xFD;
            aBuffer[glogBufIndex] = (dpm_stat->contract_exist & 0x01);//PDC status
            aBuffer[glogBufIndex] |= (dpm_stat->cur_port_role & 0x03) << 1;//Current port power role,Src/Sink
            aBuffer[glogBufIndex++] |= ((dpm_stat->attached_dev & 0x0F) << 4); //Attached device type

            g_PresentPDCInfo(dpm_stat/*dpm_stat->contract*/,aBuffer);//PDC Voltage/Current details
            lRetvalue = vbus_get_value(G_PORT0);/*return current VBUS voltage in mV */ /* 23 3F for 9.023v*/ 
            g_fill_datalog_buffer_2Bytes(lRetvalue, aBuffer);
            aBuffer[glogBufIndex] = g_ActiveCC(aBuffer);
            aBuffer[glogBufIndex] |= ((g_Struct_Ptr->RequestPacketConfig.gDUTSpecRev & 0x03) << 1) ;
            glogBufIndex++;
        break;
        
        case 0xF2:
            /*retrieving current Vbus value*/
            aBuffer[glogBufIndex++] = 0xD3;
            aBuffer[glogBufIndex++] = 0xFD;

            lRetvalue = vbus_get_value(G_PORT0);/*return current VBUS voltage in mV */
            g_fill_datalog_buffer_2Bytes(lRetvalue, aBuffer);
#if 0
            op_cur =  dpm_stat->src_cur_rdo.rdo_pps.op_cur * 5; /* In 10mA units */
            
            aBuffer[glogBufIndex++] = op_cur;
            aBuffer[glogBufIndex] = op_cur >> 8;
#endif
        break;
        case 0xF4://Checking for PD/BC1.2 interrupt
            aBuffer[glogBufIndex++] = 0xD5;
            aBuffer[glogBufIndex++] = 0xFF;//will be replaced by bytecount
            aBuffer[glogBufIndex++] = i2cBuf[0];
            aBuffer[glogBufIndex++] = i2cBuf[1];
            aBuffer[glogBufIndex++] = i2cBuf[2];
            aBuffer[glogBufIndex++] = i2cBuf[3];
            aBuffer[glogBufIndex++] = i2cBuf[4];
            aBuffer[glogBufIndex++] = i2cBuf[5];
            aBuffer[glogBufIndex++] = i2cBuf[6];
            aBuffer[glogBufIndex++] = i2cBuf[7]; 
           // i2cInterruptKeywordFill(aBuffer,&glogBufIndex);
            aBuffer[glogBufIndex++] = 0xAB;
           // glogBufIndex=11;
        break;

        case 0xF7:/**get Current data,power role*/
            aBuffer[glogBufIndex++] = 0xD7;/*Keyword**/
            aBuffer[glogBufIndex++] = 0xFD;
            const dpm_status_t *dpm_stat = dpm_get_info(G_PORT0);
            /**PDC status*/
            aBuffer[glogBufIndex++] = dpm_stat->contract_exist;
            /**Current DATA ROLE**/
            aBuffer[glogBufIndex++] = dpm_stat->cur_port_type;
            /**Current POWER Role*/
            aBuffer[glogBufIndex++] = dpm_stat->cur_port_role;
             /**< Port role: Sink, Source or Dual. *///Pranay,19OCt'22, To track Port role irrespecive of PDC status
            aBuffer[glogBufIndex++] = dpm_stat->port_role;
            break;
#if ONLY_PD_SNK_FUNC_EN            
        case 0xF6:/**GET_UVDM_DATA**/
            
            aBuffer[glogBufIndex++] = 0xD6;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            
            g_Fetch_uVDMData (aBuffer);
            
            break;
                
        case 0xF8://Get SOP1 ACK Data
        
            aBuffer[glogBufIndex++] = 0xD9;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            
            /**If data is already ready then directly push already populated data*/
            if(g_Struct_Ptr->gPDSSConfigCtrl.isCableDataReady)
            {
                
                memcpy(&aBuffer[glogBufIndex],gSOP1AckBuf,g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount);
                glogBufIndex += g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount;
                
            }
            else
            {
                /** 16 Jul'21,TBD:
                * If cable data is not ready then 
                * 1. Init DISC ID SOP ?
                  2. Get ACK packet details ? - then implement get functionality here itself
                    or
                  1. Sinlge API for both init Disc ID SOP1 and get ACK packet ? 
                        - then init here next start 1 Sec timer and decode in expiry and push to i2c
                    or
                  1. Push error code to Application?
                **/
                
                aBuffer[glogBufIndex++] = 0x00;
                memset(gSOP1AckBuf,0x00,64);
                g_Struct_Ptr->gPDSSConfigCtrl.gSOP1AckBufIndexCount = 0;
            }
        break;
        case 0xF9:
            aBuffer[glogBufIndex++] = 0xDA;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            
            uint32_t Dp0Val = 0;
            uint32_t Dp1Val = 0;
            uint8_t level = 0;
            (void)level;
            hsiom_set_config(GPIO_PORT_3_PIN_0, HSIOM_MODE_AMUXA);

            pd_adc_calibrate (0, PD_ADC_ID_1);

            level = pd_adc_sample (0, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);

            //Dp0Val = grl_pd_adc_level_to_3v3volt (0, PD_ADC_ID_1, level);

            hsiom_set_config(GPIO_PORT_3_PIN_0, HSIOM_MODE_GPIO);

            aBuffer[glogBufIndex++] = Dp0Val;
            aBuffer[glogBufIndex++] = Dp0Val >> 8;
            aBuffer[glogBufIndex++] = Dp0Val >> 16;
            aBuffer[glogBufIndex++] = Dp0Val >> 24;
            
            aBuffer[glogBufIndex++] = 0xAA;
            /****/ 
            level = 0;
            hsiom_set_config(GPIO_PORT_3_PIN_1, HSIOM_MODE_AMUXA);

            pd_adc_calibrate (0, PD_ADC_ID_1);  

            level = pd_adc_sample (0, PD_ADC_ID_1, PD_ADC_INPUT_AMUX_A);

            //Dp1Val = grl_pd_adc_level_to_3v3volt (0, PD_ADC_ID_1, level);

            hsiom_set_config(GPIO_PORT_3_PIN_1, HSIOM_MODE_GPIO);
            
            aBuffer[glogBufIndex++] = Dp1Val;
            aBuffer[glogBufIndex++] = Dp1Val >> 8;
            aBuffer[glogBufIndex++] = Dp1Val >> 16;
            aBuffer[glogBufIndex++] = Dp1Val >> 24;
            aBuffer[glogBufIndex++] = 0xAB;
            break;

#endif /*ONLY_PD_SNK_FUNC_EN*/
#ifdef EVT_LOG
      case 0xAA:
            
            aBuffer[glogBufIndex++] = 0xA2;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            
            //g_EventlogBufFetch(aBuffer);
            EvtBufCopy(aBuffer);
        break;
#endif
      case 0xB1://Fetching Battery status details
            aBuffer[glogBufIndex++] = 0xA3;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            /**Fill the received Battery status details here and push it to TC*/
            //SoC-2Bytes
            aBuffer[glogBufIndex++] = gBattStatBuf[0];
            aBuffer[glogBufIndex++] = gBattStatBuf[1];
            //Battery charging status-2Bits //Temperature status 2 bits
            aBuffer[glogBufIndex++] = gBattStatBuf[2];
            //Internal Temperature - 1Byte
            aBuffer[glogBufIndex++] = gBattStatBuf[3];
            //VID
            aBuffer[glogBufIndex++] = gBattStatBuf[4];
            aBuffer[glogBufIndex++] = gBattStatBuf[5];
            //PID
            aBuffer[glogBufIndex++] = gBattStatBuf[6];
            aBuffer[glogBufIndex++] = gBattStatBuf[7];
            //Battery Design Capacity
            aBuffer[glogBufIndex++] = gBattStatBuf[8];
            aBuffer[glogBufIndex++] = gBattStatBuf[9];
            //Battery Last Full Charge Capacity
            aBuffer[glogBufIndex++] = gBattStatBuf[10];
            aBuffer[glogBufIndex++] = gBattStatBuf[11];
            //Battery Type
            aBuffer[glogBufIndex++] = gBattStatBuf[12];
            
            memset(&gBattStatBuf[0], 0x00, 16);//Pranay,07Oct'22, Clearing Buffer after being read
            break;
      case 0xB2://Get status message response details
            
            aBuffer[glogBufIndex++] = 0xA4;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[0];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[1];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[2];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[3];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[4];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[5];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[6];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetStatusBuf[7];
                 
            break;
        case 0xB3://Get Battery statusB
                      
            aBuffer[glogBufIndex++] = 0xA5;/**Keyword*/
            aBuffer[glogBufIndex++] = 0xFD;
            
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[0];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[1];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[2];
            aBuffer[glogBufIndex++] = g_Struct_Ptr->gCustomConfig.GetBattStatusBuf[3];
            break;
        default:
            break;
          
    }
    
      aBuffer[2] = glogBufIndex; 
    
       if((RecvBuffer[3] == 0xF4) || (RecvBuffer[3] == 0xAA) || (RecvBuffer[3] == 0xF8) || (RecvBuffer[3] == 0xF9) || (RecvBuffer[3] == 0xF2) || (RecvBuffer[3] == 0xF7) )/**if Checking for PD/BC1.2/SOP1 ACK interrupt no validating PDC success or not**/
        {
            memset(&aBuffer[glogBufIndex+1], 0x00, (250-glogBufIndex));
            aBuffer[glogBufIndex + 2] = 0xDC; 
            aBuffer[glogBufIndex + 3] = 0xDC;
        }
        else
        {
          if(dpm_stat->contract_exist == 0x00)//Pranay,16July'19,Filling i2C Buffer only if PDC is success
          {
            memset(&aBuffer[3], 0x00, 250);
            aBuffer[glogBufIndex + 2] = 0xDC; 
            aBuffer[glogBufIndex + 3] = 0xDC;
          }
          else 
          {
            memset(&aBuffer[glogBufIndex+1], 0x00, (250-glogBufIndex));
            aBuffer[glogBufIndex + 2] = 0xDC; 
            aBuffer[glogBufIndex + 3] = 0xDC; 
          }
        }
    }
    if(RecvBuffer[2] == 0x01)
     { 
        aBuffer[2] = glogBufIndex; 
        memset(&aBuffer[glogBufIndex], 0x00, (250-glogBufIndex));
     }
}
#if 0
void gGet_BC12DUTConfig(uint8_t * RecvBuffer,uint8_t var,uint8_t * i2cBuffer)
{
    
            switch(var)
            {
                case BC_FSM_SINK_DCP_CONNECTED:
                   RecvBuffer[2] = 0x01;
                 RecvBuffer[3] = 0x0A;
                break;
                case BC_FSM_SINK_SDP_CONNECTED:
                   RecvBuffer[2] = 0x02;
                 RecvBuffer[3] = 0x0B;
                break;
                case BC_FSM_SINK_CDP_CONNECTED:
                   RecvBuffer[2] = 0x03;
                 RecvBuffer[3] = 0x0C;
                break;
                case BC_FSM_SINK_START:
                   RecvBuffer[2] = 0x04;
                 RecvBuffer[3] = 0x0D;
                break;
               case BC_FSM_SINK_PRIMARY_CHARGER_DETECT:
                   RecvBuffer[2] = 0x05;
                 RecvBuffer[3] = 0x0E;
                break;
                case BC_FSM_SINK_TYPE_C_ONLY_SOURCE_CONNECTED:
                   RecvBuffer[2] = 0x06;
                 RecvBuffer[3] = 0x0F;
                break;
                case BC_FSM_SINK_SECONDARY_CHARGER_DETECT:
                   RecvBuffer[2] = 0x07;
                 RecvBuffer[3] = 0xA0;
                break;     
                case BC_FSM_OFF:
                  RecvBuffer[2] = 0x18;
                 RecvBuffer[3] = 0xAF;
                break;
                default: 
                break;
            }
   i2cBuffer[2] = RecvBuffer[2];
   i2cBuffer[3] = RecvBuffer[3];
    
}
#endif

/* [] END OF FILE */
