
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
//#include <stdint.h>
//#include <stdbool.h>
#include <timer.h>
//#define MAX_NO_OF_PDO   (7u)

#define SRCCAPS_EXTND_CDWORD        0xD8
#define BC12_CURMODE_FETCH_CDWORD   0xBC
#define FW_VERSION_READ_CDWORD      0xA3
#define SNKCAPS_DATAREAD_CDWORD    0xD2
#define CCG3PA_INTR_FETCH_CDWORD    0xD5
#define VBUS_DATAREAD_CDWORD     0xD3
#define CCG3PA_READ_CDWORD       0xFA
#define ACTIVE_CC_CDWORD         0xCC
#define CAPMISMATCH_CDWORD       0xD5
#define PDCSTATUS_CDWORD         0xD4
#define GET_DUTCAPS_CDWORD       0xD1
#define SOP_CDWORD               0xFA
#define EOP_CDWORD                      0x55
#define CURRENT_DATAROLE_CDWORD         0xD7
#define UVDM_CDWORD                     0xD6
#define READBUF_DATAFILL_CHECKSUM        0xDC/**Cheksum to be filled after filling the data for a read API**/
#define FX3_WRITEDATA_CHECKSUM           0xCD/**Checksum that will be validated when received data msg from Application/Fx3**/

#define ONLY_PD_SNK_FUNC_EN             0x01

typedef enum
{
    INTR_BUF_CLR = 0,
    INTR_PD,
    INTR_BC12,
    INTR_CAPMISMATCH,
    INTR_ATTACHSTATE,
    INTR_REQ_STATE,
    INTR_DETACH,
}gPD_BC_INTR_STATUS_FETCH;

typedef enum
{
    INTR_SET = 0,
    INTR_CLR = 1,
    
}gINTR_Generate_Enum;

typedef enum
{
    PPS_TIMER_VAR = 0,
    
}gTimerExpVar;


typedef enum
{
    GRL_PD_SCOPE = 3,
    GRL_BC12_SCOPE = 4,
    
}gApp_Cmd_type_t;

typedef enum
{
    GRL_CABLE_DETACH = 0x01,
    GRL_CABLE_ATTACH = 0x02,
    GRL_TesterMode_SINK = 0x03,
    GRL_TesterMode_SRC = 0x04,
    GRL_APP_CONFIG = 0x05,
    GRL_TesterMode_DRP = 0x06,
    GRL_ATTACH_NO_CHECK_PDC = 0x07,
    GRL_DETACH_WITH_RP_RD_DISABLE = 0x08,
    
    GRL_APP_CMD_INIT = 0x10,
    GRL_SRC_CAPS_UPDATE = 0x20,
    
    GRL_CAPABILITY_MISMATCH_HANDLE = 0xF1,
    GRL_CUSTOM_CONFIG_HANDLE = 0xF2,

}gApp_PD_BC_Cmd_type_t;


typedef enum
{ 
    CCG_AS_SNK_ONLY = 0,
    CCG_AS_CABLE_TESTER = 1,
    CCG_AS_CABLE_AND_SNK = 2,
    
}gStateMachineTypes;

typedef struct
{
    uint8_t GetBattStatusBuf[4];
    uint8_t GetStatusBuf[8];
    volatile uint8_t gReceivedSetPortRoleCmd;
}g_Custom_config_Buf;

typedef struct 
{
    uint32_t gOperating_I ;         
    uint32_t gMax_Op_I ;
    uint8_t gPDO_Index ;
    uint8_t gPDO_Type;
    uint8_t gRequestEnableFlag;
    bool gDetachFlag;
    bool Capability_Mismatch;
    bool isDUT_FallBack;/** DUT fall back flag will be TRUE only after every Ps_Rdy and will be false each time when we send API.*/
    uint8_t gPDOsCount;
    uint8_t gCustomConfig;
    uint8_t gDUTSpecRev;
    bool isRuntime_SnkCapsConfigEnabled;
    
    /**Pranay, 14Dec'22, 
    *  Inorder not to generate an interrupt everytime when there is a 
    *  APDO request from SInk for every 7-13Sec, 
    *  these variables are being used to track previous APDO req Vbus v and i, 
    *  supply type and if the request is new request
    **/
    volatile uint16_t gPrevReqAPDOVbusVoltage;
    volatile uint16_t gPrevReqAPDOVbusCurrent;
    volatile uint16_t gReqSupplyType;
    volatile bool isNewAPDORequest;
    
}grlRequestPacketVar_t;

typedef enum
{
    gAptivDetachOverride = 1,
    
}grlPDStruct_CustomConfig_t;
typedef enum
{
    PDMODE = 0x01,
    QC2MODE = 0x02,
    QC3MODE = 0x03,
}QC4_3_ConfigFlag;

typedef enum
{
    GRL_PD_MODE_SET = 0x01,
    GRL_QC_MODE_SET = 0x02,
} grl_chgb_snk_Mode_term_t;
typedef enum
{
   //GRL_QC_TESTERMODE =0x01,
   GRL_SET_VOLTAGE = 0x01,
}grl_chgb_snkset_term_t;
/* venkat 6Jul'22
 * TaskID - V-1-T314 - QC2.0,3.04.0 implementation,
 * @typedef chgb_snk_term_t
 * @brief grl custom defined termination options for QC sink mode operations.
 */
typedef enum
{
    GRL_QC2_mode = 0x02,
    GRL_QC3_mode = 0x03,
    //GRL_QC4_mode = 0x04,  
} grl_chgb_snk_term_t;

typedef enum
{
    GRL_QC_REMOVE_TERM = 0x00,
    GRL_QC_5V = 0x02,
    GRL_QC_9V = 0x03,
    GRL_QC_12V = 0x04,
    GRL_QC_20V = 0x05,
    //GRL_QC4_mode = 0x04,
} grl_chgb_QC_term_t;

#if ONLY_PD_SNK_FUNC_EN
typedef enum 
{
    ControlTestAutomation_Start=0,
    ControlTestAutomation_Stop=1,
    CableConnection_Attach=2,
    CableConnection_Detach=3,
    SoftReset=4,
    HardReset=5,
    
}grlPdCtrlCmd_t;

typedef enum 
{
    RES_TYPE_IGNORE = 0,
    RES_TYPE_ACK,
    RES_TYPE_NAK,  

}SOP1Response;

typedef struct
{
    bool gDrSwapSent;
    bool gVdmInit;
    bool isBC12_Ctrl_Enabled;
    bool isRuntime_SnkCapsConfigEnabled;
    uint8_t gNoOfVDOs;
    bool CableIntrRx;
    uint8_t Cable_ResponseConfig;
    uint16_t gVDMHeader;
    uint32_t gVdmSVID[7];
    uint16_t gFromBufIndex;
    uint8_t gVUPStateMachineType;
    uint8_t gSOP1MsgID : 3;//Max value can only go upto 7 after that fallsback to 0 so using bitfields
    bool gStartSOP1DiscIDAfterPDC;
    bool isCableDataReady;
    uint8_t gSOP1AckBufIndexCount;
    uint8_t gQC4_3_ConfigFlag;  /**venkat 6Jul'22,flag for PD_negotiation fail case executing variables**/
    uint16_t gReqPulse_Count ; /*Tracking Required pulse pulse count through globle variable*/
    uint16_t gTarget_Voltage  ; /*Traking target voltage */
    uint16_t gCurrent_Pulsecnt; /*Traking current pulses generated */
}grlControlconfigVar_t;
#endif
typedef union
{
    uint32_t val;
    struct FIXEDSRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 100mA units. */
        uint32_t voltage                    : 10;   /**< Voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b00. */
    } fixed_src;
    
    struct VARSRC
    {
        uint32_t max_current                : 10;   /**< Maximum current in 10mA units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b10. */
    } var_src; 
    struct BATSRC
    {
        uint32_t max_power                  : 10;   /**< Maximum power in 250mW units. */
        uint32_t min_voltage                : 10;   /**< Minimum voltage in 50mV units. */
        uint32_t max_voltage                : 10;   /**< Maximum voltage in 50mV units. */
        uint32_t supply_type                : 2;    /**< Supply type - should be 'b01. */
    } bat_src;
    struct PPSSRC
    {
        uint32_t max_cur                    : 7;    /**< Maximum current in 50 mA units. */
        uint32_t min_volt                   : 8;    /**< Minimum voltage in 100 mV units. */
        uint32_t max_volt                   : 8;    /**< Maximum voltage in 100 mV units. */
        uint32_t supply_type                : 2;    /**< PDO type: Should be 3 for PPS APDO. */
    
    }pps_src;
    
}grlSrcPDPkt;

typedef struct
{
    uint8_t gNoOfPDOs;
    grlSrcPDPkt PDO[7];
    
}grlSrcCapsStructVar_t;

typedef enum
{
    NA = 0x00,
    GET_BATTERY_SOC_TEMP_DETAILS = 0x0D,    
}grl_customConfig;

typedef struct
{
    uint16_t gFromBufIndex;
    bool isGetBattStatusInited;
    bool isGetStatusInited;
    uint8_t gCustomConfig;
    bool isOCPOccurred;
    bool pr_swap_init_after_PDC; 
} grlLogStructVar_t;

typedef struct
{
    grlRequestPacketVar_t RequestPacketConfig;
#if ONLY_PD_SNK_FUNC_EN    
    grlControlconfigVar_t gPDSSConfigCtrl;
#endif /*ONLY_PD_SNK_FUNC_EN*/

    grlSrcCapsStructVar_t gPresent_SRCCaps;
    grlSrcCapsStructVar_t gPrev_SRCCaps;
    grlLogStructVar_t gLogData;
    g_Custom_config_Buf gCustomConfig;

}grl_Struct_t;

grl_Struct_t gStruct_t;

#define GRL_QCpulse_voltage           (0xC8)   /*venkat 6Jul'2022,In QC, each D+,D- pulse will make vbus voltage increment or decrement by 200mV-- task-V-1-T314*/
void schedule_task(uint16_t ,timer_id_t );
grl_Struct_t *g_Struct_Ptr;
const grl_Struct_t * get_grl_struct_ptr();
ccg_status_t grl_chgb_apply_sink_term(uint8_t cport, grl_chgb_snkset_term_t testermodeterms,uint8_t  *lAppBuffer);
ccg_status_t grl_qc4_confighandler(uint8_t cport,uint8_t *lAppBuffer) ;
ccg_status_t grl_qc2_sink(uint8_t cport,uint8_t *lAppBuffer);
ccg_status_t grl_chgb_QC3(uint8_t *lAppBuffer) ;
/* [] END OF FILE */
