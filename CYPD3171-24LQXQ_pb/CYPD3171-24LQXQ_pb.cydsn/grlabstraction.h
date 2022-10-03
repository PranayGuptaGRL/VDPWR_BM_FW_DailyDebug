
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
/*******************************************************************************
 * Header files including
 ******************************************************************************/
//#include <project.h>
//#include <flash_config.h>
//#include <system.h>
#include <pd.h>
#include <dpm.h>
#include <psource.h>
//#include <psink.h>
//#include <swap.h>
//#include <vdm.h>
#include <pdo.h>
//#include <app.h>
//#include <hal_ccgx.h>
//#include <timer.h>
//#include <hpi.h>
//#include <hpd.h>
//#include <boot.h>
//#include <flash.h>
//#include <status.h>
//#include <ccgx_version.h>
//#include <app_version.h>
//#include <utils.h>
#include <gpio.h>
//#include <instrumentation.h>
#include <typec_manager.h>
#include <pd_protocol.h>
/**/
//#include <project.h>
//#include <config.h>
#include <dpm.h>
#include <chgb_hal.h>
//#include <psource.h>
//#include <psink.h>
//#include <timer.h>
#include <app.h>
//#include <battery_charging.h>

#define CALL_OUT_FUNCTION(func_name) func_name
//#define GET_IN_VARIABLE(var_name) var_name
#define G_PORT0                 (0u)

typedef enum
{
  Data_Msg = 1,
  Ctrl_Msg,
  Extd_Msg,
}gApp_PD_Msg_type;


void g_fill_datalog_buffer_2Bytes(uint16_t, uint8_t *);
void g_typec_cable_connection(uint8_t *);
void g_typec_Resistance_Set(uint8_t *);
void g_typec_Pd_Msg_Send(uint8_t *);
void g_get_src_cap_details(const pd_packet_t* src_cap, uint8_t *);
void gPrepareRequestPkt(uint8_t,uint8_t, bool, bool, uint32_t, uint32_t, uint16_t /*,app_resp_cbk_t*/);
void g_Get_PDNegotiationInfo(uint8_t *,uint8_t *);
void g_decode_SRC_PDOs(uint8_t ,const pd_do_t* , uint8_t * );
void g_typec_Pd_Data_Msg_Send(uint8_t * );
void g_typec_Pd_Extd_Msg_Send(uint8_t * );
void g_typec_Pd_Ctrl_Msg_Send(uint8_t *  );
//void gGet_BC12DUTConfig(uint8_t *,uint8_t,uint8_t * );
void g_CapabilityMismatchHandler(uint8_t * );
void prepareGetDevVersionPkt();


/* [] END OF FILE */
