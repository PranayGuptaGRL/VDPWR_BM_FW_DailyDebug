/**
 * @file pdo.c
 *
 * @brief @{PDO evaluation and handling functions.@}
 *
 *******************************************************************************
 *
 * Copyright (2014-2018), Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All rights reserved.
 *
 * This software, including source code, documentation and related materials
 * (“Software”), is owned by Cypress Semiconductor Corporation or one of its
 * subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software (“EULA”).
 *
 * If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress’s integrated circuit
 * products. Any reproduction, modification, translation, compilation, or
 * representation of this Software except as specified above is prohibited
 * without the express written permission of Cypress. Disclaimer: THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the
 * right to make changes to the Software without notice. Cypress does not
 * assume any liability arising out of the application or use of the Software
 * or any product or circuit described in the Software. Cypress does not
 * authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death (“High Risk Product”). By
 * including Cypress’s product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 */

#include <config.h>
#include <pd.h>
#include <dpm.h>
#include <pdo.h>
#include <app.h>
#include <timer.h>
#include <hpi.h>
#include "srom_vars.h"
#include <grlapp.h>
#include <gpio.h>
#if APP_PPS_SINK_SUPPORT
static pd_do_t app_prog_rdo[NO_OF_TYPEC_PORTS];
static bool    app_pps_snk_en[NO_OF_TYPEC_PORTS];
#endif /* APP_PPS_SINK_SUPPORT */

#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
extern bool gl_power_throttle_cmd_pending[NO_OF_TYPEC_PORTS];
extern bool gl_power_throttle_renegotiation_complete[NO_OF_TYPEC_PORTS];
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */

#if (!(CCG_SOURCE_ONLY))

#if (!CCG_SROM_CODE_ENABLE)
/* PDO Variables. */
uint32_t gl_max_min_cur_pwr[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_power[NO_OF_TYPEC_PORTS];
uint32_t gl_contract_voltage[NO_OF_TYPEC_PORTS];
uint32_t gl_op_cur_power[NO_OF_TYPEC_PORTS];
#endif /* (!CCG_SROM_CODE_ENABLE) */

ATTRIBUTES static uint32_t calc_power(uint32_t voltage, uint32_t current)
{
    /*
       Voltage is expressed in 50 mV units.
       Current is expressed in 10 mA units.
       Power should be expressed in 250 mW units.
     */
    return (CALL_OUT_FUNCTION(div_round_up)(voltage * current, 500));
}

#if !DISABLE_PDO_BATTERY
ATTRIBUTES static uint32_t calc_current(uint32_t power, uint32_t voltage)
{
    /*
       Power is expressed in 250 mW units.
       Voltage is expressed in 50 mV units.
       Current should be expressed in 10 mA units.
     */
    return (CALL_OUT_FUNCTION(div_round_up)(power * 500, voltage));
}
#endif /* DISABLE_PDO_BATTERY */

#if CCG_UCSI_ENABLE
#if UCSI_SET_PWR_LEVEL_ENABLE    
#define PWR_LVL_4P5W                    (9)         /* 4.5W in 0.5W units */
#define PWR_LVL_4P0W                    (8)         /* 4W in 0.5W units */    
#define MIN_CURRENT                     (90)        /* 900mA in 10mA units */

/**
 * @brief Edit the advertised PDOs to match the OS's requirements
 *
 * @param port The port whose PDOs need editing
 * @param power Final power level that the OS wants CCG to get to
 * @param is_src Set to '1' if the Source Caps need editing. For Sink Caps, its set to '0'
 * @param &out_mask An output variable indicating a bitmask of enabled PDOs
 *
 * @return 
 *    NULL If CCG should fail the command by setting the Error Indicator
 *    pd_do_t[7] A 7-element PD data object array of edited PDOs
 */
pd_do_t* ucsi_change_pdo_power(uint8_t port, uint8_t power, uint8_t is_src, uint8_t *out_mask)
{
    uint8_t i, pdo_cnt;
    uint32_t cur_power;
    const pd_do_t* cur_pdo;
    static pd_do_t pdos[MAX_NO_OF_PDO];

    /* This is a source-only application. Sink PDOs are not expected to be edited */
    if(is_src == 0)
        return NULL;
    
    /* Always enable 5V */
    *out_mask = 1;

    /* Take the number of PDOs and the list from the configuration table */
    pdo_cnt = get_pd_port_config(port)->src_pdo_cnt;
    cur_pdo = (pd_do_t*)get_pd_port_config(port)->src_pdo_list;
    
    for(i = 0; i < pdo_cnt; i++)
    {
        pdos[i].val = cur_pdo[i].val;
        
        /* If the OPM sets PD power level to 0 or 255, we revert to the default configuration */
        if((power == 0) || (power == 0xFF))
            *out_mask |= (1 << i);
        else
        {
            switch(pdos[i].fixed_src.supply_type)
            {
                case PDO_FIXED_SUPPLY:
                    cur_power = calc_power(pdos[i].fixed_src.voltage, pdos[i].fixed_src.max_current);
                    /* Force the max current to be within 900mA and 3A */
                    pdos[i].fixed_src.max_current = GET_MAX(MIN_CURRENT, GET_MIN((2 * power * pdos[i].fixed_src.max_current) / cur_power, CBL_CAP_3A));

                    /* Enable other PDOs only if requested power is >4.5W */
                    if(power > PWR_LVL_4P5W)
                        *out_mask |= (1 << i);
                    break;
                    
                case PDO_VARIABLE_SUPPLY:
                    cur_power = calc_power(pdos[i].var_src.max_voltage, pdos[i].var_src.max_current);
                    pdos[i].var_src.max_current = GET_MAX(MIN_CURRENT, GET_MIN((2 * power * pdos[i].var_src.max_current) / cur_power, CBL_CAP_3A));
                    *out_mask |= (1 << i);
                    break;
                    
                case PDO_BATTERY:
                    pdos[i].bat_src.max_power = power * 2;
                    *out_mask |= (1 << i);
                    break;
            }
        }
    }

    return pdos;
}
#endif /*UCSI_SET_PWR_LEVEL_ENABLE*/
#endif /* CCG_UCSI_ENABLE */

uint32_t grl_RequestPacketConfig(uint8_t PDO_index, uint8_t pdo_Type,uint32_t Max_Op_I, uint32_t Operating_I, uint32_t Parameter)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    
    switch(Parameter)
    {
        case 1:
          // Parameter = gPDO_Index;
           Parameter = g_Struct_Ptr->RequestPacketConfig.gPDO_Index;
            break;
        case 2:
          // Parameter = gMax_Op_I;
            Parameter = g_Struct_Ptr->RequestPacketConfig.gMax_Op_I;
            break;
        case 3:
         //  Parameter = gOperating_I;
           Parameter = g_Struct_Ptr->RequestPacketConfig.gOperating_I;
        break;
        case 4:
        //Parameter = pdo_Type;
       Parameter = g_Struct_Ptr->RequestPacketConfig.gPDO_Type;    
        break;
        case 0x0F:
        
        break;
       default:
      #if 0
        gOperating_I = Operating_I;
        gMax_Op_I = Max_Op_I;
        gPDO_Index = PDO_index;
        gRequestEnableFlag =1;
        gPDO_Type = pdo_Type;
       
        RequestPacketConfig.gOperating_I = Operating_I;
        RequestPacketConfig.gMax_Op_I = Max_Op_I;
        RequestPacketConfig.gPDO_Index = PDO_index;
        RequestPacketConfig.gRequestEnableFlag = 1;
        RequestPacketConfig.gPDO_Type = pdo_Type;
        #endif
        break;
    }
      
    return Parameter;
}

ATTRIBUTES static pd_do_t gPrepareRequestPkt(uint8_t pdo_no, uint8_t pdo_Type,bool capMisMatch, bool giveBack, uint32_t op_cur_power_volt, uint32_t max_min_cur_pwr,uint16_t MsgType/*, app_resp_cbk_t app_resp_handler*/)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    const dpm_status_t* dpm = CALL_OUT_FUNCTION(dpm_get_info)(G_PORT0);
    pd_do_t snk_rdo;
    
    snk_rdo.val = 0u;
    snk_rdo.rdo_gen.no_usb_suspend = dpm->snk_usb_susp_en;
    snk_rdo.rdo_gen.usb_comm_cap = dpm->snk_usb_comm_en;
    snk_rdo.rdo_gen.cap_mismatch = capMisMatch;
    snk_rdo.rdo_gen.give_back_flag = (capMisMatch) ? false : giveBack;
#ifdef GRL_PPS_TIMER_HANDLE
    AUG_TIMER_Stop();
#endif /*GRL_PPS_TIMER_HANDLE*/
    switch(pdo_Type)
    {
        case PDO_FIXED_SUPPLY:
        case PDO_VARIABLE_SUPPLY:
        snk_rdo.rdo_gen.obj_pos = grl_RequestPacketConfig(0,0,0,0,1);//pdo_no;
        /**< Operating current in 10mA units.*/
        snk_rdo.rdo_gen.min_max_power_cur = grl_RequestPacketConfig(0,0,0,0,2);//max_min_cur_pwr;//GET_IN_VARIABLE(gl_max_min_cur_pwr)[port];
         /**< Maximum operating current in 10mA units. */
        snk_rdo.rdo_gen.op_power_cur = grl_RequestPacketConfig(0,0,0,0,3);//op_cur_power;//GET_IN_VARIABLE(gl_op_cur_power)[port];
       
        break;

        case PDO_BATTERY:
        snk_rdo.rdo_bat.obj_pos = grl_RequestPacketConfig(0,0,0,0,1);//pdo_no;
        snk_rdo.rdo_bat.op_power =grl_RequestPacketConfig(0,0,0,0,2);/**< Operating power in 250mW units. */
        snk_rdo.rdo_bat.max_op_power =grl_RequestPacketConfig(0,0,0,0,3); /**< Maximum operating power in 250mW units. */
        break;
        case PDO_AUGMENTED:
          snk_rdo.rdo_pps.obj_pos = grl_RequestPacketConfig(0,0,0,0,1);//pdo_no;
          snk_rdo.rdo_pps.out_volt = grl_RequestPacketConfig(0,0,0,0,2);/*Requested output voltage in 20 mV units.*/
          snk_rdo.rdo_pps.op_cur = grl_RequestPacketConfig(0,0,0,0,3); /**< Operating current in 50 mA units. */
        #ifdef GRL_PPS_TIMER_HANDLE
        gTimerVar = APDO_TIMER;
        MsgTimerStart(8000);
        #endif /*GRL_PPS_TIMER_HANDLE*/
        break; 
        
        /** Aptive Requirement*/
        case (0xF1): /**Fixed PDO*/
        case (0xF4) :/**Variable PDO*/

            snk_rdo.rdo_gen.obj_pos = pdo_no;
            snk_rdo.rdo_gen.op_power_cur = op_cur_power_volt ;/**<received and sending DO Maximum operating current in 10mA units. */
            snk_rdo.rdo_gen.min_max_power_cur = max_min_cur_pwr;/**< received and sending DO Maximum operating current in 10mA units. */

            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = pdo_no;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = max_min_cur_pwr;
            g_Struct_Ptr->RequestPacketConfig.gOperating_I = op_cur_power_volt;
        break;
        case (0xF2):/**BAttery PS*/

            snk_rdo.rdo_bat.obj_pos = pdo_no;
            snk_rdo.rdo_bat.op_power = op_cur_power_volt;/**250mW p.u*/
            snk_rdo.rdo_bat.max_op_power = max_min_cur_pwr;/**250mW p.u*/

            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = pdo_no;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = max_min_cur_pwr;
            g_Struct_Ptr->RequestPacketConfig.gOperating_I = op_cur_power_volt;
        break;
        case (0xF3) :/**APDO*/

            snk_rdo.rdo_pps.obj_pos = pdo_no;
            snk_rdo.rdo_pps.out_volt = (op_cur_power_volt * 5);/**Voltage received in 100mv p.u, RDO voltage to be sent is 20mv p.u so converting to respective units*/
            snk_rdo.rdo_pps.op_cur = max_min_cur_pwr;/**Received Current and RDO current are both in 50mA p.u so no need to change*/

            g_Struct_Ptr->RequestPacketConfig.gPDO_Index = pdo_no;
            g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = (op_cur_power_volt * 5);/**Voltage received in 100mv p.u, RDO voltage to be sent is 20mv p.u so converting to respective units*/
            g_Struct_Ptr->RequestPacketConfig.gOperating_I = max_min_cur_pwr;/**Received Current and RDO current are both in 50mA p.u so no need to change*/
            
            g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag = true;
            g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack = false;
        
#ifdef GRL_PPS_TIMER_HANDLE

            MsgTimerStart(8000);

#endif /*GRL_PPS_TIMER_HANDLE*/

          /* start the timer to attempt the sequence again after the defined period. */
         //   timer_start (0, APP_PPS_SNK_RECONTRACT_TIMER_ID, APP_PPS_SNK_CONTRACT_PERIOD, pps_timer_cb);
        break;
        
        /**End of Aptiv Requirement**/
        default:
            snk_rdo.rdo_gen.obj_pos = 1;
            snk_rdo.rdo_gen.min_max_power_cur = max_min_cur_pwr;//0x32
            snk_rdo.rdo_gen.op_power_cur = op_cur_power_volt;//0x0A

        break;
        
    }
 /*  
  if((snk_rdo.rdo_gen.give_back_flag == false) &&
           (snk_rdo.rdo_gen.op_power_cur > snk_rdo.rdo_gen.min_max_power_cur))
    {
       snk_rdo.rdo_gen.min_max_power_cur = snk_rdo.rdo_gen.op_power_cur;
    }
*/
#if (CCG_SROM_CODE_ENABLE || CCG_PD_REV3_ENABLE)
    if(dpm->spec_rev_sop_live >= PD_REV3)
    {
        snk_rdo.rdo_gen.unchunk_sup = true;
    }
#endif /* (CCG_SROM_CODE_ENABLE | CCG_PD_REV3_ENABLE) */

    return snk_rdo;
     
}
#if 0
ATTRIBUTES static bool grl_is_src_acceptable_snk(uint8_t port, pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    const dpm_status_t* dpm = CALL_OUT_FUNCTION(dpm_get_info)(port);
   // pd_do_t* pdo_snk = (pd_do_t*)&dpm->cur_snk_pdo[snk_pdo_idx];
   // uint32_t snk_supply_type = pdo_snk->fixed_snk.supply_type;
    uint32_t fix_volt;
    uint32_t max_volt;
   // uint32_t min_volt;
    uint32_t out = false;
    uint32_t max_min_temp;
 g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();

    max_min_temp = dpm->cur_snk_max_min[snk_pdo_idx] & SNK_MIN_MAX_MASK;
    
#if !DISABLE_PDO_BATTERY    
    
#endif /* DISABLE_PDO_BATTERY */

 // switch(pdo_src->fixed_src.supply_type)
    {
   //  case PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
      fix_volt = pdo_src->fixed_src.voltage;
            max_volt = CALL_OUT_FUNCTION(div_round_up)(fix_volt, 20);
           // min_volt = fix_volt - max_volt;
            max_volt = fix_volt + max_volt;
        GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_src->fixed_src.max_current;
        g_Struct_Ptr->RequestPacketConfig.gMax_Op_I = pdo_src->fixed_src.max_current;
        g_Struct_Ptr->RequestPacketConfig.gOperating_I = pdo_src->fixed_src.max_current;
       // break;
        out = true;
     //   break;
    }

   if (out)
        {
            GET_IN_VARIABLE(gl_contract_voltage)[port] = fix_volt;
            GET_IN_VARIABLE(gl_contract_power)[port]   = calc_power (fix_volt, GET_IN_VARIABLE(gl_op_cur_power)[port]);
            GET_IN_VARIABLE(gl_max_min_cur_pwr)[port]  = max_min_temp;
        }
    return out;
}
#endif

/**
 * Checks if SRC pdo is acceptable for SNK pdo.
 * @param pdo_src pointer to current SRC PDO
 * @param pdo_snk pointer to current SNK PDO
 * @return True if current src pdo is acceptable for current snk pdo
 */
ATTRIBUTES static bool is_src_acceptable_snk(uint8_t port, pd_do_t* pdo_src, uint8_t snk_pdo_idx)
{
    const dpm_status_t* dpm = CALL_OUT_FUNCTION(dpm_get_info)(port);
    pd_do_t* pdo_snk = (pd_do_t*)&dpm->cur_snk_pdo[snk_pdo_idx];
    uint32_t snk_supply_type = pdo_snk->fixed_snk.supply_type;
    uint32_t fix_volt;
    uint32_t max_volt;
    uint32_t min_volt;
    uint32_t out = false;
    uint32_t max_min_temp, compare_temp;
#if !DISABLE_PDO_BATTERY    
    uint32_t oper_cur_pwr;
#endif /* DISABLE_PDO_BATTERY */

    max_min_temp = dpm->cur_snk_max_min[snk_pdo_idx] & SNK_MIN_MAX_MASK;

    switch(pdo_src->fixed_src.supply_type)
    {
        case PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
            fix_volt = pdo_src->fixed_src.voltage;
            max_volt = CALL_OUT_FUNCTION(div_round_up)(fix_volt, 20);
            min_volt = fix_volt - max_volt;
            max_volt = fix_volt + max_volt;

            switch(snk_supply_type)  /* Checking sink PDO type */
            {
                case PDO_FIXED_SUPPLY:
                    if(fix_volt == pdo_snk->fixed_snk.voltage)
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->fixed_snk.op_current);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_snk->fixed_snk.op_current;
#else /* (POWER_BANK) */
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_src->fixed_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if ((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->var_snk.op_current);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_snk->var_snk.op_current;
#else /* (POWER_BANK) */
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_src->fixed_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;
#if !DISABLE_PDO_BATTERY
                case PDO_BATTERY:
                    if ((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        fix_volt = min_volt;

                        /* Calculate the operating current and min/max current values. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.op_power, min_volt);
                        max_min_temp = calc_current(max_min_temp, min_volt);

                        /* Make sure the source can supply the maximum current that may be required. */
                        compare_temp = GET_MAX(max_min_temp, oper_cur_pwr);
                        if (pdo_src->fixed_src.max_current >= compare_temp)
                        {
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* DISABLE_PDO_BATTERY */
                default:
                    break;
            }

            if (out)
            {
                GET_IN_VARIABLE(gl_contract_voltage)[port] = fix_volt;
                GET_IN_VARIABLE(gl_contract_power)[port]   = calc_power (fix_volt, GET_IN_VARIABLE(gl_op_cur_power)[port]);
                GET_IN_VARIABLE(gl_max_min_cur_pwr)[port]  = max_min_temp;
            }
            break;

#if !DISABLE_PDO_BATTERY
        case PDO_BATTERY:   /* SRC is a battery */
            max_volt = pdo_src->bat_src.max_voltage;
            min_volt = pdo_src->bat_src.min_voltage;

            switch(snk_supply_type)
            {
                case PDO_FIXED_SUPPLY:
                    /* Battery cannot supply fixed voltage
                     * Battery voltage changes with time
                     * This contract if permitted can be un-reliable */
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        /* Calculate the expected operating power and maximum power requirement. */
                        oper_cur_pwr = calc_power(max_volt, pdo_snk->var_snk.op_current);
                        max_min_temp = calc_power(max_volt, max_min_temp);

                        compare_temp = GET_MAX (max_min_temp, oper_cur_pwr);
                        if (pdo_src->bat_src.max_power >= compare_temp)
                        {
#if (!POWER_BANK)
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = oper_cur_pwr;
#else /* (POWER_BANK) */
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_src->bat_src.max_power;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;

                case PDO_BATTERY:
                    /* Battery connected directly to a battery
                     * This combination is unreliable */
                    if((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (max_min_temp, pdo_snk->bat_snk.op_power);
                        if (pdo_src->bat_src.max_power >= compare_temp)
                        {
                            GET_IN_VARIABLE(gl_op_cur_power)[port] = pdo_snk->bat_snk.op_power;
                            out = true;
                        }
                    }
                    break;

                default:
                    break;
            }

            if (out)
            {
                GET_IN_VARIABLE(gl_contract_voltage)[port] = max_volt;
                GET_IN_VARIABLE(gl_max_min_cur_pwr)[port]  = max_min_temp;
                GET_IN_VARIABLE(gl_contract_power)[port]   = GET_IN_VARIABLE(gl_op_cur_power)[port];
            }
            break;
#endif /* DISABLE_PDO_BATTERY */

        case PDO_VARIABLE_SUPPLY:   /* Variable supply PDO */
            max_volt = pdo_src->var_src.max_voltage;
            min_volt = pdo_src->var_src.min_voltage;

            switch (snk_supply_type) /* Checking sink PDO type */
            {
                case PDO_FIXED_SUPPLY:
                    /* This connection is not feasible
                     * A variable source cannot provide a fixed voltage */
                    break;

                case PDO_VARIABLE_SUPPLY:
                    if((min_volt >= pdo_snk->var_snk.min_voltage) && (max_volt <= pdo_snk->var_snk.max_voltage))
                    {
                        compare_temp = GET_MAX (pdo_snk->var_snk.op_current, max_min_temp);
                        if (pdo_src->var_src.max_current >= compare_temp)
                        {
#if (!POWER_BANK)
                            GET_IN_VARIABLE(gl_contract_power)[port] = calc_power(min_volt, pdo_snk->var_snk.op_current);
                            GET_IN_VARIABLE(gl_op_cur_power)[port]   = pdo_snk->var_snk.op_current;
#else /* (POWER_BANK) */
                            GET_IN_VARIABLE(gl_contract_power)[port] = calc_power(min_volt, pdo_src->var_src.max_current);
                            GET_IN_VARIABLE(gl_op_cur_power)[port]   = pdo_src->var_src.max_current;
#endif /* (POWER_BANK) */
                            out = true;
                        }
                    }
                    break;
#if !DISABLE_PDO_BATTERY
                case PDO_BATTERY:
                    if((min_volt >= pdo_snk->bat_snk.min_voltage) && (max_volt <= pdo_snk->bat_snk.max_voltage))
                    {
                        /* Convert from power to current. */
                        oper_cur_pwr = calc_current(pdo_snk->bat_snk.op_power, min_volt);
                        max_min_temp = calc_current(max_min_temp, min_volt);

                        compare_temp = GET_MAX (oper_cur_pwr, max_min_temp);
                        if (pdo_src->var_src.max_current >= compare_temp)
                        {
                            GET_IN_VARIABLE(gl_contract_power)[port] = pdo_snk->bat_snk.op_power;
                            GET_IN_VARIABLE(gl_op_cur_power)[port]   = oper_cur_pwr;
                            out = true;
                        }
                    }
                    break;
#endif /* DISABLE_PDO_BATTERY */
                default:
                    break;
            }

            if (out)
            {
                GET_IN_VARIABLE(gl_contract_voltage)[port] = max_volt;
                GET_IN_VARIABLE(gl_max_min_cur_pwr)[port]  = max_min_temp;
            }
            break;

        default:
            break;
    }

    return out;
}

ATTRIBUTES static pd_do_t form_rdo(uint8_t port, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
    const dpm_status_t* dpm = CALL_OUT_FUNCTION(dpm_get_info)(port);
    pd_do_t snk_rdo;

    snk_rdo.val = 0u;
    snk_rdo.rdo_gen.no_usb_suspend = dpm->snk_usb_susp_en;
    snk_rdo.rdo_gen.usb_comm_cap = dpm->snk_usb_comm_en;
    snk_rdo.rdo_gen.cap_mismatch = capMisMatch;
    snk_rdo.rdo_gen.obj_pos = pdo_no;
    snk_rdo.rdo_gen.give_back_flag = (capMisMatch) ? false : giveBack;
    snk_rdo.rdo_gen.op_power_cur = GET_IN_VARIABLE(gl_op_cur_power)[port];
    snk_rdo.rdo_gen.min_max_power_cur = GET_IN_VARIABLE(gl_max_min_cur_pwr)[port];

    if( (snk_rdo.rdo_gen.give_back_flag == false) &&
            (snk_rdo.rdo_gen.op_power_cur > snk_rdo.rdo_gen.min_max_power_cur))
    {
        snk_rdo.rdo_gen.min_max_power_cur = snk_rdo.rdo_gen.op_power_cur;
    }

#if (CCG_SROM_CODE_ENABLE || CCG_PD_REV3_ENABLE)
    if(dpm->spec_rev_sop_live >= PD_REV3)
    {
        snk_rdo.rdo_gen.unchunk_sup = true;
    }
#endif /* (CCG_SROM_CODE_ENABLE | CCG_PD_REV3_ENABLE) */

    return snk_rdo;
}

#if APP_PPS_SINK_SUPPORT
static pd_do_t handle_pps_sink_rdo(uint8_t port, const pd_packet_t *src_cap, pd_do_t dflt_rdo)
{
    const dpm_status_t *dpm_stat = dpm_get_info(port);
    pd_do_t pdo;
    pd_do_t rdo = dflt_rdo;

    if (
            (app_pps_snk_en[port]) &&
            (src_cap->len >= app_prog_rdo[port].rdo_gen.obj_pos)
       )
    {
        /* Perform sanity checks on validity of the RDO. */
        pdo = src_cap->dat[app_prog_rdo[port].rdo_gen.obj_pos - 1];

        if (
                (pdo.pps_src.supply_type == PDO_AUGMENTED) &&
                (pdo.pps_src.apdo_type == APDO_PPS) &&
                ((pdo.pps_src.max_volt * 5) >= app_prog_rdo[port].rdo_pps.out_volt) &&
                ((pdo.pps_src.min_volt * 5) <= app_prog_rdo[port].rdo_pps.out_volt) &&
                (pdo.pps_src.max_cur >= app_prog_rdo[port].rdo_pps.op_cur)
           )
        {
            rdo.val                    = app_prog_rdo[port].val;
            rdo.rdo_gen.no_usb_suspend = dpm_stat->snk_usb_susp_en;
            rdo.rdo_gen.usb_comm_cap   = dpm_stat->snk_usb_comm_en;
            rdo.rdo_gen.unchunk_sup    = true;
        }
        else
        {
            app_pps_snk_en[port]   = false;
            app_prog_rdo[port].val = 0;
        }
    }

    return (rdo);
}
#endif /* APP_PPS_SINK_SUPPORT */

void g_RuntimeSnkCapsUpdate(uint8_t port ,const pd_packet_t* src_cap)
{
    pd_do_t snk_pdo[MAX_NO_OF_PDO];
    uint8_t num_src_pdo = src_cap->len;
    uint8_t lCurrentPDOSupplytype = 0;
    //ccg_status_t retval=0;
    const dpm_status_t *dpm_stat = dpm_get_info (0);
    
    dpm_update_snk_cap_mask(0x00, 0x00);
    
    for(uint8_t i = 0; i < num_src_pdo; i++)
    {
        lCurrentPDOSupplytype = src_cap->dat[i].fixed_src.supply_type;
        switch(lCurrentPDOSupplytype)
        {
           case PDO_FIXED_SUPPLY:  
            //Snk.op.Current ->10mAP.U;Src.max.current -> 100mA P.U , So convert accordingly
            snk_pdo[i].fixed_snk.op_current = (src_cap->dat[i].fixed_src.max_current) ;
            snk_pdo[i].fixed_snk.voltage = src_cap->dat[i].fixed_src.voltage;
            break;
           case PDO_VARIABLE_SUPPLY:
            snk_pdo[i].var_snk.max_voltage = src_cap->dat[i].var_src.max_voltage;
            snk_pdo[i].var_snk.min_voltage = src_cap->dat[i].var_src.min_voltage;
            snk_pdo[i].var_snk.op_current = src_cap->dat[i].var_src.max_current;
            break;
           case PDO_BATTERY:
            snk_pdo[i].bat_snk.max_voltage = src_cap->dat[i].bat_src.max_voltage;
            snk_pdo[i].bat_snk.min_voltage = src_cap->dat[i].bat_src.min_voltage;
            snk_pdo[i].bat_snk.op_power = src_cap->dat[i].bat_src.max_power;
            break;
           case PDO_AUGMENTED:
            snk_pdo[i].pps_snk.max_volt = src_cap->dat[i].pps_src.max_volt;
            snk_pdo[i].pps_snk.min_volt = src_cap->dat[i].pps_src.min_volt;
            snk_pdo[i].pps_snk.op_cur = src_cap->dat[i].pps_src.max_cur;
            break;
            
        }        
        dpm_update_snk_cap_mask(0,(dpm_stat->snk_pdo_mask | (1 << i)));
        
        dpm_update_snk_cap(port,num_src_pdo, snk_pdo);  
        
    }
   
    
}
/**Function is used whenever Source initiates another Srccaps after initial PDC by changing its capacbilities it is expected that control comes here,
Filling the Maximum capabilities(V and i) based last requested PDC's PDO index(As per Aptivs requirement)
*/
ATTRIBUTES static pd_do_t g_DUTFallBackRequestpacketFill(const pd_packet_t* src_cap)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    pd_do_t snk_rdo;
            /**Based on last requested PDO supply type**/
            switch(g_Struct_Ptr->RequestPacketConfig.gPDO_Type)/*src_cap->dat[(g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1)].fixed_src.supply_type*/
            {
            case PDO_FIXED_SUPPLY:  /* Fixed supply PDO */
                snk_rdo = gPrepareRequestPkt(g_Struct_Ptr->RequestPacketConfig.gPDO_Index,/*gPDO_Type*/
                                    0xF1,
                                    false, false/*(dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK)*/,
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].fixed_src.max_current)),
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].fixed_src.max_current)),
                                    DATA_MSG_REQUEST/*,app_resp_handler*/); 
                break;
            case PDO_BATTERY:
                snk_rdo = gPrepareRequestPkt(g_Struct_Ptr->RequestPacketConfig.gPDO_Index,/*gPDO_Type*/
                                    0xF2,
                                    false, false/*(dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK)*/,
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].bat_src.max_power)),
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].bat_src.max_power)),
                                    DATA_MSG_REQUEST/*,app_resp_handler*/); 
                break;
            case PDO_AUGMENTED:
                snk_rdo = gPrepareRequestPkt(g_Struct_Ptr->RequestPacketConfig.gPDO_Index,/*gPDO_Type*/
                                    0xF3,
                                    false, false/*(dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK)*/,
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].pps_src.max_volt)),
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].pps_src.max_cur)),
                                    DATA_MSG_REQUEST/*,app_resp_handler*/);  
                break;
            case PDO_VARIABLE_SUPPLY:
                    snk_rdo = gPrepareRequestPkt(g_Struct_Ptr->RequestPacketConfig.gPDO_Index,/*gPDO_Type*/
                                    0xF4,
                                    false, false/*(dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK)*/,
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].var_src.max_current)),
                                   ((src_cap->dat[g_Struct_Ptr->RequestPacketConfig.gPDO_Index-1].var_src.max_current)),
                                    DATA_MSG_REQUEST/*,app_resp_handler*/); 
                break;
                }
            //g_Struct_Ptr->RequestPacketConfig.gCustomConfig = gAptivDetachOverride;
            g_Struct_Ptr->RequestPacketConfig.Capability_Mismatch = true;  
    return snk_rdo;
}
/***Handler used whenever DUT changes its capabilities in runtime i.e., Whenever DUT fallbakc happens*/
ATTRIBUTES static pd_do_t g_DUTFallBackHandler(const pd_packet_t* src_cap)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    pd_do_t snk_rdo;
    
    g_PdssGPIOIntrHandler(INTR_SET);
    PD_BC_i2cBufHandler(INTR_BUF_CLR);
    
    const dpm_status_t *dpm_stat = dpm_get_info(0);
    
    i2cBuf[2] = 0x00;
    i2cBuf[3]  = (dpm_stat->cur_port_role & 0x03);
    i2cBuf[3]  |= ((dpm_stat->role_at_connect & 0x03) << 2);
    i2cBuf[3]  |= ((dpm_stat->attached_dev & 0x0F) << 4);
    
    PD_BC_i2cBufHandler(INTR_CAPMISMATCH);
    g_PdssGPIOIntrHandler(INTR_CLR);
    
    /**Filling the request packet based on aptiv requirement whenever DUT Srccaps Fallback happens**/
    snk_rdo = g_DUTFallBackRequestpacketFill(src_cap); 
    
    return snk_rdo;
}
/*
*Compare existing capabilities with Incoming capabilities and return true if matches else return false
*/
bool g_CompareSrcCaps(grlSrcCapsStructVar_t * aIncomingCaps,grlSrcCapsStructVar_t * aExistingCaps)
{
    if(aIncomingCaps->gNoOfPDOs == aExistingCaps->gNoOfPDOs)
    {
        for(uint8_t i = 0; i < aIncomingCaps->gNoOfPDOs; ++i)
        {
            /** TBD : If this doesnt work than need to Compare each and every PDOs current and voltage values based on the supply type*/    
            if(aExistingCaps->PDO[i].val != aIncomingCaps->PDO[i].val)
                return false;/**Return false only if capabilities advertised are different*/
        }
    }
    else
        return false;/**Return false only if capabilities advertised are different*/
    
    return true;/**Return true only if capabilities advertised are same*/
}
/***Handling all the GRL related configurations in this functions*/
ATTRIBUTES static pd_do_t grl_eval_Src_caps(uint8_t port ,const pd_packet_t* src_cap)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
//#if ONLY_PD_SNK_FUNC_EN    
        /***If not configured from application than update ccg3pa's snkcaps based on DUT Src caps*/
        if( g_Struct_Ptr->RequestPacketConfig.isRuntime_SnkCapsConfigEnabled )
            g_RuntimeSnkCapsUpdate(port,src_cap);
//#endif /*ONLY_PD_SNK_FUNC_EN*/
        pd_do_t snk_rdo;
        
        /** Aptiv Requirement,IF Source changed its capabilities after PDC,expected to enter this condition and should request to max capabilites of last PDO */
        if(g_Struct_Ptr->RequestPacketConfig.isDUT_FallBack == true)
        {
            /***At present handling without checking any condition, can futher add a condion to validate whether DUT has really changed its capabilities or not*/
            g_SrcCapsDecode(&g_Struct_Ptr->gPresent_SRCCaps);
                
            if(!g_CompareSrcCaps(&g_Struct_Ptr->gPresent_SRCCaps, &g_Struct_Ptr->gPrev_SRCCaps))
                snk_rdo = g_DUTFallBackHandler(src_cap);
#if ONLY_PD_SNK_FUNC_EN
#ifdef EVT_LOG
	    //gBufLog(false,0xC1);         
#endif
#endif/*ONLY_PD_SNK_FUNC_EN*/
        }
        else if(g_Struct_Ptr->RequestPacketConfig.gRequestEnableFlag)/**If Request packet is configured from application expected to enter this condition**/
        { 
            g_Struct_Ptr->RequestPacketConfig.gPDOsCount = src_cap->len; /**Keeping track of no.of src caps received in last time PDC AMS */
            snk_rdo = gPrepareRequestPkt(1,/*gPDO_Type*/
                                g_Struct_Ptr->RequestPacketConfig.gPDO_Type, 
                                false,false, 
                                0x12C, 0x12C,DATA_MSG_REQUEST);
#if ONLY_PD_SNK_FUNC_EN

#ifdef EVT_LOG
        //gBufLog(false,0xC4);
#endif

#endif/*ONLY_PD_SNK_FUNC_EN*/
        }
        else/*Expected to enter this state after every Detach and after power on so default configurations needs to be handled here**/
        {
            g_Struct_Ptr->RequestPacketConfig.gPDOsCount = src_cap->len;/**Used for tracking whether any capability mismatch happened further or not**/
        //gBufLog(false,0xC3);
            if(src_cap->hdr.hdr.spec_rev == SPEC_REV_2_0)//if DUTs Spec rev is 2.0 ,requesting MaxOperating current of 500mA, Op.Current of 100mA
                snk_rdo = gPrepareRequestPkt(1,0x0F, false, false, 0x0A, 0x32,DATA_MSG_REQUEST );    
            else if(src_cap->hdr.hdr.spec_rev == SPEC_REV_3_0)//if DUTs Spec rev is 3.0 ,requesting MaxOperating current of 500mA, Op.Current of 500mA
                snk_rdo = gPrepareRequestPkt(1,0x0F, false, false, 0x32, 0x32,DATA_MSG_REQUEST );    
            else
                snk_rdo = gPrepareRequestPkt(1,0x0F, false, false, 0x0A, 0x0A,DATA_MSG_REQUEST );    
        }
    return snk_rdo;
}
/*
 * This function can be used to ask EC to evaluate a src cap message
 * For now evaluating here and executing the callback in this function itself
 */
ATTRIBUTES void eval_src_cap(uint8_t port, const pd_packet_t* src_cap, app_resp_cbk_t app_resp_handler)
{
    g_Struct_Ptr = (grl_Struct_t *)get_grl_struct_ptr();
    const dpm_status_t* dpm = CALL_OUT_FUNCTION(dpm_get_info)(port);
    uint8_t src_pdo_index, snk_pdo_index;
    uint8_t num_src_pdo = src_cap->len;
    pd_do_t* snk_pdo = (pd_do_t*)&dpm->cur_snk_pdo[0];
    uint16_t src_vsafe5_cur = src_cap->dat[0].fixed_src.max_current; /* Source max current for first PDO */
    pd_do_t snk_rdo;
    uint32_t highest_gl_contract_power = 0u;
 
    bool match = false;
    bool high_cap = snk_pdo[0].fixed_snk.high_cap; 

        for(snk_pdo_index = 0u; snk_pdo_index < dpm->cur_snk_pdo_count; snk_pdo_index++)
        {
            for(src_pdo_index = 0u; src_pdo_index < num_src_pdo; src_pdo_index++)
            {
                if(is_src_acceptable_snk(port, (pd_do_t*)(&src_cap->dat[src_pdo_index]), snk_pdo_index))
                {
                    /* Contract_power is calculated in is_src_acceptable_snk() */
                    if (GET_IN_VARIABLE(gl_contract_power)[port] >= highest_gl_contract_power)
                    {
                        /* Check if sink needs higher capability */
                        if ((high_cap) && (GET_IN_VARIABLE(gl_contract_voltage)[port] == (VSAFE_5V/PD_VOLT_PER_UNIT)))
                        {
                            /* 5V contract isn't acceptable with high_cap = 1 */
                            continue;
                        }

                        highest_gl_contract_power = GET_IN_VARIABLE(gl_contract_power)[port];
                        snk_rdo = form_rdo(port, (src_pdo_index + 1u), false,
                                (dpm->cur_snk_max_min[snk_pdo_index] & GIVE_BACK_MASK));
                        match = true;

    #if APP_PPS_SINK_SUPPORT
                        /* Update to PPS RDO if so required by user. */
                        snk_rdo = handle_pps_sink_rdo (port, src_cap, snk_rdo);
    #endif /* APP_PPS_SINK_SUPPORT */
                    }
                }
            }
        
            if(match == false)
            {
                /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
                GET_IN_VARIABLE(gl_contract_voltage)[port] = snk_pdo[0].fixed_snk.voltage;
                GET_IN_VARIABLE(gl_op_cur_power)[port] = snk_pdo[0].fixed_snk.op_current;
                GET_IN_VARIABLE(gl_contract_power)[port] = CALL_OUT_FUNCTION(div_round_up)(
                        GET_IN_VARIABLE(gl_contract_voltage)[port] * GET_IN_VARIABLE(gl_op_cur_power)[port], 500u);

                if(src_vsafe5_cur < GET_IN_VARIABLE(gl_op_cur_power)[port])
                {
                    /* SNK operation current can't be bigger than SRC max_current */
                    GET_IN_VARIABLE(gl_op_cur_power)[port] = src_vsafe5_cur;
                }

                GET_IN_VARIABLE(gl_max_min_cur_pwr)[port] = dpm->cur_snk_max_min[0];
              snk_rdo = form_rdo(port, 1u, true, false);
            }
        }
#ifdef GRL_CAPS    
    snk_rdo = grl_eval_Src_caps(port,src_cap);/**Pranay,07May'2020,Handling all the GRL Related configurations here in this function**/
#endif


     (CALL_OUT_FUNCTION(app_get_resp_buf)(port))->resp_do = snk_rdo;
    app_resp_handler(port, CALL_OUT_FUNCTION(app_get_resp_buf)(port));
}
#endif /* (!(CCG_SOURCE_ONLY)) */

/*
 * This function can be used to ask EC to evaluate a request message
 * For now evaluating here and executing the callback in this function itself
 */
ATTRIBUTES void eval_rdo(uint8_t port, pd_do_t rdo, app_resp_cbk_t app_resp_handler)
{
    if (CALL_OUT_FUNCTION(dpm_is_rdo_valid)(port, rdo) == CCG_STAT_SUCCESS)
    {
        CALL_OUT_FUNCTION(app_get_resp_buf)(port)->req_status = REQ_ACCEPT;
    }
    else
    {
        CALL_OUT_FUNCTION(app_get_resp_buf)(port)->req_status = REQ_REJECT;
#if (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE)
        if(true == gl_power_throttle_cmd_pending[port])
        {
             gl_power_throttle_renegotiation_complete[port] = true;
        }
#endif /* (CCG_TEMP_BASED_VOLTAGE_THROTTLING || CCG_VIN_BASED_VOLTAGE_THROTTLING || CCG_LOAD_SHARING_ENABLE) */
    }

    app_resp_handler(port, CALL_OUT_FUNCTION(app_get_resp_buf)(port));
}

#if (CCG_SROM_CODE_ENABLE)
void app_update_rdo (uint8_t port, const pd_packet_t* src_cap, app_resp_t *app_resp)
{
    /*  Enable below code if RDO prepared by ROM code needs to be over-ridden. */
#if OVERRIDE_DFLT_RDO
    pd_do_t rdo = app_resp->resp_do;

    if (rdo.rdo_gen.cap_mismatch == 0)
    {
        uint32_t src_current = src_cap->dat[rdo.rdo_gen.obj_pos - 1].fixed_src.max_current;

        if (rdo.rdo_gen.give_back_flag != 0)
        {
            /* Give back is set. Update operating current with value from PDO. */
            rdo.rdo_gen.op_power_cur = src_current;
        }
        else
        {
            /* Give back is not set. Update max. current with value from PDO. */
            rdo.rdo_gen.min_max_power_cur = src_current;
        }
        
        app_resp->resp_do = rdo;
    }
#endif /* OVERRIDE_DFLT_RDO */
}
#endif /* (CCG_SROM_CODE_ENABLE) */

#if APP_PPS_SINK_SUPPORT

static void snk_recontract_timer_cb (uint8_t port, timer_id_t id);

static void snk_recontract_cb (uint8_t port, resp_status_t resp, const pd_packet_t *pkt_ptr)
{
    if (resp == SEQ_ABORTED)
    {
        /* Restart the timer so that the command can be retried. */
        timer_start (port, 0x91, 5, snk_recontract_timer_cb);
    }
    else
    {
        /* Restart the timer to attempt the sequence again after 9 seconds. */
        timer_start (port, 0x91, 9000, snk_recontract_timer_cb);
    }
}

static void snk_recontract_timer_cb (uint8_t port, timer_id_t id)
{
    const dpm_status_t *dpm_stat = dpm_get_info (port);
    dpm_pd_cmd_buf_t    param;
    ccg_status_t        stat;

    if ((app_pps_snk_en[port]) && (dpm_stat->contract_exist != 0) && (dpm_stat->cur_port_role == PRT_ROLE_SINK) &&
                (dpm_stat->spec_rev_sop_live >= PD_REV3))
    {
        param.cmd_sop      = SOP;
        param.no_of_cmd_do = 0;
        param.timeout      = 0;
        stat = dpm_pd_command (port, DPM_CMD_SNK_CAP_CHNG, &param, snk_recontract_cb);
        if (stat != CCG_STAT_SUCCESS)
        {
            /* Restart the timer so that the command can be retried. */
            timer_start (port, id, 5, snk_recontract_timer_cb);
        }
    }
    else
    {
        app_pps_snk_en[port]   = false;
        app_prog_rdo[port].val = 0;
    }
}

uint8_t hpi_user_reg_handler(uint16_t addr, uint8_t size, uint8_t *data)
{
    const dpm_status_t *dpm_stat;
    uint8_t stat = 0x0F;
    uint8_t port = (addr >> 12) - 1;
    pd_do_t rdo;

    if ((port < NO_OF_TYPEC_PORTS) && ((addr & 0xFF) == 0x38) && (size == 4))
    {
        /* Store the user provided value in the register for debug purposes. */
        hpi_init_userdef_regs (addr, size, data);

        dpm_stat = dpm_get_info (port);
        rdo.val  = MAKE_DWORD(data[3], data[2], data[1], data[0]);

        /* Operation is only valid if we are a sink and PD contract exists. */
        if ((dpm_stat->contract_exist != 0) && (dpm_stat->cur_port_role == PRT_ROLE_SINK) &&
                (dpm_stat->spec_rev_sop_live >= PD_REV3))
        {
            app_prog_rdo[port]   = rdo;
            app_pps_snk_en[port] = true;
            stat = 0x02;                        /* Success code. */

            /* Start a timer which will start re-negotiation so that PPS source can be selected. */
            timer_start (port, 0x91, 1, snk_recontract_timer_cb);
        }
    }

    return (stat);
}

void app_pps_sink_disable(uint8_t port)
{
    app_pps_snk_en[port] = false;
}

#endif /* APP_PPS_SINK_SUPPORT */

/* End of File */

