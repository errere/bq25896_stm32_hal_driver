#ifndef __BQ25896_H__
#define __BQ25896_H__
#include <stdint.h>

typedef enum
{
    BQ_OK,
    BQ_ERR_TRANS,
    BQ_ERR_CODE,
    BQ_ERR_TIMEOUT,
    BQ_ERR_MAX
} bq25896_error_t;

typedef enum
{
    BQ_CP_UP,
    BQ_CP_DOWN,
    BQ_CP_MAX,
} bq25896_current_pluse_direction_t;

typedef enum
{
    BQ_BST_CL_0A5 = 0,
    BQ_BST_CL_0A75 = 1,
    BQ_BST_CL_1A2 = 2,
    BQ_BST_CL_1A4 = 3,
    BQ_BST_CL_1A65 = 4,
    BQ_BST_CL_1A875 = 5,
    BQ_BST_CL_2A15 = 6,
    BQ_BST_CL_MAX,
} bq25896_boost_current_limit_t;

/*============================================================reg 01============================================================*/
// Boost Mode Hot/Cold Temperature Monitor Threshold
#define BQ_BST_TT_DISABLE 3

#define BQ_BST_HTT_34P75 0
#define BQ_BST_HTT_37P75 1
#define BQ_BST_HTT_31P25 2

#define BQ_BST_CTT_77 0
#define BQ_BST_CTT_80 1

/*============================================================reg 02============================================================*/
#define BQ_ADC_ONE_SHOT 0
#define BQ_ADC_CONTINUOUS 1

#define BQ_BST_FREQ_1M5 0
#define BQ_BST_FREQ_500K 1

/*============================================================reg 03============================================================*/
#define BQ_BST_DISABLE_VOLTAGE_2V9 0
#define BQ_BST_DISABLE_VOLTAGE_2V5 1

/*============================================================reg 06============================================================*/
#define BQ_PRE_CHG_TO_FAST_CHG_THRESHOLD_2V8 0
#define BQ_PRE_CHG_TO_FAST_CHG_THRESHOLD_3V0 1

#define BQ_RE_CHG_THRESHOLD_0V1 0
#define BQ_RE_CHG_THRESHOLD_0V2 1
/*============================================================reg 07============================================================*/
#define BQ_WTD_DISABLE 0
#define BQ_WTD_40S 1
#define BQ_WTD_80S 2
#define BQ_WTD_160S 3

#define BQ_FAST_CHG_SAFE_TIMER_5H 0
#define BQ_FAST_CHG_SAFE_TIMER_8H 1
#define BQ_FAST_CHG_SAFE_TIMER_12H 2
#define BQ_FAST_CHG_SAFE_TIMER_20H 3

#define BQ_JEITA_LT_CURR_50P 0
#define BQ_JEITA_LT_CURR_20P 1

/*============================================================reg 08============================================================*/
#define BQ_THERMAL_REG_THRESHOLD_60C 0
#define BQ_THERMAL_REG_THRESHOLD_80C 1
#define BQ_THERMAL_REG_THRESHOLD_100C 2
#define BQ_THERMAL_REG_THRESHOLD_120C 3

/*============================================================reg 09============================================================*/
#define BQ_JEITA_HIGH_TEMP_VOLTAGE_VREG_SUB_200 0 // Set Charge Voltage to VREG-200mV during JEITA hig temperature
#define BQ_JEITA_HIGH_TEMP_VOLTAGE_VREG 1         // Set Charge Voltage to VREG during JEITA high temperature

/*============================================================reg 0d============================================================*/
#define BQ_VINDPM_MODE_RELATIVE 0
#define BQ_VINDPM_MODE_ABSOLUTE 1

/*============================================================status============================================================*/
#define BQ_STATUS_VBUS_NO_INPUT 0
#define BQ_STATUS_VBUS_SDP 1
#define BQ_STATUS_VBUS_ADAP 2
#define BQ_STATUS_VBUS_OTG 7

#define BQ_STATUS_NO_CHARGE 0
#define BQ_STATUS_PRE_CHARGE 1
#define BQ_STATUS_FAST_CHARGE 2
#define BQ_STATUS_CHARGE_DONE 3

#define BQ_STATUS_CHARGE_OK 0
#define BQ_STATUS_CHARGE_INPUT_FAULT 1
#define BQ_STATUS_CHARGE_THERMAL_FAULT 2
#define BQ_STATUS_CHARGE_SAFETY_TIMER_FAULT 3

#define BQ_STATUS_NTC_OK 0
#define BQ_STATUS_NTC_WARM 2
#define BQ_STATUS_NTC_COOL 3
#define BQ_STATUS_NTC_COLD 5
#define BQ_STATUS_NTC_HOT 6

/*

typedef struct
{
    // input
    //  00
    uint8_t input_current_lim; // Input Current Limit (100 to 3250),50 ma step
    // 01
    uint8_t input_volt_lim_offset;
    // 02
    uint8_t input_current_optimizer;
    uint8_t force_input_detection;
    uint8_t auto_input_detection_enable;
    // 09
    uint8_t force_start_input_current_optimizer;

    // otg
    // 03
    uint8_t boost_mode_enable;
    //  0a
    uint8_t boost_mode_voltage_regulation;
    uint8_t boost_mode_current_limit;

    // charge
    //  03
    uint8_t charge_enable;
    uint8_t min_batt_voltage_to_exit_bst_mode;
    // 04
    uint8_t fast_charge_current_limit;
    // 05
    uint8_t pre_charge_current_limit;
    uint8_t termination_current_limit;
    // 06
    uint8_t charge_voltage_limit;
    uint8_t bat_pre_charge_to_fast_charge_threshold;
    uint8_t bat_re_charging_threshold_offset;
    // 07
    uint8_t charge_termination_enable;
    uint8_t charging_safety_timer_enable;
    uint8_t fast_charge_timer_setting;

    // sys
    // reg 03
    uint8_t i2c_wtd_feed;
    uint8_t min_system_voltage;

    // reg 07
    uint8_t stat_pin_disable;
    uint8_t i2c_wtd_config;
    // reg 09
    uint8_t force_BATFET_off;
    uint8_t BATFET_turn_off_delay;
    uint8_t BATFET_full_system_reset_enable;

} bq_common_config_t;

typedef struct
{
    // reg 0e ro
    uint8_t ADC_BAT_voltage;
    // reg 0f ro
    uint8_t ADC_SYS_voltage;
    // reg 10 ro
    uint8_t ADC_TS_voltage;
    // reg 11 ro
    uint8_t VBUS_good;
    // reg 12 ro
    uint8_t ADC_charge_current;
} bq_ADC_resault_t;

typedef struct
{
    bq_ADC_resault_t adc;

    /////////////
    // 0b
    uint8_t VBUS_status; // ro
    /////////////
    // 0c ro
    uint8_t bst_mode_fault_status;
    /////////////
    // 0b
    uint8_t charge_status; // ro
    // 0c ro
    uint8_t charge_fault_status;
    /////////////
    // reg 0b
    uint8_t pg;                     // ro
    uint8_t VSYS_regulation_status; // ro
    // reg 0c ro
    uint8_t wtd_fault_status;
    uint8_t batt_fault_status;
    uint8_t NTC_fault_status;
    // reg 0e ro
    uint8_t thermal_regulation_status;
    // reg 11 ro
    uint8_t VBUS_good;
    // reg 13 ro
    uint8_t VNDPM_status;
    uint8_t INDPM_status;
    uint8_t input_current_limit_with_ico;
    // reg 14
    uint8_t ico_status;          // ro
    uint8_t id;                  // ro
    uint8_t temperature_profile; // ro
    uint8_t dev_rev;             // ro
} bq_readBack_resault_t;

<TODO> full reg config
typedef struct
{
    // 00
    uint8_t input_current_lim; // Input Current Limit (100 to 3250),50 ma step
    // 01
    uint8_t input_volt_lim_offset;
    // 02
    uint8_t input_current_optimizer;
    uint8_t force_input_detection;
    uint8_t auto_input_detection_enable;
    // 09
    uint8_t force_start_input_current_optimizer;
    // 0b
    uint8_t VBUS_status; // ro
} bq_input_config_t;     // 7

typedef struct
{
    // 01
    uint8_t bst_mode_thermal_prot_hot;
    uint8_t bst_mode_thermal_prot_cold;
    // 03
    uint8_t boost_mode_enable;
    //  0a
    uint8_t boost_mode_voltage_regulation;
    uint8_t PFM_mode_allow_in_boost_mode;
    uint8_t boost_mode_current_limit;
    // 0c ro
    uint8_t bst_mode_fault_status;
} bq_otg_config_t; // 7

typedef struct
{
    // 03
    uint8_t charge_enable;
    uint8_t min_batt_voltage_to_exit_bst_mode;
    // 04
    uint8_t fast_charge_current_limit;
    // 05
    uint8_t pre_charge_current_limit;
    uint8_t termination_current_limit;
    // 06
    uint8_t charge_voltage_limit;
    uint8_t bat_pre_charge_to_fast_charge_threshold;
    uint8_t bat_re_charging_threshold_offset;
    // 07
    uint8_t charge_termination_enable;
    uint8_t charging_safety_timer_enable;
    uint8_t fast_charge_timer_setting;
    // 0b
    uint8_t charge_status; // ro
    // 0c ro
    uint8_t charge_fault_status;
} bq_charge_config_t; // 13

typedef struct
{
    // reg 00
    uint8_t hiz_enable;      // REGN LDO off
    uint8_t ilim_pin_enable; // limits input current to the lower value of ILIM pin and IILIM register (ICO_EN = 0) or IDPM_LIM register(ICO_EN = 1).
    // reg 02
    uint8_t adc_conv_start;
    uint8_t adc_conv_rate;
    uint8_t bst_mode_freq;
    // reg 03
    uint8_t batt_load_enable;
    uint8_t i2c_wtd_feed;
    uint8_t min_system_voltage;
    // reg 04
    uint8_t current_pluse_enable;
    // reg 07
    uint8_t stat_pin_disable;
    uint8_t i2c_wtd_config;
    uint8_t JEITA_low_temp_current_setting;
    // reg 08
    uint8_t IR_compensation_resistor_setting;
    uint8_t IR_compensation_voltage_clamp;
    uint8_t thermal_regulation_threshold;
    // reg 09
    uint8_t safety_timer_setting_during_DPM_or_thermal_regulation;
    uint8_t force_BATFET_off;
    uint8_t JEITA_high_temp_voltage_setting;
    uint8_t BATFET_turn_off_delay;
    uint8_t BATFET_full_system_reset_enable;
    uint8_t current_pluse_control_voltage_up_enable;
    uint8_t current_pluse_control_voltage_down_enable;
    // reg 0b
    uint8_t pg;                     // ro
    uint8_t VSYS_regulation_status; // ro
    // reg 0c ro
    uint8_t wtd_fault_status;
    uint8_t batt_fault_status;
    uint8_t NTC_fault_status;
    // reg 0d
    uint8_t VINDPM_threshold_setting_method;
    uint8_t absolute_VINDPM_threshold;
    // reg 0e ro
    uint8_t thermal_regulation_status;
    uint8_t ADC_BAT_voltage;
    // reg 0f ro
    uint8_t ADC_SYS_voltage;
    // reg 10 ro
    uint8_t ADC_TS_voltage;
    // reg 11 ro
    uint8_t VBUS_good;
    uint8_t ADC_VBUS_voltage;
    // reg 12 ro
    uint8_t ADC_charge_current;
    // reg 13 ro
    uint8_t VNDPM_status;
    uint8_t INDPM_status;
    uint8_t input_current_limit_with_ico;
    // reg 14
    uint8_t reg_reset;
    uint8_t ico_status;          // ro
    uint8_t id;                  // ro
    uint8_t temperature_profile; // ro
    uint8_t dev_rev;             // ro
} bq_system_config_t;            // 44



typedef struct
{
    reg 00
    uint8_t hiz_enable;        // REGN LDO off
    uint8_t ilim_pin_enable;   // limits input current to the lower value of ILIM pin and IILIM register (ICO_EN = 0) or IDPM_LIM register(ICO_EN = 1).
    uint8_t input_current_lim; // Input Current Limit (100 to 3250),50 ma step
    reg 01
    uint8_t bst_mode_thermal_prot_hot;
    uint8_t bst_mode_thermal_prot_cold;
    uint8_t input_volt_lim_offset;
    reg 02
    uint8_t adc_conv_start;
    uint8_t adc_conv_rate;
    uint8_t bst_mode_freq;
    uint8_t input_current_optimizer;
    uint8_t force_input_detection;
    uint8_t auto_input_detection_enable;
    reg 03
    uint8_t batt_load_enable;
    uint8_t i2c_wtd_feed;
    uint8_t boost_mode_enable;
    uint8_t charge_enable;
    uint8_t min_system_voltage;
    uint8_t min_batt_voltage_to_exit_bst_mode;
    reg 04
    uint8_t current_pluse_enable;
    uint8_t fast_charge_current_limit;
    reg 05
    uint8_t pre_charge_current_limit;
    uint8_t termination_current_limit;
    reg 06
    uint8_t charge_voltage_limit;
    uint8_t bat_pre_charge_to_fast_charge_threshold;
    uint8_t bat_re_charging_threshold_offset;
    reg 07
    uint8_t charge_termination_enable;
    uint8_t stat_pin_disable;
    uint8_t i2c_wtd_config;
    uint8_t charging_safety_timer_enable;
    uint8_t fast_charge_timer_setting;
    uint8_t JEITA_low_temp_current_setting;
    reg 08
    uint8_t IR_compensation_resistor_setting;
    uint8_t IR_compensation_voltage_clamp;
    uint8_t thermal_regulation_threshold;
    reg 09
    uint8_t force_start_input_current_optimizer;
    uint8_t safety_timer_setting_during_DPM_or_thermal_regulation;
    uint8_t force_BATFET_off;
    uint8_t JEITA_high_temp_voltage_setting;
    uint8_t BATFET_turn_off_delau;
    uint8_t BATFET_full_system_reset_enable;
    uint8_t current_pluse_control_voltage_up_enable;
    uint8_t current_pluse_control_voltage_down_enable;
    reg 0a
    uint8_t boost_mode_voltage_regulation;
    uint8_t PFM_mode_allow_in_boost_mode;
    uint8_t boost_mode_current_limit;
    reg 0b
    uint8_t VBUS_status;
    uint8_t charge_status;
    uint8_t pg;
    uint8_t VSYS_regulation_status;
    reg 0c
    uint8_t wtd_fault_status;
    uint8_t bst_mode_fault_status;
    uint8_t charge_fault_status;
    uint8_t batt_fault_status;
    uint8_t NTC_fault_status;
    reg 0d
    uint8_t VINDPM_threshold_setting_method;
    uint8_t absolute_VINDPM_threshold;
    reg 0e
    uint8_t thermal_regulation_status;
    uint8_t ADC_BAT_voltage;
    reg 0f
    uint8_t ADC_SYS_voltage;
    reg 10
    uint8_t ADC_TS_voltage;
    reg 11
    uint8_t VBUS_good;
    uint8_t ADC_VBUS_voltage;
    reg 12
    uint8_t ADC_charge_current;
    reg 13
    uint8_t VNDPM_status;
    uint8_t INDPM_status;
    uint8_t input_current_limit_with_ico;
    reg 14
    uint8_t reg_reset;
    uint8_t ico_status;
    uint8_t id;
    uint8_t temperature_profile;
    uint8_t dev_rev;

} bq_all_reg_t;71

*/

// typedef struct
// {
//     //  reg 00
//     uint8_t hiz_enable : 1;        // REGN LDO off
//     uint8_t ilim_pin_enable : 1;   // limits input current to the lower value of ILIM pin and IILIM register (ICO_EN = 0) or IDPM_LIM register(ICO_EN = 1).
//     uint8_t input_current_lim : 6; // Input Current Limit (100 to 3250),50 ma step
//     // reg 01
//     uint8_t bst_mode_thermal_prot_hot : 2;  // Boost Mode Hot Temperature Monitor Threshold
//     uint8_t bst_mode_thermal_prot_cold : 1; // Boost Mode Cold Temperature Monitor Threshold
//     uint8_t input_volt_lim_offset : 5;      // Input Voltage Limit Offset
//     // reg 02
//     uint8_t adc_conv_start : 1;          // ADC Conversion Start Control
//     uint8_t adc_conv_rate : 1;           // ADC Conversion Rate Selection
//     uint8_t bst_mode_freq : 1;           // Boost Mode Frequency Selection
//     uint8_t input_current_optimizer : 1; // Input Current Optimizer (ICO) Enable
//     uint8_t : 2;
//     uint8_t force_input_detection : 1;       // Force Input Detection
//     uint8_t auto_input_detection_enable : 1; // Automatic Input Detection Enable
//     // reg 03
//     uint8_t batt_load_enable : 1;                  // Battery Load (IBATLOAD) Enable
//     uint8_t i2c_wtd_feed : 1;                      // I2C Watchdog Timer Reset
//     uint8_t boost_mode_enable : 1;                 // Boost (OTG) Mode Configuration
//     uint8_t charge_enable : 1;                     // Charge Enable Configuration
//     uint8_t min_system_voltage : 3;                // Minimum System Voltage Limit
//     uint8_t min_batt_voltage_to_exit_bst_mode : 1; // Minimum Battery Voltage (falling) to exit boost mode
//     // reg 04
//     uint8_t current_pluse_enable : 1;      // Current pulse control Enable
//     uint8_t fast_charge_current_limit : 7; // Fast Charge Current Limit
//     // reg 05
//     uint8_t pre_charge_current_limit : 4;  // Precharge Current Limit
//     uint8_t termination_current_limit : 4; // Termination Current Limit
//     // reg 06
//     uint8_t charge_voltage_limit : 6;                    // Charge Voltage Limit
//     uint8_t bat_pre_charge_to_fast_charge_threshold : 1; // Battery Precharge to Fast Charge Threshold
//     uint8_t bat_re_charging_threshold_offset : 1;        // Battery Recharge Threshold Offset
//     // reg 07
//     uint8_t charge_termination_enable : 1;      // Charging Termination Enable
//     uint8_t stat_pin_disable : 1;               // STAT Pin Disable
//     uint8_t i2c_wtd_config : 2;                 // I2C Watchdog Timer Setting
//     uint8_t charging_safety_timer_enable : 1;   // Charging Safety Timer Enable
//     uint8_t fast_charge_timer_setting : 2;      // Fast Charge Timer Setting
//     uint8_t JEITA_low_temp_current_setting : 1; // JEITA Low Temperature Current Setting
//     // reg 08
//     uint8_t IR_compensation_resistor_setting : 3; // IR Compensation Resistor Setting
//     uint8_t IR_compensation_voltage_clamp : 3;    // IR Compensation Voltage Clamp
//     uint8_t thermal_regulation_threshold : 2;     // Thermal Regulation Threshold
//     // reg 09
//     uint8_t force_start_input_current_optimizer : 1;                   // Force Start Input Current Optimizer (ICO)
//     uint8_t safety_timer_setting_during_DPM_or_thermal_regulation : 1; // Safety Timer Setting during DPM or Thermal Regulation
//     uint8_t force_BATFET_off : 1;                                      // Force BATFET off to enable ship mode
//     uint8_t JEITA_high_temp_voltage_setting : 1;                       // JEITA High Temperature Voltage Setting
//     uint8_t BATFET_turn_off_delay : 1;                                 // BATFET turn off delay control
//     uint8_t BATFET_full_system_reset_enable : 1;                       // BATFET full system reset enable
//     uint8_t current_pluse_control_voltage_up_enable : 1;               // Current pulse control voltage up enable
//     uint8_t current_pluse_control_voltage_down_enable : 1;             // Current pulse control voltage down enable
//     // reg 0a
//     uint8_t boost_mode_voltage_regulation : 4; // Boost Mode Voltage Regulation
//     uint8_t PFM_mode_allow_in_boost_mode : 1;  // PFM mode allowed in boost mode
//     uint8_t boost_mode_current_limit : 3;      // Boost Mode Current Limit
//     // RO regs
//     //  reg 0b
//     uint8_t VBUS_status : 3;            // VBUS Status register
//     uint8_t charge_status : 2;          // Charging Status
//     uint8_t pg : 1;                     // Power Good Status
//     uint8_t : 1;                        // Reserved: Always reads 1
//     uint8_t VSYS_regulation_status : 1; // VSYS Regulation Status
//     // reg 0c
//     uint8_t wtd_fault_status : 1;      // Watchdog Fault Status
//     uint8_t bst_mode_fault_status : 1; // Boost Mode Fault Status
//     uint8_t charge_fault_status : 2;   // Charge Fault Status
//     uint8_t batt_fault_status : 1;     // Battery Fault Status
//     uint8_t NTC_fault_status : 3;      // NTC Fault Status
//     // reg 0d rw
//     uint8_t VINDPM_threshold_setting_method : 1; // VINDPM Threshold Setting Method Note: Register is reset to default value when input source is plugged-in
//     uint8_t absolute_VINDPM_threshold : 7;       // Absolute VINDPM Threshold Note: Register is reset to default value when input source is plugged-in
//     // reg 0e
//     uint8_t thermal_regulation_status : 1; // Thermal Regulation Status
//     uint8_t ADC_BAT_voltage : 7;           // ADC conversion of Battery Voltage (VBAT)
//     // reg 0f
//     uint8_t : 1;
//     uint8_t ADC_SYS_voltage : 7; // ADDC conversion of System Voltage (VSYS)
//     // reg 10
//     uint8_t : 1;
//     uint8_t ADC_TS_voltage : 7; // ADC conversion of TS Voltage (TS) as percentage of REGN
//     // reg 11
//     uint8_t VBUS_good : 1;        // VBUS Good Status
//     uint8_t ADC_VBUS_voltage : 7; // ADC conversion of VBUS voltage (VBUS)
//     // reg 12
//     uint8_t : 1;
//     uint8_t ADC_charge_current : 7; // ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT
//     // reg 13
//     uint8_t VNDPM_status : 1;                 // VINDPM Status
//     uint8_t INDPM_status : 1;                 // IINDPM Status
//     uint8_t input_current_limit_with_ico : 6; // Input Current Limit in effect while Input Current Optimizer (ICO) is enabled
//     // reg 14
//     uint8_t reg_reset : 1;           // Register Reset(RW)
//     uint8_t ico_status : 1;          // Input Current Optimizer (ICO) Status
//     uint8_t id : 3;                  // Device Configuration
//     uint8_t temperature_profile : 1; // Temperature Profile
//     uint8_t dev_rev : 2;             // Device Revision: 10

// } __attribute__((packed)) bq25896_all_reg_t; // 71

typedef struct
{
    //  reg 00
    uint8_t input_current_lim : 6; // Input Current Limit (100 to 3250),50 ma step
    uint8_t ilim_pin_enable : 1;   // limits input current to the lower value of ILIM pin and IILIM register (ICO_EN = 0) or IDPM_LIM register(ICO_EN = 1).
    uint8_t hiz_enable : 1;        // REGN LDO off
    // reg 01
    uint8_t input_volt_lim_offset : 5;      // Input Voltage Limit Offset
    uint8_t bst_mode_thermal_prot_cold : 1; // Boost Mode Cold Temperature Monitor Threshold
    uint8_t bst_mode_thermal_prot_hot : 2;  // Boost Mode Hot Temperature Monitor Threshold
    // reg 02
    uint8_t auto_input_detection_enable : 1; // Automatic Input Detection Enable
    uint8_t force_input_detection : 1;       // Force Input Detection
    uint8_t : 2;
    uint8_t input_current_optimizer : 1; // Input Current Optimizer (ICO) Enable
    uint8_t bst_mode_freq : 1;           // Boost Mode Frequency Selection
    uint8_t adc_conv_rate : 1;           // ADC Conversion Rate Selection
    uint8_t adc_conv_start : 1;          // ADC Conversion Start Control
    // reg 03
    uint8_t min_batt_voltage_to_exit_bst_mode : 1; // Minimum Battery Voltage (falling) to exit boost mode
    uint8_t min_system_voltage : 3;                // Minimum System Voltage Limit
    uint8_t charge_enable : 1;                     // Charge Enable Configuration
    uint8_t boost_mode_enable : 1;                 // Boost (OTG) Mode Configuration
    uint8_t i2c_wtd_feed : 1;                      // I2C Watchdog Timer Reset
    uint8_t batt_load_enable : 1;                  // Battery Load (IBATLOAD) Enable
    // reg 04
    uint8_t fast_charge_current_limit : 7; // Fast Charge Current Limit
    uint8_t current_pluse_enable : 1;      // Current pulse control Enable
    // reg 05
    uint8_t termination_current_limit : 4; // Termination Current Limit
    uint8_t pre_charge_current_limit : 4;  // Precharge Current Limit
    // reg 06
    uint8_t bat_re_charging_threshold_offset : 1;        // Battery Recharge Threshold Offset
    uint8_t bat_pre_charge_to_fast_charge_threshold : 1; // Battery Precharge to Fast Charge Threshold
    uint8_t charge_voltage_limit : 6;                    // Charge Voltage Limit
    // reg 07
    uint8_t JEITA_low_temp_current_setting : 1; // JEITA Low Temperature Current Setting
    uint8_t fast_charge_timer_setting : 2;      // Fast Charge Timer Setting
    uint8_t charging_safety_timer_enable : 1;   // Charging Safety Timer Enable
    uint8_t i2c_wtd_config : 2;                 // I2C Watchdog Timer Setting
    uint8_t stat_pin_disable : 1;               // STAT Pin Disable
    uint8_t charge_termination_enable : 1;      // Charging Termination Enable
    // reg 08
    uint8_t thermal_regulation_threshold : 2;     // Thermal Regulation Threshold
    uint8_t IR_compensation_voltage_clamp : 3;    // IR Compensation Voltage Clamp
    uint8_t IR_compensation_resistor_setting : 3; // IR Compensation Resistor Setting
    // reg 09
    uint8_t current_pluse_control_voltage_down_enable : 1;             // Current pulse control voltage down enable
    uint8_t current_pluse_control_voltage_up_enable : 1;               // Current pulse control voltage up enable
    uint8_t BATFET_full_system_reset_enable : 1;                       // BATFET full system reset enable
    uint8_t BATFET_turn_off_delay : 1;                                 // BATFET turn off delay control
    uint8_t JEITA_high_temp_voltage_setting : 1;                       // JEITA High Temperature Voltage Setting
    uint8_t force_BATFET_off : 1;                                      // Force BATFET off to enable ship mode
    uint8_t safety_timer_setting_during_DPM_or_thermal_regulation : 1; // Safety Timer Setting during DPM or Thermal Regulation
    uint8_t force_start_input_current_optimizer : 1;                   // Force Start Input Current Optimizer (ICO)
    // reg 0a
    uint8_t boost_mode_current_limit : 3;      // Boost Mode Current Limit
    uint8_t PFM_mode_allow_in_boost_mode : 1;  // PFM mode allowed in boost mode
    uint8_t boost_mode_voltage_regulation : 4; // Boost Mode Voltage Regulation
    // RO regs
    //  reg 0b
    uint8_t VSYS_regulation_status : 1; // VSYS Regulation Status
    uint8_t : 1;                        // Reserved: Always reads 1
    uint8_t pg : 1;                     // Power Good Status
    uint8_t charge_status : 2;          // Charging Status
    uint8_t VBUS_status : 3;            // VBUS Status register
    // reg 0c
    uint8_t NTC_fault_status : 3;      // NTC Fault Status
    uint8_t batt_fault_status : 1;     // Battery Fault Status
    uint8_t charge_fault_status : 2;   // Charge Fault Status
    uint8_t bst_mode_fault_status : 1; // Boost Mode Fault Status
    uint8_t wtd_fault_status : 1;      // Watchdog Fault Status
    // reg 0d rw
    uint8_t absolute_VINDPM_threshold : 7;       // Absolute VINDPM Threshold Note: Register is reset to default value when input source is plugged-in
    uint8_t VINDPM_threshold_setting_method : 1; // VINDPM Threshold Setting Method Note: Register is reset to default value when input source is plugged-in
    // reg 0e
    uint8_t ADC_BAT_voltage : 7;           // ADC conversion of Battery Voltage (VBAT)
    uint8_t thermal_regulation_status : 1; // Thermal Regulation Status
    // reg 0f
    uint8_t ADC_SYS_voltage : 7; // ADDC conversion of System Voltage (VSYS)
    uint8_t : 1;
    // reg 10
    uint8_t ADC_TS_voltage : 7; // ADC conversion of TS Voltage (TS) as percentage of REGN
    uint8_t : 1;
    // reg 11
    uint8_t ADC_VBUS_voltage : 7; // ADC conversion of VBUS voltage (VBUS)
    uint8_t VBUS_good : 1;        // VBUS Good Status
    // reg 12
    uint8_t ADC_charge_current : 7; // ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT
    uint8_t : 1;
    // reg 13
    uint8_t input_current_limit_with_ico : 6; // Input Current Limit in effect while Input Current Optimizer (ICO) is enabled
    uint8_t INDPM_status : 1;                 // IINDPM Status
    uint8_t VNDPM_status : 1;                 // VINDPM Status
    // reg 14
    uint8_t dev_rev : 2;             // Device Revision: 10
    uint8_t temperature_profile : 1; // Temperature Profile
    uint8_t id : 3;                  // Device Configuration
    uint8_t ico_status : 1;          // Input Current Optimizer (ICO) Status
    uint8_t reg_reset : 1;           // Register Reset(RW)

} __attribute__((packed)) bq25896_all_reg_t; // 71

/*============================================================config start============================================================*/
typedef struct
{
    //  reg 00
    uint8_t ilim_pin_enable;       // limits input current to the lower value of ILIM pin and IILIM register (ICO_EN = 0) or IDPM_LIM register(ICO_EN = 1).
    uint16_t input_current_lim_ma; // Input Current Limit (100 to 3250),50ma step,Offset: 100mA
    //  reg 01
    uint16_t input_voltage_lim_offset_mv; // Input Voltage Limit Offset (0 to 3100),100mV step
    //  reg 02
    uint8_t input_current_optimizer_enable; // Input Current Optimizer (ICO) Enable
    uint8_t force_input_detection_enable;   // Force Input Detection
    uint8_t auto_input_detection_enable;    // Automatic Input Detection Enable
    //  reg 09
    uint8_t force_start_input_current_optimizer; // Force Start Input Current Optimizer (ICO)
    //  reg 0d
    uint8_t VINDPM_threshold_setting_method; // VINDPM Threshold Setting Method Note: Register is reset to default value when input source is plugged-in
    uint16_t absolute_VINDPM_threshold_mv;   // Absolute VINDPM Threshold Note: Register is reset to default value when input source is plugged-in, Register is read only when FORCE_VINDPM=0  (3900 to 15300),100mV step,Offset: 2.6V

} __attribute__((packed)) bq25896_input_config_t;

typedef struct
{
    // reg 03
    uint8_t charge_enable; // Charge Enable Configuration
    // reg 04
    uint16_t fast_charge_current_limit_ma; // Fast Charge Current Limit(0 to 3008),64ma step
    // reg 05
    uint16_t pre_charge_current_limit_ma;  // Precharge Current Limit(64 to 1024),64ma step,Offset: 64mA
    uint16_t termination_current_limit_ma; // Termination Current Limit(64 to 1024),64ma step,Offset: 64mA
    // reg 06
    uint16_t charge_voltage_limit_mv;                // Charge Voltage Limit(3840 to 1608),16mv step,Offset: 3.840V
    uint8_t bat_pre_charge_to_fast_charge_threshold; // Battery Precharge to Fast Charge Threshold
    uint8_t bat_re_charging_threshold_offset;        // Battery Recharge Threshold Offset
    // reg 07
    uint8_t charge_termination_enable;    // Charging Termination Enable
    uint8_t charging_safety_timer_enable; // Charging Safety Timer Enable
    uint8_t fast_charge_timer_setting;    // Fast Charge Timer Setting

} __attribute__((packed)) bq25896_charge_config_t;

typedef struct
{
    // reg 01
    uint8_t bst_mode_thermal_prot_hot;  // Boost Mode Hot Temperature Monitor Threshold
    uint8_t bst_mode_thermal_prot_cold; // Boost Mode Cold Temperature Monitor Threshold
    // reg 02
    uint8_t bst_mode_freq; // Boost Mode Frequency Selection
    // reg 03
    uint8_t boost_mode_enable; // Boost (OTG) Mode Configuration
    // reg 0a
    uint16_t boost_mode_voltage_mv;                         // Boost Mode Voltage Regulation(4550 to 5510),64vv step,Offset: 4.55V
    uint8_t PFM_mode_allow_in_boost_mode;                   // PFM mode allowed in boost mode
    bq25896_boost_current_limit_t boost_mode_current_limit; // Boost Mode Current Limit

} __attribute__((packed)) bq25896_boost_config_t;

typedef struct
{
    // reg 00
    uint8_t hiz_enable; // REGN LDO off
    // reg 02
    uint8_t adc_conv_enable; // ADC Conversion Start Control This bit is read-only when CONV_RATE = 1
    uint8_t adc_conv_rate;   // ADC Conversion Rate Selection
    // reg 03
    uint8_t batt_load_enable;                  // Battery Load (IBATLOAD) Enable
    uint16_t min_system_voltage_mv;            // Minimum System Voltage Limit(3000 to 3700),100mv step,Offset: 3.0V
    uint8_t min_batt_voltage_to_exit_bst_mode; // Minimum Battery Voltage (falling) to exit boost mode
    // reg 04
    uint8_t current_pluse_enable; // Current pulse control Enable
    // reg 07
    uint8_t stat_pin_disable;               // STAT Pin Disable
    uint8_t i2c_wtd_config;                 // I2C Watchdog Timer Setting
    uint8_t JEITA_low_temp_current_setting; // JEITA Low Temperature Current Setting
    // reg 08
    uint8_t IR_compensation_resistor_setting_mr; // IR Compensation Resistor Setting(0 to 140),20mr step
    uint8_t IR_compensation_voltage_clamp_mv;    // IR Compensation Voltage Clamp(0 to 224),32mv step
    uint8_t thermal_regulation_threshold;        // Thermal Regulation Threshold
    // reg 09
    uint8_t safety_timer_slow_during_DPM_or_thermal_regulation_enable; // Safety Timer Setting during DPM or Thermal Regulation
    uint8_t JEITA_high_temp_voltage_setting;                           // JEITA High Temperature Voltage Setting
    uint8_t BATFET_turn_off_delay_enable;                              // BATFET turn off delay control
    uint8_t BATFET_full_system_reset_enable;                           // BATFET full system reset enable

} __attribute__((packed)) bq25896_system_config_t;

/*============================================================config end============================================================*/

/*============================================================status start============================================================*/
typedef struct
{
    // reg 0b ro
    uint8_t VBUS_status;     // VBUS Status register
    uint8_t charge_status;   // Charging Status
    uint8_t pg;              // Power Good Status
    uint8_t VSYS_regulation; // VSYS Regulation Status
    // reg 0c ro
    uint8_t wtd_fault;           // Watchdog Fault Status
    uint8_t bst_overload;        // Boost Mode Fault Status
    uint8_t charge_fault_status; // Charge Fault Status
    uint8_t batt_ovp;            // Battery Fault Status
    uint8_t NTC_fault_status;    // NTC Fault Status
    // reg 0e ro
    uint8_t thermal_regulation; // Thermal Regulation Status
    // reg 11 ro
    uint8_t VBUS_good; // VBUS Good Status
    // reg 13 ro
    uint8_t VNDPM; // VINDPM Status
    uint8_t INDPM; // IINDPM Status
    // reg 14 ro
    uint8_t ico_done; // Input Current Optimizer (ICO) Status

} __attribute__((packed)) bq25896_system_status_t;

typedef struct
{
    // reg 0e ro
    uint16_t ADC_BAT_voltage_mv; // ADC conversion of Battery Voltage (VBAT)
    // reg 0f ro
    uint16_t ADC_SYS_voltage_mv; // ADDC conversion of System Voltage (VSYS)
    // reg 10 ro
    uint32_t ADC_TS_voltage_pst; // ADC conversion of TS Voltage (TS) as percentage of REGN(x1000)
    // reg 11 ro
    uint16_t ADC_VBUS_voltage_mv; // ADC conversion of VBUS voltage (VBUS)
    // reg 12 ro
    uint16_t ADC_charge_current_ma; // ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT
    // reg 13 ro
    uint16_t input_current_limit_with_ico_ma; // Input Current Limit in effect while Input Current Optimizer (ICO) is enabled

} __attribute__((packed)) bq25896_adc_resault_t;

/*============================================================status end============================================================*/

bq25896_error_t bq25896_init();

bq25896_error_t bq25896_set_input_config(bq25896_input_config_t cfg);
bq25896_error_t bq25896_set_charge_config(bq25896_charge_config_t cfg);
bq25896_error_t bq25896_set_boost_config(bq25896_boost_config_t cfg);
bq25896_error_t bq25896_set_system_config(bq25896_system_config_t cfg);

bq25896_error_t bq25896_get_input_config(bq25896_input_config_t *cfg);
bq25896_error_t bq25896_get_charge_config(bq25896_charge_config_t *cfg);
bq25896_error_t bq25896_get_boost_config(bq25896_boost_config_t *cfg);
bq25896_error_t bq25896_get_system_config(bq25896_system_config_t *cfg);

bq25896_error_t bq25896_get_system_status(bq25896_system_status_t *dst);
bq25896_error_t bq25896_get_adc(bq25896_adc_resault_t *dst);

bq25896_error_t bq25896_get_all_reg(bq25896_all_reg_t *reg);

bq25896_error_t bq25896_poweroff();
bq25896_error_t bq25896_wtd_feed();
bq25896_error_t bq25896_current_pluse(bq25896_current_pluse_direction_t dir);
bq25896_error_t bq25896_reg_reset();

void bq25896_print_all_regs(bq25896_all_reg_t *regs);
void bq25896_print_input_config(bq25896_input_config_t *regs);
void bq25896_print_charge_config(bq25896_charge_config_t *regs);
void bq25896_print_boost_config(bq25896_boost_config_t *regs);
void bq25896_print_system_config(bq25896_system_config_t *regs);

void bq25896_print_system_status(bq25896_system_status_t *regs);
void bq25896_print_adc(bq25896_adc_resault_t *regs);
#endif