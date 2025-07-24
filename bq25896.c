#include "bq25896.h"

#include "FreeRTOS.h"
#include "task.h"

#include "i2c.h"
#include "stm32h7xx_ll_i2c.h"

#include "elog.h"

#include <string.h> //memcpy

static const char TAG[] = "bq25896";

#define BQ_25896_IIC_TIMEOUT 5000

#define BQ_25896_IIC_ADDR 0x6b
#define BQ_25896_IIC_INTF hi2c1

#define BQ_25896_NINT_GPIO_PORT GPIOB
#define BQ_25896_NCE_GPIO_PORT GPIOB
#define BQ_25896_POTG_GPIO_PORT GPIOB

#define BQ_25896_NINT_GPIO_PIN GPIO_PIN_5
#define BQ_25896_NCE_GPIO_PIN GPIO_PIN_3
#define BQ_25896_POTG_GPIO_PIN GPIO_PIN_4

#define BQ_25896_IS_ONE_BINARY(x) ((x) ? 1 : 0)

#define BQ_25896_RETURN_ON_FALSE(x, ret, mtag, mmsg, ...) \
    do                                                    \
    {                                                     \
        if (!(x))                                         \
        {                                                 \
            elog_e(mtag, mmsg, ##__VA_ARGS__);            \
            return ret;                                   \
        }                                                 \
    } while (0)

#define BQ_DELAY_MS(x) vTaskDelay(pdMS_TO_TICKS(x))

typedef struct
{
    uint8_t mask; // not lsh
    uint8_t lsh;
    uint8_t dat; // not lsh
} bq_reg_desc_t;

static bq25896_error_t bq25896_check_iic_connect()
{

    if (HAL_I2C_IsDeviceReady(&BQ_25896_IIC_INTF, (BQ_25896_IIC_ADDR << 1), 10, 100) == HAL_OK)
    {
        elog_i(TAG, "bq 25896 connect ok");
        return BQ_OK;
    }
    return BQ_ERR_TRANS;
}

static bq25896_error_t bq25896_read_reg(uint8_t reg, uint8_t *dst, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, (BQ_25896_IIC_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, dst, len, BQ_25896_IIC_TIMEOUT);
    BQ_25896_RETURN_ON_FALSE((ret == HAL_OK), BQ_ERR_TRANS, TAG, "read reg err %d", ret);
    // elog_i(TAG, "bq r %X len=%d", reg, len);
    // elog_hexdump(TAG, 8, dst, len);
    return BQ_OK;
}

static bq25896_error_t bq25896_write_reg(uint8_t reg, uint8_t *src, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, (BQ_25896_IIC_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, src, len, BQ_25896_IIC_TIMEOUT);
    BQ_25896_RETURN_ON_FALSE((ret == HAL_OK), BQ_ERR_TRANS, TAG, "write reg err %d", ret);
    // elog_i(TAG, "bq w %X len=%d", reg, len);
    // elog_hexdump(TAG, 8, src, len);
    return BQ_OK;
}

static bq25896_error_t bq_25896_print_id()
{
    uint8_t tmp = 0x00;
    bq25896_error_t ret = bq25896_read_reg(0x14, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read id err %d", ret);
    elog_i(TAG, "bq id : %02x", tmp & 0x3f); // 0x06 is ok
    return BQ_OK;
}

/**
 *
 * @brief modify reg
 *
 * @param reg target register
 *
 * @param src modify value (not lsh) with mask and
 * @param mask mask(not lsh)
 * @param lsh src lsh num
 *
 * @note no range check
 *
 * @example bq_25896_modify_reg(0x0,0x1,2,2) is 0b00001100
 */
static bq25896_error_t bq_25896_modify_reg(uint8_t reg, uint8_t src, uint8_t mask, uint8_t lsh)
{
    elog_i(TAG, "mreg 0x%x  msk=0x%x,lsh=%d,val=%d", reg, mask, lsh, src);
    uint8_t tmp = 0;
    bq25896_error_t ret = bq25896_read_reg(reg, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", reg);
    elog_i(TAG, "mreg 0x%x read 0x%x", reg, tmp);

    src = src & mask; // process no lsh dat with no lsh mask

    mask = mask << lsh;

    tmp = tmp & ~(mask);

    src = (src << lsh);

    tmp = tmp | (src);

    ret = bq25896_write_reg(reg, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", reg);
    elog_i(TAG, "mreg %x write 0x%x", reg, tmp);
    return BQ_OK;
}

static bq25896_error_t bq_25896_modify_muti_regs(uint8_t reg, bq_reg_desc_t *desc, uint8_t len)
{

    uint8_t reg_data = 0;
    bq25896_error_t ret = bq25896_read_reg(reg, &reg_data, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", reg);
    elog_i(TAG, "mreg 0x%x read 0x%x", reg, reg_data);

    elog_i(TAG, "modify %d regs", len);

    uint8_t msk_temp = 0;
    uint8_t val_temp = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        elog_i(TAG, "mreg[%d] msk=0x%x,lsh=%d,val=%d", i, desc[i].mask, desc[i].lsh, desc[i].dat);
        msk_temp = desc[i].mask;
        val_temp = desc[i].dat;

        val_temp &= msk_temp; // process no lsh dat with no lsh mask

        msk_temp <<= desc[i].lsh;
        val_temp <<= desc[i].lsh;

        reg_data &= ~(msk_temp);
        reg_data |= (val_temp);
    }

    ret = bq25896_write_reg(reg, &reg_data, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", reg);
    elog_i(TAG, "mreg 0x%x write 0x%x", reg, reg_data);
    return BQ_OK;
}

/// @brief wait for reg to target status
/// @param reg target reg
/// @param mask bit mask
/// @param target status
/// @param timeout loop counter for timeout
/// @return ok or timeout or io error
static bq25896_error_t bq_25896_auto_poll(uint8_t reg, uint8_t mask, uint8_t target, uint32_t timeout)
{
    uint8_t tmp = 0;
    for (uint32_t i = 0; i < timeout; i++)
    {
        bq25896_error_t ret = bq25896_read_reg(reg, &tmp, 1);
        BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", reg);
        tmp = tmp & mask;
        if (tmp == target)
        {
            return BQ_OK;
        }
        BQ_DELAY_MS(1);
    }
    return BQ_ERR_TIMEOUT;
}

/*============================================================basic api============================================================*/

bq25896_error_t bq25896_init()
{
    bq25896_error_t ret = bq25896_check_iic_connect();
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "check conn err %d", ret);
    ret = bq_25896_print_id();
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read id err %d", ret);

    HAL_GPIO_WritePin(BQ_25896_NCE_GPIO_PORT, BQ_25896_NCE_GPIO_PIN, 0); // ce low

    return BQ_OK;
}

/*============================================================set api============================================================*/

bq25896_error_t bq25896_set_input_config(bq25896_input_config_t cfg)
{
    /*
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
    uint16_t absolute_VINDPM_threshold_mv;   // Absolute VINDPM Threshold Note: Register is reset to default value when input source is plugged-in(3900 to 15300),100mV step,Offset: 2.6V
    */
    bq25896_error_t ret;

    // 0
    cfg.input_current_lim_ma = cfg.input_current_lim_ma - 100;
    cfg.input_current_lim_ma = cfg.input_current_lim_ma / 50;
    bq_reg_desc_t xfer_desc_for_reg0x00[2] = {
        {
            .mask = 0x01,
            .lsh = 6,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.ilim_pin_enable),
        },
        {
            .mask = 0x3f,
            .lsh = 0,
            .dat = cfg.input_current_lim_ma,
        },
    };
    ret = bq_25896_modify_muti_regs(0x00, xfer_desc_for_reg0x00, 2);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x00);
    // 1
    ret = bq_25896_modify_reg(0x01, (cfg.input_voltage_lim_offset_mv / 100), 0x1f, 0);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x01);
    // 2
    bq_reg_desc_t xfer_desc_for_reg0x02[3] = {
        {
            .mask = 0x01,
            .lsh = 4,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.input_current_optimizer_enable),
        },
        {
            .mask = 0x01,
            .lsh = 1,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.force_input_detection_enable),
        },
        {
            .mask = 0x01,
            .lsh = 0,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.auto_input_detection_enable),
        },
    };
    ret = bq_25896_modify_muti_regs(0x02, xfer_desc_for_reg0x02, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x02);
    // 9
    ret = bq_25896_modify_reg(0x09, BQ_25896_IS_ONE_BINARY(cfg.force_start_input_current_optimizer), 0x01, 7);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x09);
    // d
    cfg.absolute_VINDPM_threshold_mv = cfg.absolute_VINDPM_threshold_mv - 2600;
    cfg.absolute_VINDPM_threshold_mv = cfg.absolute_VINDPM_threshold_mv / 100;

    bq_reg_desc_t xfer_desc_for_reg0x0d[2] = {
        {
            .mask = 0x01,
            .lsh = 7,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.VINDPM_threshold_setting_method),
        },
        {
            .mask = 0x7f,
            .lsh = 0,
            .dat = cfg.absolute_VINDPM_threshold_mv,
        },
    };
    ret = bq_25896_modify_muti_regs(0x0d, xfer_desc_for_reg0x0d, 2);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0d);
    // // 0
    // ret = bq_25896_modify_reg(0x00, BQ_25896_IS_ONE_BINARY(cfg.ilim_pin_enable), 0x01, 6);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0);
    // cfg.input_current_lim_ma = cfg.input_current_lim_ma - 100;
    // cfg.input_current_lim_ma = cfg.input_current_lim_ma / 50;
    // ret = bq_25896_modify_reg(0x0, (uint8_t)cfg.input_current_lim_ma, 0x3f, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0);
    // // 1
    // cfg.input_voltage_lim_offset_mv = cfg.input_voltage_lim_offset_mv / 100;
    // ret = bq_25896_modify_reg(0x01, (uint8_t)cfg.input_voltage_lim_offset_mv, 0x1f, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 1);
    // // 2
    // ret = bq_25896_modify_reg(0x02, BQ_25896_IS_ONE_BINARY(cfg.input_current_optimizer_enable), 0x01, 4);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 2);
    // ret = bq_25896_modify_reg(0x02, BQ_25896_IS_ONE_BINARY(cfg.force_input_detection_enable), 0x01, 1);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 2);
    // ret = bq_25896_modify_reg(0x02, BQ_25896_IS_ONE_BINARY(cfg.auto_input_detection_enable), 0x01, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 2);
    // // 9
    // ret = bq_25896_modify_reg(0x09, BQ_25896_IS_ONE_BINARY(cfg.force_start_input_current_optimizer), 0x01, 7);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 9);
    // // d
    // ret = bq_25896_modify_reg(0x0d, BQ_25896_IS_ONE_BINARY(cfg.VINDPM_threshold_setting_method), 0x01, 7);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0xd);
    // cfg.absolute_VINDPM_threshold_mv = cfg.absolute_VINDPM_threshold_mv - 2600;
    // cfg.absolute_VINDPM_threshold_mv = cfg.absolute_VINDPM_threshold_mv / 100;
    // ret = bq_25896_modify_reg(0x0d, (uint8_t)cfg.absolute_VINDPM_threshold_mv, 0x7f, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0xd);

    return BQ_OK;
}

bq25896_error_t bq25896_set_charge_config(bq25896_charge_config_t cfg)
{
    /*
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
    */
    bq25896_error_t ret;
    // 3
    ret = bq_25896_modify_reg(0x03, BQ_25896_IS_ONE_BINARY(cfg.charge_enable), 0x01, 4);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 3);
    // 4
    cfg.fast_charge_current_limit_ma = cfg.fast_charge_current_limit_ma / 64;
    ret = bq_25896_modify_reg(0x04, cfg.fast_charge_current_limit_ma, 0x7f, 0);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x04);
    // 5
    cfg.pre_charge_current_limit_ma = cfg.pre_charge_current_limit_ma - 64;
    cfg.pre_charge_current_limit_ma = cfg.pre_charge_current_limit_ma / 64;
    cfg.termination_current_limit_ma = cfg.termination_current_limit_ma - 64;
    cfg.termination_current_limit_ma = cfg.termination_current_limit_ma / 64;
    bq_reg_desc_t xfer_desc_for_reg0x05[2] = {
        {
            .mask = 0x0f,
            .lsh = 4,
            .dat = cfg.pre_charge_current_limit_ma,
        },
        {
            .mask = 0x0f,
            .lsh = 0,
            .dat = cfg.termination_current_limit_ma,
        },
    };
    ret = bq_25896_modify_muti_regs(0x05, xfer_desc_for_reg0x05, 2);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x05);
    // 6
    cfg.charge_voltage_limit_mv = cfg.charge_voltage_limit_mv - 3840;
    cfg.charge_voltage_limit_mv = cfg.charge_voltage_limit_mv / 16;
    bq_reg_desc_t xfer_desc_for_reg0x06[3] = {
        {
            .mask = 0x3f,
            .lsh = 2,
            .dat = cfg.charge_voltage_limit_mv,
        },
        {
            .mask = 0x01,
            .lsh = 1,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.bat_pre_charge_to_fast_charge_threshold),
        },
        {
            .mask = 0x01,
            .lsh = 0,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.bat_re_charging_threshold_offset),
        },
    };
    ret = bq_25896_modify_muti_regs(0x06, xfer_desc_for_reg0x06, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x06);
    // 7
    bq_reg_desc_t xfer_desc_for_reg0x07[3] = {
        {
            .mask = 0x01,
            .lsh = 7,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.charge_termination_enable),
        },
        {
            .mask = 0x01,
            .lsh = 3,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.charging_safety_timer_enable),
        },
        {
            .mask = 0x03,
            .lsh = 1,
            .dat = cfg.fast_charge_timer_setting,
        },
    };
    ret = bq_25896_modify_muti_regs(0x07, xfer_desc_for_reg0x07, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x07);

    // // 3
    // ret = bq_25896_modify_reg(0x03, BQ_25896_IS_ONE_BINARY(cfg.charge_enable), 0x01, 4);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 3);
    // // 4
    // cfg.fast_charge_current_limit_ma = cfg.fast_charge_current_limit_ma / 64;
    // ret = bq_25896_modify_reg(0x04, cfg.fast_charge_current_limit_ma, 0x7f, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x04);
    // // 5
    // cfg.pre_charge_current_limit_ma = cfg.pre_charge_current_limit_ma - 64;
    // cfg.pre_charge_current_limit_ma = cfg.pre_charge_current_limit_ma / 64;
    // ret = bq_25896_modify_reg(0x05, (uint8_t)cfg.pre_charge_current_limit_ma, 0x0f, 4);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 5);
    // cfg.termination_current_limit_ma = cfg.termination_current_limit_ma - 64;
    // cfg.termination_current_limit_ma = cfg.termination_current_limit_ma / 64;
    // ret = bq_25896_modify_reg(0x05, (uint8_t)cfg.termination_current_limit_ma, 0x0f, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 5);
    // // 6
    // cfg.charge_voltage_limit_mv = cfg.charge_voltage_limit_mv - 3840;
    // cfg.charge_voltage_limit_mv = cfg.charge_voltage_limit_mv / 16;
    // ret = bq_25896_modify_reg(0x06, (uint8_t)cfg.charge_voltage_limit_mv, 0x3f, 2);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 6);
    // ret = bq_25896_modify_reg(0x06, BQ_25896_IS_ONE_BINARY(cfg.bat_pre_charge_to_fast_charge_threshold), 0x01, 1);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 6);
    // ret = bq_25896_modify_reg(0x06, BQ_25896_IS_ONE_BINARY(cfg.bat_re_charging_threshold_offset), 0x01, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 6);
    // // 7
    // ret = bq_25896_modify_reg(0x07, BQ_25896_IS_ONE_BINARY(cfg.charge_termination_enable), 0x01, 7);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x07);
    // ret = bq_25896_modify_reg(0x07, BQ_25896_IS_ONE_BINARY(cfg.charging_safety_timer_enable), 0x01, 3);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x07);
    // ret = bq_25896_modify_reg(0x07, cfg.fast_charge_timer_setting, 0x03, 1);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x07);
    return BQ_OK;
}

bq25896_error_t bq25896_set_boost_config(bq25896_boost_config_t cfg)
{
    /*
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
    */
    bq25896_error_t ret;
    // 1
    bq_reg_desc_t xfer_desc_for_reg0x01[2] = {
        {
            .mask = 0x03,
            .lsh = 6,
            .dat = cfg.bst_mode_thermal_prot_hot,
        },
        {
            .mask = 0x01,
            .lsh = 5,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.bst_mode_thermal_prot_cold),
        },
    };
    ret = bq_25896_modify_muti_regs(0x01, xfer_desc_for_reg0x01, 2);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x01);
    // 2
    ret = bq_25896_modify_reg(0x02, BQ_25896_IS_ONE_BINARY(cfg.bst_mode_freq), 0x01, 5);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x02);
    // 3
    ret = bq_25896_modify_reg(0x03, BQ_25896_IS_ONE_BINARY(cfg.boost_mode_enable), 0x01, 5);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x03);
    // a
    cfg.boost_mode_voltage_mv = cfg.boost_mode_voltage_mv - 4550;
    cfg.boost_mode_voltage_mv = cfg.boost_mode_voltage_mv / 64;
    bq_reg_desc_t xfer_desc_for_reg0x0a[3] = {
        {
            .mask = 0x0f,
            .lsh = 4,
            .dat = cfg.boost_mode_voltage_mv,
        },
        {
            .mask = 0x01,
            .lsh = 3,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.PFM_mode_allow_in_boost_mode),
        },
        {
            .mask = 0x07,
            .lsh = 0,
            .dat = cfg.boost_mode_current_limit,
        },
    };
    ret = bq_25896_modify_muti_regs(0x0a, xfer_desc_for_reg0x0a, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0a);

    // // 1
    // ret = bq_25896_modify_reg(0x01, cfg.bst_mode_thermal_prot_hot, 0x03, 6);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x01);
    // ret = bq_25896_modify_reg(0x01, BQ_25896_IS_ONE_BINARY(cfg.bst_mode_thermal_prot_cold), 0x01, 5);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x01);
    // // 2
    // ret = bq_25896_modify_reg(0x02, BQ_25896_IS_ONE_BINARY(cfg.bst_mode_freq), 0x01, 5);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x02);
    // // 3
    // ret = bq_25896_modify_reg(0x03, BQ_25896_IS_ONE_BINARY(cfg.boost_mode_enable), 0x01, 5);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x03);
    // // a
    // cfg.boost_mode_voltage_mv = cfg.boost_mode_voltage_mv - 4550;
    // cfg.boost_mode_voltage_mv = cfg.boost_mode_voltage_mv / 64;
    // ret = bq_25896_modify_reg(0x0a, cfg.boost_mode_voltage_mv, 0x0f, 4);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0a);
    // ret = bq_25896_modify_reg(0x0a, BQ_25896_IS_ONE_BINARY(cfg.PFM_mode_allow_in_boost_mode), 0x01, 3);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0a);
    // ret = bq_25896_modify_reg(0x0a, cfg.boost_mode_current_limit, 0x07, 0);
    // BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0a);
    return BQ_OK;
}

bq25896_error_t bq25896_set_system_config(bq25896_system_config_t cfg)
{
    /*
    // reg 00
    uint8_t hiz_enable; // REGN LDO off
    // reg 02
    uint8_t adc_conv_enable; // ADC Conversion Start Control
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
    */
    bq25896_error_t ret;
    // 0
    ret = bq_25896_modify_reg(0x0, BQ_25896_IS_ONE_BINARY(cfg.hiz_enable), 0x01, 7);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x0);
    // 2
    bq_reg_desc_t xfer_desc_for_reg0x02[2] = {
        {
            .mask = 0x01,
            .lsh = 7,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.adc_conv_enable),
        },
        {
            .mask = 0x01,
            .lsh = 6,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.adc_conv_rate),
        },
    };
    ret = bq_25896_modify_muti_regs(0x02, xfer_desc_for_reg0x02, 2);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x02);
    // 3
    cfg.min_system_voltage_mv -= 3000;
    cfg.min_system_voltage_mv /= 100;
    bq_reg_desc_t xfer_desc_for_reg0x03[3] = {
        {
            .mask = 0x01,
            .lsh = 7,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.batt_load_enable),
        },
        {
            .mask = 0x07,
            .lsh = 1,
            .dat = cfg.min_system_voltage_mv,
        },
        {
            .mask = 0x01,
            .lsh = 0,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.min_batt_voltage_to_exit_bst_mode),
        },
    };
    ret = bq_25896_modify_muti_regs(0x03, xfer_desc_for_reg0x03, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x03);
    // 4
    ret = bq_25896_modify_reg(0x04, BQ_25896_IS_ONE_BINARY(cfg.current_pluse_enable), 0x01, 7);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x04);
    // 7
    bq_reg_desc_t xfer_desc_for_reg0x07[3] = {
        {
            .mask = 0x01,
            .lsh = 6,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.stat_pin_disable),
        },
        {
            .mask = 0x03,
            .lsh = 4,
            .dat = cfg.i2c_wtd_config,
        },
        {
            .mask = 0x01,
            .lsh = 0,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.JEITA_low_temp_current_setting),
        },
    };
    ret = bq_25896_modify_muti_regs(0x07, xfer_desc_for_reg0x07, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x07);
    // 8
    cfg.IR_compensation_resistor_setting_mr /= 20;
    cfg.IR_compensation_voltage_clamp_mv /= 32;
    bq_reg_desc_t xfer_desc_for_reg0x08[3] = {
        {
            .mask = 0x07,
            .lsh = 5,
            .dat = cfg.IR_compensation_resistor_setting_mr,
        },
        {
            .mask = 0x07,
            .lsh = 2,
            .dat = cfg.IR_compensation_voltage_clamp_mv,
        },
        {
            .mask = 0x03,
            .lsh = 0,
            .dat = cfg.thermal_regulation_threshold,
        },
    };
    ret = bq_25896_modify_muti_regs(0x08, xfer_desc_for_reg0x08, 3);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x08);
    // 9
    bq_reg_desc_t xfer_desc_for_reg0x09[4] = {
        {
            .mask = 0x01,
            .lsh = 6,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.safety_timer_slow_during_DPM_or_thermal_regulation_enable),
        },
        {
            .mask = 0x01,
            .lsh = 4,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.JEITA_high_temp_voltage_setting),
        },
        {
            .mask = 0x01,
            .lsh = 3,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.BATFET_turn_off_delay_enable),
        },
        {
            .mask = 0x01,
            .lsh = 2,
            .dat = BQ_25896_IS_ONE_BINARY(cfg.BATFET_full_system_reset_enable),
        },
    };
    ret = bq_25896_modify_muti_regs(0x09, xfer_desc_for_reg0x09, 4);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "write reg %d err", 0x09);
    return BQ_OK;
}

/*============================================================get api============================================================*/

bq25896_error_t bq25896_get_input_config(bq25896_input_config_t *cfg)
{
    /*
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
    uint16_t absolute_VINDPM_threshold_mv;   // Absolute VINDPM Threshold Note: Register is reset to default value when input source is plugged-in(3900 to 15300),100mV step,Offset: 2.6V
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 0
    ret = bq25896_read_reg(0x0, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0);
    cfg->ilim_pin_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    cfg->input_current_lim_ma = (tmp & 0x3f);
    cfg->input_current_lim_ma = cfg->input_current_lim_ma * 50;
    cfg->input_current_lim_ma = cfg->input_current_lim_ma + 100;
    // 1
    ret = bq25896_read_reg(0x1, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 1);
    cfg->input_voltage_lim_offset_mv = (tmp & 0x1f);
    cfg->input_voltage_lim_offset_mv = cfg->input_voltage_lim_offset_mv * 100;
    // 2
    ret = bq25896_read_reg(0x2, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 2);
    cfg->input_current_optimizer_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x10);
    cfg->force_input_detection_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x02);
    cfg->auto_input_detection_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x01);
    // 9
    ret = bq25896_read_reg(0x9, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 9);
    cfg->force_start_input_current_optimizer = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    // d
    ret = bq25896_read_reg(0xd, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0xd);
    elog_i(TAG, "bq_get in reg d = 0x%x", tmp);
    cfg->VINDPM_threshold_setting_method = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    cfg->absolute_VINDPM_threshold_mv = (tmp & 0x7f);
    cfg->absolute_VINDPM_threshold_mv = cfg->absolute_VINDPM_threshold_mv * 100;
    cfg->absolute_VINDPM_threshold_mv = cfg->absolute_VINDPM_threshold_mv + 2600;

    return BQ_OK;
}

bq25896_error_t bq25896_get_charge_config(bq25896_charge_config_t *cfg)
{
    /*
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
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 3
    ret = bq25896_read_reg(0x3, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 3);
    cfg->charge_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x10);
    // 4
    ret = bq25896_read_reg(0x4, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 4);
    cfg->fast_charge_current_limit_ma = tmp & 0x7f;
    cfg->fast_charge_current_limit_ma = cfg->fast_charge_current_limit_ma * 64;
    // 5
    ret = bq25896_read_reg(0x5, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 5);
    cfg->pre_charge_current_limit_ma = (tmp & 0xf0) >> 4;
    cfg->pre_charge_current_limit_ma = cfg->pre_charge_current_limit_ma * 64;
    cfg->pre_charge_current_limit_ma = cfg->pre_charge_current_limit_ma + 64;
    cfg->termination_current_limit_ma = (tmp & 0x0f);
    cfg->termination_current_limit_ma = cfg->termination_current_limit_ma * 64;
    cfg->termination_current_limit_ma = cfg->termination_current_limit_ma + 64;
    // 6
    ret = bq25896_read_reg(0x6, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 6);
    cfg->charge_voltage_limit_mv = (tmp & 0xfc) >> 2;
    cfg->charge_voltage_limit_mv = cfg->charge_voltage_limit_mv * 16;
    cfg->charge_voltage_limit_mv = cfg->charge_voltage_limit_mv + 3840;
    cfg->bat_pre_charge_to_fast_charge_threshold = BQ_25896_IS_ONE_BINARY(tmp & 0x2);
    cfg->bat_re_charging_threshold_offset = BQ_25896_IS_ONE_BINARY(tmp & 0x1);
    // 7
    ret = bq25896_read_reg(0x7, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 7);
    cfg->charge_termination_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    cfg->charging_safety_timer_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x08);
    cfg->fast_charge_timer_setting = (tmp & 0x6) >> 1;
    return BQ_OK;
}

bq25896_error_t bq25896_get_boost_config(bq25896_boost_config_t *cfg)
{
    /*
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
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 1
    ret = bq25896_read_reg(0x1, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 1);
    cfg->bst_mode_thermal_prot_hot = (tmp & 0xc0) >> 6;
    cfg->bst_mode_thermal_prot_cold = BQ_25896_IS_ONE_BINARY(tmp & 0x20);
    // 2
    ret = bq25896_read_reg(0x2, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 2);
    cfg->bst_mode_freq = BQ_25896_IS_ONE_BINARY(tmp & 0x20);
    // 3
    ret = bq25896_read_reg(0x3, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 3);
    cfg->boost_mode_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x20);
    // a
    ret = bq25896_read_reg(0xa, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0xa);
    cfg->boost_mode_voltage_mv = (tmp & 0xf0) >> 4;
    cfg->boost_mode_voltage_mv = cfg->boost_mode_voltage_mv * 64;
    cfg->boost_mode_voltage_mv = cfg->boost_mode_voltage_mv + 4550;
    cfg->PFM_mode_allow_in_boost_mode = BQ_25896_IS_ONE_BINARY(tmp & 0x08);
    cfg->boost_mode_current_limit = (bq25896_boost_current_limit_t)(tmp & 0x07);

    return BQ_OK;
}

bq25896_error_t bq25896_get_system_config(bq25896_system_config_t *cfg)
{
    /*
    // reg 00
    uint8_t hiz_enable; // REGN LDO off
    // reg 02
    uint8_t adc_conv_enable; // ADC Conversion Start Control
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
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 0
    ret = bq25896_read_reg(0x00, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x00);
    cfg->hiz_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    // 2
    ret = bq25896_read_reg(0x02, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x02);
    cfg->adc_conv_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    cfg->adc_conv_rate = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    // 3
    ret = bq25896_read_reg(0x03, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x03);
    cfg->batt_load_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    cfg->min_system_voltage_mv = (((tmp & 0x0e) >> 1) * 100) + 3000;
    cfg->min_batt_voltage_to_exit_bst_mode = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    // 4
    ret = bq25896_read_reg(0x04, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x04);
    cfg->current_pluse_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    // 7
    ret = bq25896_read_reg(0x07, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x07);
    cfg->stat_pin_disable = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    cfg->i2c_wtd_config = (tmp & 0x30) >> 4;
    cfg->JEITA_low_temp_current_setting = BQ_25896_IS_ONE_BINARY(tmp & 0x01);
    // 8
    ret = bq25896_read_reg(0x08, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x08);
    cfg->IR_compensation_resistor_setting_mr = ((tmp & 0xe0) >> 5) * 20;
    cfg->IR_compensation_voltage_clamp_mv = ((tmp & 0x1c) >> 2) * 32;
    cfg->thermal_regulation_threshold = (tmp & 0x03);
    // 9
    ret = bq25896_read_reg(0x09, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x09);
    cfg->safety_timer_slow_during_DPM_or_thermal_regulation_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    cfg->JEITA_high_temp_voltage_setting = BQ_25896_IS_ONE_BINARY(tmp & 0x10);
    cfg->BATFET_turn_off_delay_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x08);
    cfg->BATFET_full_system_reset_enable = BQ_25896_IS_ONE_BINARY(tmp & 0x04);
    return BQ_OK;
}

bq25896_error_t bq25896_get_system_status(bq25896_system_status_t *dst)
{
    /*
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
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 0x0b
    ret = bq25896_read_reg(0x0b, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x0b);
    dst->VBUS_status = (tmp & 0xe0) >> 5;
    dst->charge_status = (tmp & 0x18) >> 3;
    dst->pg = BQ_25896_IS_ONE_BINARY(tmp & 0x04);
    dst->VSYS_regulation = BQ_25896_IS_ONE_BINARY(tmp & 0x01);
    // 0x0c
    ret = bq25896_read_reg(0x0c, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x0c);
    dst->wtd_fault = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    dst->bst_overload = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    dst->charge_fault_status = (tmp & 0x30) >> 4;
    dst->batt_ovp = BQ_25896_IS_ONE_BINARY(tmp & 0x08);
    dst->NTC_fault_status = (tmp & 0x07);
    // 0x0e
    ret = bq25896_read_reg(0x0e, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x0e);
    dst->thermal_regulation = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    // 0x11
    ret = bq25896_read_reg(0x11, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x11);
    dst->VBUS_good = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    // 0x13
    ret = bq25896_read_reg(0x13, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x13);
    dst->VNDPM = BQ_25896_IS_ONE_BINARY(tmp & 0x80);
    dst->INDPM = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    // 0x14
    ret = bq25896_read_reg(0x14, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x14);
    dst->ico_done = BQ_25896_IS_ONE_BINARY(tmp & 0x40);
    return BQ_OK;
}

bq25896_error_t bq25896_get_adc(bq25896_adc_resault_t *dst)
{
    /*
    // reg 0e ro
    uint16_t ADC_BAT_voltage_mv; // ADC conversion of Battery Voltage (VBAT)
    // reg 0f ro
    uint16_t ADC_SYS_voltage_mv; // ADDC conversion of System Voltage (VSYS)
    // reg 10 ro
    uint16_t ADC_TS_voltage_mv; // ADC conversion of TS Voltage (TS) as percentage of REGN
    // reg 11 ro
    uint16_t ADC_VBUS_voltage_mv; // ADC conversion of VBUS voltage (VBUS)
    // reg 12 ro
    uint16_t ADC_charge_current_ma; // ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT
    // reg 13 ro
    uint16_t input_current_limit_with_ico_ma; // Input Current Limit in effect while Input Current Optimizer (ICO) is enabled
    */
    uint8_t tmp;
    bq25896_error_t ret;
    // 0x0e
    ret = bq25896_read_reg(0x0e, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x0e);
    dst->ADC_BAT_voltage_mv = ((tmp & 0x7f) * 20) + 2304;
    // 0x0f
    ret = bq25896_read_reg(0x0f, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x0f);
    dst->ADC_SYS_voltage_mv = ((tmp & 0x7f) * 20) + 2304;
    // 0x10
    ret = bq25896_read_reg(0x10, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x10);
    dst->ADC_TS_voltage_pst = ((uint32_t)(tmp & 0x7f) * 465UL) + 21000UL;
    // 0x11
    ret = bq25896_read_reg(0x11, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x11);
    dst->ADC_VBUS_voltage_mv = ((tmp & 0x7f) * 100) + 2600;
    // 0x12
    ret = bq25896_read_reg(0x12, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x12);
    dst->ADC_charge_current_ma = ((tmp & 0x7f) * 50);
    // 0x13
    ret = bq25896_read_reg(0x13, &tmp, 1);
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "read reg %d err", 0x13);
    dst->input_current_limit_with_ico_ma = ((tmp & 0x7f) * 50) + 100;
    return BQ_OK;
}

/*============================================================get all============================================================*/

bq25896_error_t bq25896_get_all_reg(bq25896_all_reg_t *reg)
{
    uint8_t regs[sizeof(bq25896_all_reg_t)];
    bq25896_error_t ret = bq25896_read_reg(0x0, regs, sizeof(bq25896_all_reg_t));
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 read all err %d", ret);

    elog_hexdump(TAG, 8, regs, sizeof(bq25896_all_reg_t));
    // reverse_bits(regs, sizeof(bq25896_all_reg_t));
    // elog_i(TAG,"rev");
    // elog_hexdump(TAG, 8, regs, sizeof(bq25896_all_reg_t));
    memcpy(reg, regs, sizeof(bq25896_all_reg_t));
    return BQ_OK;
}

/*============================================================imm api============================================================*/
bq25896_error_t bq25896_poweroff()
{
    bq25896_error_t ret = bq_25896_modify_reg(0x09, 0x01, 0x01, 5); // Force BATFET off
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 poweroff err %d", ret);
    return BQ_OK;
}

bq25896_error_t bq25896_wtd_feed()
{
    bq25896_error_t ret = bq_25896_modify_reg(0x03, 0x01, 0x01, 6); // Reset (Back to 0 after timer reset)
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 wtd feed err %d", ret);
    return bq_25896_auto_poll(0x03, 0x40, 0x00, 0xffffffff);
}

bq25896_error_t bq25896_current_pluse(bq25896_current_pluse_direction_t dir)
{
    // This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
    bq25896_error_t ret = BQ_OK;
    switch (dir)
    {
    case BQ_CP_UP:
        ret = bq_25896_modify_reg(0x09, 0x01, 0x01, 1); // Current pulse control voltage up enable
        BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 current pluse err %d", ret);
        return bq_25896_auto_poll(0x09, 0x2, 0x00, 0xffffffff); // This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed

        break;
    case BQ_CP_DOWN:
        ret = bq_25896_modify_reg(0x09, 0x01, 0x01, 0);
        ; // Current pulse control voltage down enable

        BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 current pluse err %d", ret);
        return bq_25896_auto_poll(0x09, 0x1, 0x00, 0xffffffff); // This bit is can only be set when EN_PUMPX bit is set and returns to 0 after current pulse control sequence is completed
        break;
    default:
        elog_w(TAG, "bq current pluse dir error %d", dir);
        return BQ_ERR_CODE;
        break;
    }
    elog_w(TAG, "???????");
    return BQ_OK;
}

bq25896_error_t bq25896_reg_reset()
{
    bq25896_error_t ret = bq_25896_modify_reg(0x14, 0x1, 0x01, 7); // Reset to default register value and reset safety timer
    BQ_25896_RETURN_ON_FALSE((ret == BQ_OK), BQ_ERR_TRANS, TAG, "bq25896 wtd feed err %d", ret);
    return bq_25896_auto_poll(0x14, 0x80, 0x00, 0xffffffff); // Reset to 0 after register reset is completed
}

/*============================================================debug api============================================================*/
void bq25896_print_all_regs(bq25896_all_reg_t *regs)
{

    // Register 00
    elog_i(TAG, "====================");
    elog_i(TAG, "hiz_enable:%X ", regs->hiz_enable);
    elog_i(TAG, "ilim_pin_enable:%X ", regs->ilim_pin_enable);
    elog_i(TAG, "input_current_lim:%X", regs->input_current_lim);

    // Register 01
    elog_i(TAG, "====================");
    elog_i(TAG, "bst_mode_thermal_prot_hot:%X ", regs->bst_mode_thermal_prot_hot);
    elog_i(TAG, "bst_mode_thermal_prot_cold:%X ", regs->bst_mode_thermal_prot_cold);
    elog_i(TAG, "input_volt_lim_offset:%X", regs->input_volt_lim_offset);

    // Register 02
    elog_i(TAG, "====================");
    elog_i(TAG, "adc_conv_start:%X ", regs->adc_conv_start);
    elog_i(TAG, "adc_conv_rate:%X ", regs->adc_conv_rate);
    elog_i(TAG, "bst_mode_freq:%X ", regs->bst_mode_freq);
    elog_i(TAG, "input_current_optimizer:%X ", regs->input_current_optimizer);
    elog_i(TAG, "force_input_detection:%X ", regs->force_input_detection);
    elog_i(TAG, "auto_input_detection_enable:%X", regs->auto_input_detection_enable);

    // Register 03
    elog_i(TAG, "====================");
    elog_i(TAG, "batt_load_enable:%X ", regs->batt_load_enable);
    elog_i(TAG, "i2c_wtd_feed:%X ", regs->i2c_wtd_feed);
    elog_i(TAG, "boost_mode_enable:%X ", regs->boost_mode_enable);
    elog_i(TAG, "charge_enable:%X ", regs->charge_enable);
    elog_i(TAG, "min_system_voltage:%X ", regs->min_system_voltage);
    elog_i(TAG, "min_batt_voltage_to_exit_bst_mode:%X", regs->min_batt_voltage_to_exit_bst_mode);

    // Register 04
    elog_i(TAG, "====================");
    elog_i(TAG, "current_pluse_enable:%X ", regs->current_pluse_enable);
    elog_i(TAG, "fast_charge_current_limit:%X", regs->fast_charge_current_limit);

    // Register 05
    elog_i(TAG, "====================");
    elog_i(TAG, "pre_charge_current_limit:%X ", regs->pre_charge_current_limit);
    elog_i(TAG, "termination_current_limit:%X", regs->termination_current_limit);

    // Register 06
    elog_i(TAG, "====================");
    elog_i(TAG, "charge_voltage_limit:%X ", regs->charge_voltage_limit);
    elog_i(TAG, "bat_pre_charge_to_fast_charge_threshold:%X ", regs->bat_pre_charge_to_fast_charge_threshold);
    elog_i(TAG, "bat_re_charging_threshold_offset:%X", regs->bat_re_charging_threshold_offset);

    // Register 07
    elog_i(TAG, "====================");
    elog_i(TAG, "charge_termination_enable:%X ", regs->charge_termination_enable);
    elog_i(TAG, "stat_pin_disable:%X ", regs->stat_pin_disable);
    elog_i(TAG, "i2c_wtd_config:%X ", regs->i2c_wtd_config);
    elog_i(TAG, "charging_safety_timer_enable:%X ", regs->charging_safety_timer_enable);
    elog_i(TAG, "fast_charge_timer_setting:%X ", regs->fast_charge_timer_setting);
    elog_i(TAG, "JEITA_low_temp_current_setting:%X", regs->JEITA_low_temp_current_setting);

    // Register 08
    elog_i(TAG, "====================");
    elog_i(TAG, "IR_compensation_resistor_setting:%X ", regs->IR_compensation_resistor_setting);
    elog_i(TAG, "IR_compensation_voltage_clamp:%X ", regs->IR_compensation_voltage_clamp);
    elog_i(TAG, "thermal_regulation_threshold:%X", regs->thermal_regulation_threshold);

    // Register 09
    elog_i(TAG, "====================");
    elog_i(TAG, "force_start_input_current_optimizer:%X ", regs->force_start_input_current_optimizer);
    elog_i(TAG, "safety_timer_setting_during_DPM_or_thermal_regulation:%X ", regs->safety_timer_setting_during_DPM_or_thermal_regulation);
    elog_i(TAG, "force_BATFET_off:%X ", regs->force_BATFET_off);
    elog_i(TAG, "JEITA_high_temp_voltage_setting:%X ", regs->JEITA_high_temp_voltage_setting);
    elog_i(TAG, "BATFET_turn_off_delay:%X ", regs->BATFET_turn_off_delay);
    elog_i(TAG, "BATFET_full_system_reset_enable:%X ", regs->BATFET_full_system_reset_enable);
    elog_i(TAG, "current_pluse_control_voltage_up_enable:%X ", regs->current_pluse_control_voltage_up_enable);
    elog_i(TAG, "current_pluse_control_voltage_down_enable:%X", regs->current_pluse_control_voltage_down_enable);

    // Register 0A
    elog_i(TAG, "====================");
    elog_i(TAG, "boost_mode_voltage_regulation:%X ", regs->boost_mode_voltage_regulation);
    elog_i(TAG, "PFM_mode_allow_in_boost_mode:%X ", regs->PFM_mode_allow_in_boost_mode);
    elog_i(TAG, "boost_mode_current_limit:%X", regs->boost_mode_current_limit);

    // Register 0B
    elog_i(TAG, "====================");
    elog_i(TAG, "VBUS_status:%X ", regs->VBUS_status);
    elog_i(TAG, "charge_status:%X ", regs->charge_status);
    elog_i(TAG, "pg:%X ", regs->pg);
    elog_i(TAG, "VSYS_regulation_status:%X", regs->VSYS_regulation_status);

    // Register 0C
    elog_i(TAG, "====================");
    elog_i(TAG, "wtd_fault_status:%X ", regs->wtd_fault_status);
    elog_i(TAG, "bst_mode_fault_status:%X ", regs->bst_mode_fault_status);
    elog_i(TAG, "charge_fault_status:%X ", regs->charge_fault_status);
    elog_i(TAG, "batt_fault_status:%X ", regs->batt_fault_status);
    elog_i(TAG, "NTC_fault_status:%X", regs->NTC_fault_status);

    // Register 0D
    elog_i(TAG, "====================");
    elog_i(TAG, "VINDPM_threshold_setting_method:%X ", regs->VINDPM_threshold_setting_method);
    elog_i(TAG, "absolute_VINDPM_threshold:%X", regs->absolute_VINDPM_threshold);

    // Register 0E
    elog_i(TAG, "====================");
    elog_i(TAG, "thermal_regulation_status:%X ", regs->thermal_regulation_status);
    elog_i(TAG, "ADC_BAT_voltage:%X", regs->ADC_BAT_voltage);

    // Register 0F
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_SYS_voltage:%X", regs->ADC_SYS_voltage);

    // Register 10
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_TS_voltage:%X", regs->ADC_TS_voltage);

    // Register 11
    elog_i(TAG, "====================");
    elog_i(TAG, "VBUS_good:%X ", regs->VBUS_good);
    elog_i(TAG, "ADC_VBUS_voltage:%X", regs->ADC_VBUS_voltage);

    // Register 12
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_charge_current:%X", regs->ADC_charge_current);

    // Register 13
    elog_i(TAG, "====================");
    elog_i(TAG, "VNDPM_status:%X ", regs->VNDPM_status);
    elog_i(TAG, "INDPM_status:%X ", regs->INDPM_status);
    elog_i(TAG, "input_current_limit_with_ico:%X", regs->input_current_limit_with_ico);

    // Register 14
    elog_i(TAG, "====================");
    elog_i(TAG, "reg_reset:%X ", regs->reg_reset);
    elog_i(TAG, "ico_status:%X ", regs->ico_status);
    elog_i(TAG, "id:%X ", regs->id);
    elog_i(TAG, "temperature_profile:%X ", regs->temperature_profile);
    elog_i(TAG, "dev_rev:%X", regs->dev_rev);

} // bq25896_print_all_regs

void bq25896_print_input_config(bq25896_input_config_t *regs)
{
    elog_i(TAG, "====================");
    elog_i(TAG, "ilim_pin_enable:%d ", regs->ilim_pin_enable);
    elog_i(TAG, "input_current_lim_ma:%d ", regs->input_current_lim_ma);
    elog_i(TAG, "====================");
    elog_i(TAG, "input_voltage_lim_offset_mv:%d ", regs->input_voltage_lim_offset_mv);
    elog_i(TAG, "====================");
    elog_i(TAG, "input_current_optimizer_enable:%d ", regs->input_current_optimizer_enable);
    elog_i(TAG, "force_input_detection_enable:%d ", regs->force_input_detection_enable);
    elog_i(TAG, "auto_input_detection_enable:%d ", regs->auto_input_detection_enable);
    elog_i(TAG, "====================");
    elog_i(TAG, "force_start_input_current_optimizer:%d ", regs->force_start_input_current_optimizer);
    elog_i(TAG, "====================");
    elog_i(TAG, "VINDPM_threshold_setting_method:%d ", regs->VINDPM_threshold_setting_method);
    elog_i(TAG, "absolute_VINDPM_threshold_mv:%d ", regs->absolute_VINDPM_threshold_mv);
} // bq25896_print_input_config

void bq25896_print_charge_config(bq25896_charge_config_t *regs)
{
    elog_i(TAG, "====================");
    elog_i(TAG, "charge_enable:%d ", regs->charge_enable);
    elog_i(TAG, "====================");
    elog_i(TAG, "fast_charge_current_limit_ma:%d ", regs->fast_charge_current_limit_ma);
    elog_i(TAG, "====================");
    elog_i(TAG, "pre_charge_current_limit_ma:%d ", regs->pre_charge_current_limit_ma);
    elog_i(TAG, "termination_current_limit_ma:%d ", regs->termination_current_limit_ma);
    elog_i(TAG, "====================");
    elog_i(TAG, "charge_voltage_limit_mv:%d ", regs->charge_voltage_limit_mv);
    elog_i(TAG, "bat_pre_charge_to_fast_charge_threshold:%d ", regs->bat_pre_charge_to_fast_charge_threshold);
    elog_i(TAG, "bat_re_charging_threshold_offset:%d ", regs->bat_re_charging_threshold_offset);
    elog_i(TAG, "====================");
    elog_i(TAG, "charge_termination_enable:%d ", regs->charge_termination_enable);
    elog_i(TAG, "charging_safety_timer_enable:%d ", regs->charging_safety_timer_enable);
    elog_i(TAG, "fast_charge_timer_setting:%d ", regs->fast_charge_timer_setting);

} // bq25896_print_input_config

void bq25896_print_boost_config(bq25896_boost_config_t *regs)
{
    elog_i(TAG, "====================");
    elog_i(TAG, "bst_mode_thermal_prot_hot:%d ", regs->bst_mode_thermal_prot_hot);
    elog_i(TAG, "bst_mode_thermal_prot_cold:%d ", regs->bst_mode_thermal_prot_cold);
    elog_i(TAG, "====================");
    elog_i(TAG, "bst_mode_freq:%d ", regs->bst_mode_freq);
    elog_i(TAG, "====================");
    elog_i(TAG, "boost_mode_enable:%d ", regs->boost_mode_enable);
    elog_i(TAG, "====================");
    elog_i(TAG, "boost_mode_voltage_mv:%d ", regs->boost_mode_voltage_mv);
    elog_i(TAG, "PFM_mode_allow_in_boost_mode:%d ", regs->PFM_mode_allow_in_boost_mode);
    elog_i(TAG, "boost_mode_current_limit:%d ", regs->boost_mode_current_limit);
} // bq25896_print_input_config

void bq25896_print_system_config(bq25896_system_config_t *regs)
{
    elog_i(TAG, "====================");
    elog_i(TAG, "hiz_enable:%d ", regs->hiz_enable);
    elog_i(TAG, "====================");
    elog_i(TAG, "adc_conv_enable:%d ", regs->adc_conv_enable);
    elog_i(TAG, "adc_conv_rate:%d ", regs->adc_conv_rate);
    elog_i(TAG, "====================");
    elog_i(TAG, "batt_load_enable:%d ", regs->batt_load_enable);
    elog_i(TAG, "min_system_voltage_mv:%d ", regs->min_system_voltage_mv);
    elog_i(TAG, "min_batt_voltage_to_exit_bst_mode:%d ", regs->min_batt_voltage_to_exit_bst_mode);
    elog_i(TAG, "====================");
    elog_i(TAG, "current_pluse_enable:%d ", regs->current_pluse_enable);
    elog_i(TAG, "====================");
    elog_i(TAG, "stat_pin_disable:%d ", regs->stat_pin_disable);
    elog_i(TAG, "i2c_wtd_config:%d ", regs->i2c_wtd_config);
    elog_i(TAG, "JEITA_low_temp_current_setting:%d ", regs->JEITA_low_temp_current_setting);
    elog_i(TAG, "====================");
    elog_i(TAG, "IR_compensation_resistor_setting_mr:%d ", regs->IR_compensation_resistor_setting_mr);
    elog_i(TAG, "IR_compensation_voltage_clamp_mv:%d ", regs->IR_compensation_voltage_clamp_mv);
    elog_i(TAG, "thermal_regulation_threshold:%d ", regs->thermal_regulation_threshold);
    elog_i(TAG, "====================");
    elog_i(TAG, "safety_timer_slow_during_DPM_or_thermal_regulation_enable:%d ", regs->safety_timer_slow_during_DPM_or_thermal_regulation_enable);
    elog_i(TAG, "JEITA_high_temp_voltage_setting:%d ", regs->JEITA_high_temp_voltage_setting);
    elog_i(TAG, "BATFET_turn_off_delay_enable:%d ", regs->BATFET_turn_off_delay_enable);
    elog_i(TAG, "BATFET_full_system_reset_enable:%d ", regs->BATFET_full_system_reset_enable);
} // bq25896_print_input_config

void bq25896_print_system_status(bq25896_system_status_t *regs)
{
    elog_i(TAG, "VBUS_status:%d ", regs->VBUS_status);
    elog_i(TAG, "charge_status:%d ", regs->charge_status);
    elog_i(TAG, "pg:%d ", regs->pg);
    elog_i(TAG, "VSYS_regulation:%d ", regs->VSYS_regulation);
    elog_i(TAG, "wtd_fault:%d ", regs->wtd_fault);
    elog_i(TAG, "bst_overload:%d ", regs->bst_overload);
    elog_i(TAG, "charge_fault_status:%d ", regs->charge_fault_status);
    elog_i(TAG, "batt_ovp:%d ", regs->batt_ovp);
    elog_i(TAG, "NTC_fault_status:%d ", regs->NTC_fault_status);
    elog_i(TAG, "thermal_regulation:%d ", regs->thermal_regulation);
    elog_i(TAG, "VBUS_good:%d ", regs->VBUS_good);
    elog_i(TAG, "VNDPM:%d ", regs->VNDPM);
    elog_i(TAG, "INDPM:%d ", regs->INDPM);
    elog_i(TAG, "ico_done:%d ", regs->ico_done);
} // bq25896_print_system_status

void bq25896_print_adc(bq25896_adc_resault_t *regs)
{
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_BAT_voltage_mv:%d ", regs->ADC_BAT_voltage_mv);
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_SYS_voltage_mv:%d ", regs->ADC_SYS_voltage_mv);
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_TS_voltage_pst:%ld ", regs->ADC_TS_voltage_pst);
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_VBUS_voltage_mv:%d ", regs->ADC_VBUS_voltage_mv);
    elog_i(TAG, "====================");
    elog_i(TAG, "ADC_charge_current_ma:%d ", regs->ADC_charge_current_ma);
    elog_i(TAG, "====================");
    elog_i(TAG, "input_current_limit_with_ico_ma:%d ", regs->input_current_limit_with_ico_ma);
} // bq25896_print_adc
// eof