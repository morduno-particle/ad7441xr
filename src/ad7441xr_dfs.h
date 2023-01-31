/***************************************************************************//**
 *   @file   ad7441xr_dfs.h
 *   @brief  Defines file of AD7441xR Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef _AD7441XRDFS_H_
#define _AD7441XRDFS_H_

#define AD7441XR_N_CHANNELS             4

#define AD7441XR_CH_A                   0
#define AD7441XR_CH_B                   1
#define AD7441XR_CH_C                   2
#define AD7441XR_CH_D                   3

/** The value of the sense resistor in ohms */
#define AD7441XR_RSENSE                 100
/** 16 bit ADC */
#define AD7441XR_ADC_MAX_VALUE		65535

/** Register map */

#define AD7441XR_NOP                            0x00
#define AD7441XR_CH_FUNC_SETUP(x)               (0x01 + x)
#define AD7441XR_ADC_CONFIG(x)                  (0x05 + x)
#define AD7441XR_DIN_CONFIG(x)                  (0x09 + x)
#define AD7441XR_GPO_PARALLEL                   0x0D
#define AD7441XR_GPO_CONFIG(x)                  (0x0E + x)
#define AD7441XR_OUTPUT_CONFIG(x)               (0x12 + x)
#define AD7441XR_DAC_CODE(x)                    (0x16 + x)
#define AD7441XR_DAC_CLR_CODE(x)                (0x1A + x)
#define AD7441XR_DAC_ACTIVE(x)                  (0x1E + x)
#define AD7441XR_DIN_THRESH                     0x22
#define AD7441XR_ADC_CONV_CTRL                  0x23
#define AD7441XR_DIAG_ASSIGN                    0x24
#define AD7441XR_DIN_COMP_OUT                   0x25
#define AD7441XR_ADC_RESULT(x)                  (0x26 + x)
#define AD7441XR_DIAG_RESULT(x)                 (0x2A + x)
#define AD7441XR_ALERT_STATUS                   0x2E
#define AD7441XR_LIVE_STATUS                    0x2F
#define AD7441XR_ALERT_MASK                     0x3C
#define AD7441XR_DIN_COUNTER(x)                 (0x3D + x)
#define AD7441XR_READ_SELECT                    0x41
#define AD7441XR_THERM_RST                      0x43
#define AD7441XR_CMD_KEY                        0x44
#define AD7441XR_SCRATCH                        0x45
#define AD7441XR_SILICON_REV                    0x46
#define AD7441XR_ALERT_STATUS_RESET             NO_OS_GENMASK(15, 0)

/** Software reset sequence */
#define AD7441XR_CMD_KEY_RESET_1                0x15FA
#define AD7441XR_CMD_KEY_RESET_2                0xAF51

/** Command to load code into the DAC from DAC_CODEx */
#define AD7441XR_CMD_KEY_LDAC	                0x953A

/** Command to load code into the DAC from DAC_CLR_CODEx */
#define AD7441XR_CMD_KEY_DAC_CLEAR		0x73D1

#define AD7441XR_SPI_RD_RET_INFO_MASK		NO_OS_BIT(8)
#define AD7441XR_ERR_CLR_MASK			NO_OS_GENMASK(15, 0)
#define AD7441XR_SPI_CRC_ERR_MASK		NO_OS_BIT(13)
#define AD7441XR_CH_FUNC_SETUP_MASK             NO_OS_GENMASK(3, 0)
#define AD7441XR_ADC_RANGE_MASK                 NO_OS_GENMASK(7, 5)
#define AD7441XR_ADC_REJECTION_MASK             NO_OS_GENMASK(4, 3)
#define AD7441XR_DEBOUNCE_TIME_MASK             NO_OS_GENMASK(4, 0)
#define AD7441XR_DEBOUNCE_MODE_MASK             NO_OS_BIT(5)
#define AD7441XR_DIAG_RESULT_MASK		NO_OS_GENMASK(15, 0)
#define AD7441XR_REV_ID				NO_OS_GENMASK(7, 0)
#define AD7441XR_CH_200K_TO_GND_MASK		NO_OS_BIT(2)

/** GPO_PARALLEL register */
#define AD7441XR_GPO_PAR_DATA_MASK(x)		NO_OS_BIT(x)

/** GPO_CONFIGx register */
#define AD7441XR_GPO_SELECT_MASK		NO_OS_GENMASK(2, 0)
#define AD7441XR_GPO_DATA_MASK			NO_OS_BIT(3)

/** OUTPUT_CONFIGx register */
#define AD7441XR_SLEW_EN_MASK			NO_OS_GENMASK(7, 6)
#define AD7441XR_SLEW_LIN_STEP_MASK		NO_OS_GENMASK(5, 4)
#define AD7441XR_SLEW_LIN_RATE_MASK		NO_OS_GENMASK(3, 2)
#define AD7441XR_CLR_EN_MASK			NO_OS_BIT(1)
#define AD7441XR_I_LIMIT_MASK			NO_OS_BIT(0)

/** DAC_CODEx register */
#define AD7441XR_DAC_CODE_MASK			NO_OS_GENMASK(12, 0)

/** DAC_CLR_CODEx register */
#define AD7441XR_CLR_CODE_MASK			NO_OS_GENMASK(12, 0)

/** DAC_ACTIVEx register */
#define AD7441XR_DAC_ACTIVE_CODE_MASK		NO_OS_GENMASK(12, 0)

/** DIN_THRESH */
#define AD7441XR_COMP_THRESH_MASK		NO_OS_GENMASK(5, 1)
#define AD7441XR_DIN_THRESH_MODE_MASK		NO_OS_BIT(0)

/** ADC_CONV_CTRL register */
#define AD7441XR_CONV_SEQ_MASK                  NO_OS_GENMASK(9, 8)
#define AD7441XR_DIAG_EN_MASK(x)		(NO_OS_BIT(x) << 4)
#define AD7441XR_CH_EN_MASK(x)                  NO_OS_BIT(x)

/** DIAG_ASSIGN register */
#define AD7441XR_DIAG_ASSIGN_MASK(x)		(NO_OS_GENMASK(3, 0) << (x))

/** The maximum voltage output of the DAC is 11V */
#define AD7441XR_DAC_RANGE			11000
/** 13 bit DAC */
#define AD7441XR_DAC_RESOLUTION			13

/** The number of possible DAC values */
#define AD7441XR_THRESHOLD_DAC_RANGE		29
/** The comparator's value can be set betwen 0 - 16 V*/
#define AD7441XR_THRESHOLD_RANGE		16000

#define AD7441XR_TEMP_OFFSET			-2392
#define AD7441XR_TEMP_SCALE			8950
#define AD7441XR_TEMP_SCALE_DIV			1000

#define AD7441XR_RANGE_10V_SCALE		15259ULL
#define AD7441XR_RANGE_10V_SCALE_DIV		100000ULL
#define AD7441XR_RANGE_2V5_SCALE		38147ULL
#define AD7441XR_RANGE_2V5_SCALE_DIV		1000000ULL
#define AD7441XR_RANGE_5V_SCALE			76294ULL
#define AD7441XR_RANGE_5V_SCALE_DIV		1000000ULL
#define AD7441XR_RANGE_5V_OFFSET		-(AD7441XR_ADC_MAX_VALUE / 2)
#define AD7441XR_RTD_PULL_UP			2100000ULL

/**
 * @brief The chips supported by this driver.
 */
enum ad7441xr_chip_id {
	ad74413r,
	ad74412r
};

/**
 * @brief Rejection config values. The HART variants are not supported by the
 * AD74412R device.
 */
enum ad7441xr_rejection {
	AD7441XR_REJECTION_50_60,
	AD7441XR_REJECTION_NONE,
	AD7441XR_REJECTION_50_60_HART,
	AD7441XR_REJECTION_HART
};

/**
 * @brief Operation modes of the device.
 */
enum ad7441xr_op_mode {
	AD7441XR_HIGH_Z,
	AD7441XR_VOLTAGE_OUT,
	AD7441XR_CURRENT_OUT,
	AD7441XR_VOLTAGE_IN,
	AD7441XR_CURRENT_IN_EXT,
	AD7441XR_CURRENT_IN_LOOP,
	AD7441XR_RESISTANCE,
	AD7441XR_DIGITAL_INPUT,
	AD7441XR_DIGITAL_INPUT_LOOP,
	AD7441XR_CURRENT_IN_EXT_HART,
	AD7441XR_CURRENT_IN_LOOP_HART,
};

/**
 * @brief ADC ranges configurations. These are dependent on the operation mode.
 */
enum ad7441xr_adc_range {
	AD7441XR_ADC_RANGE_10V,
	AD7441XR_ADC_RANGE_2P5V_EXT_POW,
	AD7441XR_ADC_RANGE_2P5V_INT_POW,
	AD7441XR_ADC_RANGE_5V_BI_DIR
};

/**
 * @brief ADC conversion sequence commands.
 */
enum ad7441xr_conv_seq {
	AD7441XR_STOP_PWR_UP,
	AD7441XR_START_SINGLE,
	AD7441XR_START_CONT,
	AD7441XR_STOP_PWR_DOWN,
};

/**
 * @brief GPO operation modes.
 */
enum ad7441xr_gpo_select {
	AD7441XR_GPO_CONFIG_100K_PD,
	AD7441XR_GPO_CONFIG_DATA,
	AD7441XR_GPO_CONFIG_PAR_DATA,
	AD7441XR_GPO_CONFIG_COMP,
	AD7441XR_GPO_CONFIG_HIGH_Z
};

/**
 * @brief Possible values to be loaded in the DIAG_RESULT register
 */
enum ad7441xr_diag_mode {
	AD7441XR_DIAG_AGND,
	AD7441XR_DIAG_TEMP,
	AD7441XR_DIAG_AVDD,
	AD7441XR_DIAG_AVSS,
	AD7441XR_DIAG_REFOUT,
	AD7441XR_DIAG_ALDO_5V,
	AD7441XR_DIAG_ALDO_1V8,
	AD7441XR_DIAG_DLDO_1V8,
	AD7441XR_DIAG_DVCC,
	AD7441XR_DIAG_IOVDD,
	AD7441XR_SENSEL_A,
	AD7441XR_SENSEL_B,
	AD7441XR_SENSEL_C,
	AD7441XR_SENSEL_D
};

/**
 * @brief Debounce modes for the IOx inputs when using the digital input op mode.
 */
enum ad7441xr_debounce_mode {
	AD7441XR_DEBOUNCE_MODE_0,
	AD7441XR_DEBOUNCE_MODE_1
};

/**
 * @brief The number of increments per step a DAC does when slew control is enabled.
 */
enum ad7441xr_slew_lin_step {
	AD7441XR_STEP_64,
	AD7441XR_STEP_120,
	AD7441XR_STEP_500,
	AD7441XR_STEP_1820,
};

/**
 * @brief Possible update rates for a DAC when slew control is enabled
 */
enum ad7441xr_lin_rate {
	AD7441XR_LIN_RATE_4KHZ,
	AD7441XR_LIN_RATE_64KHZ,
	AD7441XR_LIN_RATE_150KHZ,
	AD7441XR_LIN_RATE_240KHZ,
};

enum ad7441xr_adc_sample {
	AD7441XR_ADC_SAMPLE_20HZ = 20,
	AD7441XR_ADC_SAMPLE_4800HZ = 4800,
	AD7441XR_ADC_SAMPLE_10HZ = 10,
	AD7441XR_ADC_SAMPLE_1200HZ = 1200,
};

/**
 * @brief Initialization parameter for the device descriptor.
 */
struct ad7441xr_init_param {
	enum ad7441xr_chip_id chip_id;
};

/**
 * @brief ADC value format
 */
struct ad7441xr_adc_value {
	int64_t integer;
	uint32_t decimal;
};

/**
 * @brief Device channel state.
 */
struct ad7441xr_channel_config {
	bool enabled;
	enum ad7441xr_op_mode function;
};

/**
 * @brief Bitfield struct which maps on the LIVE_STATUS register
 */
struct _ad7441xr_live_status {
	uint8_t VI_ERR_A: 1;
	uint8_t VI_ERR_B: 1;
	uint8_t VI_ERR_C: 1;
	uint8_t VI_ERR_D: 1;
	uint8_t HI_TEMP_ERR: 1;
	uint8_t CHARGE_PUMP_ERR: 1;
	uint8_t ALDO5V_ERR: 1;
	uint8_t AVDD_ERR: 1;
	uint8_t DVCC_ERR: 1;
	uint8_t ALDO1V8_ERR: 1;
	uint8_t ADC_CH_CURR: 3;
	uint8_t ADC_BUSY: 1;
	uint8_t ADC_DATA_RDY: 1;
	uint8_t _RESERVED: 1;
};

/**
 * @brief Used to store the live status bit fields.
 */
union ad7441xr_live_status {
	struct _ad7441xr_live_status status_bits;
	uint16_t value;
};

/**
 * @brief Bitfield struct which maps on the ALERT_STATUS register
 */
struct _ad7441xr_alert_status {
	uint8_t VI_ERR_A: 1;
	uint8_t VI_ERR_B: 1;
	uint8_t VI_ERR_C: 1;
	uint8_t VI_ERR_D: 1;
	uint8_t HI_TEMP_ERR: 1;
	uint8_t CHARGE_PUMP_ERR: 1;
	uint8_t ALDO5V_ERR: 1;
	uint8_t AVDD_ERR: 1;
	uint8_t DVCC_ERR: 1;
	uint8_t ALDO1V8_ERR: 1;
	uint8_t ADC_CONV_ERR: 1;
	uint8_t ADC_SAT_ERR: 1;
	uint8_t SPI_SCLK_CNT_ERR: 1;
	uint8_t SPI_CRC_ERR: 1;
	uint8_t CAL_MEM_ERR: 1;
	uint8_t RESET_OCCURRED: 1;
};

/**
 * @brief Used to store the alert status bit fields.
 */
union ad7441xr_alert_status {
	struct _ad7441xr_alert_status error_bits;
	uint16_t value;
};

#endif