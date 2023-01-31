/***************************************************************************//**
 *   @file   ad7441xr.cpp
 *   @brief  Source file of AD7441xR Driver.
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

#include "Particle.h"
#include "ad7441xr.h"
#include "crc8.h"
#include "util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD7441XR_FRAME_SIZE 		4
#define AD7441XR_CRC_POLYNOMIAL 	0x7
#define AD7441XR_DIN_DEBOUNCE_LEN 	NO_OS_BIT(5)

/******************************************************************************/
/************************ Variable Declarations ******************************/
/******************************************************************************/
DECLARE_CRC8_TABLE(_crc_table);

static const unsigned int ad7441xr_debounce_map[AD7441XR_DIN_DEBOUNCE_LEN] = {
	0,     13,    18,    24,    32,    42,    56,    75,
	100,   130,   180,   240,   320,   420,   560,   750,
	1000,  1300,  1800,  2400,  3200,  4200,  5600,  7500,
	10000, 13000, 18000, 24000, 32000, 42000, 56000, 75000,
};

/** The time required for an ADC conversion by rejection (us) */
static const uint32_t conv_times_ad74413r[] = { 208, 833, 50000, 100000 };
static const uint32_t conv_times_ad74412r[] = { 50000, 208};

AD7441XR::AD7441XR(int cs, SPIClass &spi, enum ad7441xr_chip_id id, int rst, int alrt) :
	_cs(cs), spi(spi), chipId(id), _rstPin(rst), _alertPin(alrt)	
{
    pinMode(_cs, OUTPUT);

	if (_rstPin != -1)
	{
		pinMode(_rstPin, OUTPUT);
		usingResetPin = true;
	}

	if (_alertPin != -1)
	{
		pinMode(_alertPin, INPUT_PULLUP);
		usingAlertPin = true;
	}
}

int AD7441XR::setChannelStatus(uint32_t ch, bool status)
{
    int ret;

	ret = _updateRegister(AD7441XR_ADC_CONV_CTRL, AD7441XR_CH_EN_MASK(ch), status);
	if (ret)
		return ret;

    channelConfigs[ch].enabled = status;

	return 0;
}

int AD7441XR::init()
{
    int ret;

	spi.begin(_cs);
	digitalWrite(_cs, HIGH);

	if (usingResetPin == true)
	{
		digitalWrite(_rstPin, HIGH);
	}

	crc8_populate_msb(_crc_table, AD7441XR_CRC_POLYNOMIAL);
	
	//  After power-up, the user must wait approximately 10 ms
	//  before any transaction to the device can take place.
	delay(10);

	ret = softReset();
	if (ret)
		return ret;

	ret = _clearErrors();
	if (ret)
		return ret;

	ret = _scratchTest();
	if (ret)
		return ret;
	
	return 0;

}

int AD7441XR::_clearErrors()
{
	return _writeRegister(AD7441XR_ALERT_STATUS, AD7441XR_ERR_CLR_MASK);
}

int AD7441XR::_getActiveChannels(uint8_t *nb_channels)
{
	int ret;
	uint16_t reg_val;

	ret = _readRegister(AD7441XR_ADC_CONV_CTRL, &reg_val);
	if (ret)
		return ret;

	reg_val = no_os_field_get(NO_OS_GENMASK(3, 0), reg_val);
	*nb_channels = no_os_hweight8((uint8_t)reg_val);

	return 0;
}

int AD7441XR::_getRawAdcResult(uint32_t ch, uint16_t *val)
{
	_readRegister(AD7441XR_ADC_RESULT(ch), val);
	return 0;
}

int AD7441XR::_getAdcRejection(uint32_t ch, enum ad7441xr_rejection *val)
{
	int ret;
	uint16_t rejection_val;

	ret = _readRegister(AD7441XR_ADC_CONFIG(ch), &rejection_val);
	if (ret)
		return ret;

	// CHECK
	//*val = no_os_field_get(AD7441XR_ADC_REJECTION_MASK, rejection_val);
	*val = static_cast<enum ad7441xr_rejection>(no_os_field_get(AD7441XR_ADC_REJECTION_MASK, rejection_val));

	return 0;
}

int AD7441XR::_getAdcSingle(uint32_t ch, uint16_t *val)
{
	int ret;
	uint32_t del;
	uint8_t nb_active_channels;
	enum ad7441xr_rejection rejection;

	ret = setChannelStatus(ch, true);
	if (ret)
		return ret;

	ret = _getActiveChannels(&nb_active_channels);
	if (ret)
		return ret;

	ret = _setAdcConversions(AD7441XR_START_SINGLE);
	if (ret)
		return ret;

	ret = _getAdcRejection(ch, &rejection);
	if (ret)
		return ret;

	if (chipId == ad74413r)
		del = conv_times_ad74413r[rejection];
	else
		del = conv_times_ad74412r[rejection];

	/** Wait for all channels to complete the conversion. */
	if (del < 1000)
		delayMicroseconds(del * nb_active_channels);
	else
		delay(del * nb_active_channels / 1000);

	ret = _getRawAdcResult(ch, val);
	if (ret)
		return ret;

	ret = _setAdcConversions(AD7441XR_STOP_PWR_DOWN);
	if (ret)
		return ret;

	ret = setChannelStatus(ch, false);
	if (ret)
		return ret;

	return 0;
}

int AD7441XR::getAdcValue(uint32_t ch, struct ad7441xr_adc_value *val)
{
	int ret;
	uint16_t adc_code;

	ret = _getAdcSingle(ch, &adc_code);
	if (ret)
		return ret;

	switch (channelConfigs[ch].function) {
	case AD7441XR_HIGH_Z:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	case AD7441XR_VOLTAGE_OUT:
		/**
		 * I_Rsense = (Vmin + (ADC_CODE/65535) * range) / Rsense
		 */
		val->integer = no_os_div_s64_rem((adc_code + AD7441XR_RANGE_5V_OFFSET) *
						 AD7441XR_RANGE_5V_SCALE,
						 AD7441XR_RSENSE * AD7441XR_RANGE_5V_SCALE_DIV,
						 (int32_t *)&val->decimal);
		break;
	case AD7441XR_CURRENT_OUT:
	case AD7441XR_VOLTAGE_IN:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	case AD7441XR_CURRENT_IN_EXT_HART:
		if (chipId == ad74412r)
			//return -ENOTSUP;
			return -134;
	case AD7441XR_CURRENT_IN_LOOP_HART:
		if (chipId == ad74412r)
			//return -ENOTSUP;
			return -134;
	case AD7441XR_CURRENT_IN_EXT:
	case AD7441XR_CURRENT_IN_LOOP:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_2V5_SCALE,
						 AD7441XR_RANGE_2V5_SCALE_DIV * AD7441XR_RSENSE,
						 &val->decimal);
		break;
	case AD7441XR_RESISTANCE:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RTD_PULL_UP,
						 AD7441XR_ADC_MAX_VALUE - adc_code,
						 &val->decimal);
		break;
	case AD7441XR_DIGITAL_INPUT:
	case AD7441XR_DIGITAL_INPUT_LOOP:
		val->integer = no_os_div_u64_rem(adc_code * AD7441XR_RANGE_10V_SCALE,
						 AD7441XR_RANGE_10V_SCALE_DIV,
						 &val->decimal);
		break;
	default:
		//return -EINVAL;
		return -22;
	}

	return 0;
}

int AD7441XR::getAlertStatus(union ad7441xr_alert_status *status)
{
	_readRegister(AD7441XR_ALERT_STATUS, &status->value);

	return _clearErrors();	
}

int AD7441XR::getLiveStatus(union ad7441xr_live_status *status)
{
	return _readRegister(AD7441XR_LIVE_STATUS, &status->value);	
}

int AD7441XR::loop()
{
	return digitalRead(_alertPin); 
}

int AD7441XR::_setAdcConversions(enum ad7441xr_conv_seq status)
{
	int ret;

	ret = _updateRegister(AD7441XR_ADC_CONV_CTRL, AD7441XR_CONV_SEQ_MASK, status);
	if (ret)
		return ret;
	
	/**
	 * The write to CONV_SEQ powers up the ADC. If the ADC was powered down, the user must wait
	 * for 100us before the ADC starts doing conversions.
	 */
	delayMicroseconds(100);

	return 0;
}

int AD7441XR::setChannelFunction(uint32_t ch, enum ad7441xr_op_mode ch_func)
{
    int ret;

	ret = _updateRegister(AD7441XR_CH_FUNC_SETUP(ch), AD7441XR_CH_FUNC_SETUP_MASK, ch_func);
	if (ret)
		return ret;

	ret = _updateRegister(AD7441XR_ADC_CONFIG(ch), AD7441XR_CH_200K_TO_GND_MASK, 1);
	if (ret)
		return ret;

	channelConfigs[ch].function = ch_func;

	return 0;
}

void AD7441XR::_formatRegWrite(uint8_t reg, uint16_t val, uint8_t *buff)
{
    buff[0] = reg;
	no_os_put_unaligned_be16(val, &buff[1]);
	buff[3] = crc8(_crc_table, buff, 3, 0);
}

int AD7441XR::_readRegister(uint32_t addr, uint16_t *val)
{
    int ret;
	uint8_t expected_crc;

	ret = _readRegisterRaw(addr, _rxBuffer);

	if (ret)
		return ret;

	expected_crc = crc8(_crc_table, _rxBuffer, 3, 0);
	if (expected_crc != _rxBuffer[3])
		return -22;
	

	*val = no_os_get_unaligned_be16(&_rxBuffer[1]);

	return 0;
}

int AD7441XR::_readRegisterRaw(uint32_t addr, uint8_t *val)
{
    //int ret;
	/**
	 * Reading a register on AD74413r requires writing the address to the READ_SELECT
	 * register first and then doing another spi read, which will contain the requested
	 * register value.
	 */

    _formatRegWrite(AD7441XR_READ_SELECT, addr, _txBuffer);

    spi.beginTransaction(SPISettings(1*MHZ, MSBFIRST, SPI_MODE1));
	digitalWrite(_cs, LOW);
	spi.transfer(_txBuffer, NULL, AD7441XR_FRAME_SIZE, NULL);
	digitalWrite(_cs, HIGH);
	spi.endTransaction();
    
	_formatRegWrite(AD7441XR_NOP, 0x00, _txBuffer);

    spi.beginTransaction(SPISettings(1*MHZ, MSBFIRST, SPI_MODE1));
	digitalWrite(_cs, LOW);
	spi.transfer(_txBuffer, val, AD7441XR_FRAME_SIZE, NULL);
	digitalWrite(_cs, HIGH);
	spi.endTransaction();
    
    return 0;
}

int AD7441XR::_scratchTest()
{
	int ret;
	uint16_t val;
	uint16_t test_val = 0x1234;

	ret = _writeRegister(AD7441XR_SCRATCH, test_val);
	if (ret)
		return ret;

	ret = _readRegister(AD7441XR_SCRATCH, &val);
	if (ret)
		return ret;

	if (val != test_val)
		return -22;

	return 0;
}

int AD7441XR::softReset()
{
	int ret;

	ret = _writeRegister(AD7441XR_CMD_KEY, AD7441XR_CMD_KEY_RESET_1);
	if (ret)
		return ret;

	return _writeRegister(AD7441XR_CMD_KEY, AD7441XR_CMD_KEY_RESET_2);
}

int AD7441XR::_updateRegister(uint32_t addr, uint16_t mask, uint16_t val)
{
    int ret;
	uint16_t c_val;

	ret = _readRegister(addr, &c_val);
	if (ret)
		return ret;

	c_val &= ~mask;
	c_val |= no_os_field_prep(mask, val);

	return _writeRegister(addr, c_val);
}

int AD7441XR::_writeRegister(uint32_t addr, uint16_t val)
{
    _formatRegWrite(addr, val, _txBuffer);

	spi.beginTransaction(SPISettings(1*MHZ, MSBFIRST, SPI_MODE1));
	digitalWrite(_cs, LOW);
	spi.transfer(_txBuffer, NULL, AD7441XR_FRAME_SIZE, NULL);
	digitalWrite(_cs, HIGH);
	spi.endTransaction();

	return 0;
}