/***************************************************************************//**
 *   @file   ad7441xr.h
 *   @brief  Header file of AD7441xR Driver.
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

#ifndef _AD7441XR_H_
#define _AD7441XR_H_

#include "stdint.h"
#include "stdbool.h"
#include "ad7441xr_dfs.h"

class AD7441XR {
    private:
        int _cs;
        SPIClass &spi;
        enum ad7441xr_chip_id chipId;
        int _rstPin;
        int _alertPin;
        boolean usingResetPin;
        boolean usingAlertPin;
        uint8_t _txBuffer[4];
        uint8_t _rxBuffer[4];
        struct ad7441xr_channel_config channelConfigs[AD7441XR_N_CHANNELS];
    
    private:
        static void _formatRegWrite(uint8_t, uint16_t, uint8_t *buff);

        /** Clear the ALERT_STATUS register */
        int _clearErrors();
        
        /** Get the number of active channels */
        int _getActiveChannels(uint8_t *);
        
        /** Get the rejection setting for a specific channel */
        int _getAdcRejection(uint32_t, enum ad7441xr_rejection *);
        
        /** Get a single ADC raw value for a specific channel, then power down the ADC */
        int _getAdcSingle(uint32_t, uint16_t *);

        /** Read the raw ADC raw conversion value */
        int _getRawAdcResult(uint32_t, uint16_t *);
        
        int _readRegister(uint32_t, uint16_t *);
        int _readRegisterRaw(uint32_t, uint8_t *);

        /** Comm test function */
        int _scratchTest();

        /** Start or stop ADC conversions */
        int _setAdcConversions(enum ad7441xr_conv_seq);

        int _updateRegister(uint32_t, uint16_t, uint16_t);
        int _writeRegister(uint32_t, uint16_t);
    
    public:
        AD7441XR(int, SPIClass &spi,  enum ad7441xr_chip_id, int rstPin = -1, int alertPin = -1);
        int init();

        /** Get the ADC real value, according to the operation mode */
        int getAdcValue(uint32_t, struct ad7441xr_adc_value *);

        /** Read the alert status bits */
        int getAlertStatus(union ad7441xr_alert_status *);
        
        /** Read the live status bits */
        int getLiveStatus(union ad7441xr_live_status *);

        /** Monitor ALERT pin */
        int loop();

        /** Perform a soft reset */
        int softReset();

        /** Enable/disable a specific ADC channel */
        int setChannelStatus(uint32_t, bool);
        
        /** Set the operation mode for a specific channel */
        int setChannelFunction(uint32_t, enum ad7441xr_op_mode);
};

#endif