/*
 * File: usage.ino
 * Description: Basic application firmware to use the library
 * Author: Manuel Orduño
 * Date: 01/31/2023
 */

#include "Particle.h"
#include "ad7441xr.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

#define AD7441XR_CHIP_ID        ad74412r
#define AD7441XR_SPI_INTERFACE  SPI
#define AD7441XR_CS_PIN         D2
#define AD7441XR_RST_PIN        D1
#define AD7441XR_ALERT_PIN      D0

SerialLogHandler logHandler(115200, LOG_LEVEL_INFO);

AD7441XR swio(AD7441XR_CS_PIN, AD7441XR_SPI_INTERFACE, 
    AD7441XR_CHIP_ID, AD7441XR_RST_PIN, AD7441XR_ALERT_PIN);

int ret;
struct ad7441xr_adc_value result;
union ad7441xr_alert_status alerts;

system_tick_t lastReading = 0;
std::chrono::milliseconds readingInterval = 5s;

void setup() 
{
    waitFor(Serial.isConnected, 15000);

    swio.init();

    swio.setChannelStatus(AD7441XR_CH_A, false);
    swio.setChannelStatus(AD7441XR_CH_B, false);
    swio.setChannelStatus(AD7441XR_CH_C, false);

    // enable channel D and configure it as analog voltage input
    swio.setChannelStatus(AD7441XR_CH_D, true);
    swio.setChannelFunction(AD7441XR_CH_D, AD7441XR_VOLTAGE_IN);
}

void loop() 
{

    // if the alert pin is asserted, read the Alerts register (and clear the flag)
    if(!digitalRead(AD7441XR_ALERT_PIN))
    { 
        swio.getAlertStatus(&alerts);
        Log.info("Alert Status: 0x%04X\n", alerts.value);
    }
    
    if (millis() - lastReading >= readingInterval.count()) 
    {
		lastReading = millis();

        swio.getAdcValue(AD7441XR_CH_D, &result);
        Log.info("Channel D: %ld"".%02lu mV (Voltage input)", (int32_t)result.integer, result.decimal);
    }
}