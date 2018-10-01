// Copyright (c) 2018 Tara Keeling
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include <Arduino.h>
#include <USIWire.h>
#include "tssd1306/ssd1306.h"
#include "bmp280_driver/bmp280.h"

static const uint32_t TimeBetweenUpdatesInMS = 1000;

void WireStartTransmission( const int Address );
void WireWrite( const uint8_t* Data, const size_t Length );
void WireEndTransmission( void );

void Wire_BMP280Delay( uint32_t Milliseconds );
int8_t Wire_BMP280Read( uint8_t DevID, uint8_t RegAddr, uint8_t* Data, uint16_t Length );
int8_t Wire_BMP280Write( uint8_t DevID, uint8_t RegAddr, uint8_t* Data, uint16_t Length );

bool TryEnableTempSensor( void );
int32_t GetTemperature( void );

void WireStartTransmission( const int Address ) {
    Wire.beginTransmission( Address );
}

void WireWrite( const uint8_t* Data, const size_t Length ) {
    Wire.write( Data, Length );
}

void WireEndTransmission( void ) {
    Wire.endTransmission( );
}

void Wire_BMP280Delay( uint32_t Milliseconds ) {
    delay( Milliseconds );
}

int8_t Wire_BMP280Read( uint8_t DevID, uint8_t RegAddr, uint8_t* Data, uint16_t Length ) {
    int8_t Result = BMP280_OK;
    int BytesRead = 0;
    int i = 0;

    if ( Data != NULL ) {
        Wire.beginTransmission( ( int ) DevID );
            Wire.write( &RegAddr, sizeof( uint8_t ) );
        Result = ( Wire.endTransmission( ) == 0 ) ? BMP280_OK : BMP280_E_COMM_FAIL;

        if ( Result == BMP280_OK ) {
            BytesRead = Wire.requestFrom( ( int ) DevID, Length );

            if ( BytesRead == ( int ) Length ) {
                for ( i = 0; i < BytesRead; i++ ) {
                    Data[ i ] = Wire.read( );
                }
            } else {
                Result = BMP280_E_INVALID_LEN;
            }
        }
    } else {
        Result = BMP280_E_NULL_PTR;
    }

    return Result;
}

int8_t Wire_BMP280Write( uint8_t DevID, uint8_t RegAddr, uint8_t* Data, uint16_t Length ) {
    int8_t Result = BMP280_OK;

    if ( Data != NULL ) {
        Wire.beginTransmission( ( int ) DevID );
            Wire.write( &RegAddr, sizeof( uint8_t ) );
            Wire.write( Data, Length );
        Result = ( Wire.endTransmission( ) == 0 ) ? BMP280_OK : BMP280_E_COMM_FAIL;
    } else {
        Result = BMP280_E_NULL_PTR;
    }

    return Result;
}

struct bmp280_dev Tempsensor = {
    .chip_id = 0,
    .dev_id = BMP280_I2C_ADDR_PRIM,
    .intf = BMP280_I2C_INTF,
    .read = Wire_BMP280Read,
    .write = Wire_BMP280Write,
    .delay_ms = Wire_BMP280Delay
};

bool TempsensorIsReady = false;

const I2CProcs WireWrapper = {
    WireStartTransmission,
    WireWrite,
    WireEndTransmission,
    BUFFER_LENGTH
};

bool TryEnableTempSensor( void ) {
    struct bmp280_config SensorCfg;

    if ( bmp280_init( &Tempsensor ) == BMP280_OK ) {
        if ( bmp280_get_config( &SensorCfg, &Tempsensor ) == BMP280_OK ) {
            SensorCfg.os_temp = BMP280_OS_4X;
            SensorCfg.os_pres = BMP280_OS_16X;
            SensorCfg.odr = BMP280_ODR_1000_MS;
            SensorCfg.filter = BMP280_FILTER_COEFF_2;

            if ( bmp280_set_config( &SensorCfg, &Tempsensor ) == BMP280_OK ) {
                return ( bmp280_set_power_mode( BMP280_NORMAL_MODE, &Tempsensor ) ) == BMP280_OK ? true : false;
            }
        }
    }

    return false;
}

int32_t GetTemperature( void ) {
    struct bmp280_uncomp_data Data;
    uint8_t MeasurementTime = 0;
    int i = 0;

    MeasurementTime = bmp280_compute_meas_time( &Tempsensor );

    for ( ; i < 8; i++ ) {
        delay( MeasurementTime );

        if ( bmp280_get_uncomp_data( &Data, &Tempsensor ) != BMP280_OK ) {
            break;
        }

        return bmp280_comp_temp_32bit( Data.uncomp_temp, &Tempsensor );
    }

    return 0;
}

void setup( void ) {
    Wire.begin( );

    SSD1306_Init( &WireWrapper );
    SSD1306_SetFont( &Font_Tarable7Seg_16x32 );

    if ( TryEnableTempSensor( ) == false ) {
        SSD1306_PrintString( "FFFFFFFF" );
    } else {
        TempsensorIsReady = true;
    }
}

void loop( void ) {
    uint32_t NextUpdateTime = 0;
    int32_t BigTemperature = 0;
    int TemperatureHi = 0;
    int TemperatureLo = 0;
    uint32_t TimeNow = 0;

    while ( TempsensorIsReady == true ) {
        TimeNow = millis( );

        if ( TimeNow >= NextUpdateTime ) {
            NextUpdateTime = TimeNow + TimeBetweenUpdatesInMS;

            BigTemperature = GetTemperature( );
            TemperatureHi = ( int ) ( BigTemperature / 100 );
            TemperatureLo = ( int ) ( BigTemperature % 100 );

            SSD1306_PrintInt( TemperatureHi );
            SSD1306_PrintChar( '.' );
            SSD1306_PrintInt( TemperatureLo );
            SSD1306_PrintString( "'C   \r" );
        }
    }
}
