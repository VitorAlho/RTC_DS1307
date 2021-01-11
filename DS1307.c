/* 
 * 
 * File: DS1307.C
 * 
 * Author: Eng. Vitor Alho
 * 
 * Comments: Biblioteca com alto nível de abstração paro relógio RTC DS1307
 * 
 * Data: Dezembro de 2020
 * 
 * Mais informações no arquivo DS1307.H
 * 
 */
#include <xc.h>
#include "DS1307.h"

///////////////////////////////////////////////////////////////
/////////////////////    REGISTRADORES    /////////////////////
///////////////////////////////////////////////////////////////

#define SEC_REG  0x00          // Range 00-59
#define MIN_REG  0x01          // Range 00-59
#define HOUR_REG 0x02          // Range 01-12 +AM/PM OU 00-23

// Good when timekeeping functions are not required
#define CLOCK_HALT_BIT    ( 1 << 7 )  // 1 = clock output disabled | 0 = clock output enabled
                        
#define HOUR_12_OR_24_BIT ( 1 << 6 )  // 1 = 12 AM/PM format | 0 = 24 format
#define AM_PM_BIT         ( 1 << 5 )  // when HOUR_12_OR_24_BIT = 1, this is AM/PM indicator
    
#define DAY_OF_WEEK_REG 0x03   // Range 01-07
#define DAY_OF_MONTH_REG 0x04  // Range 01-31
#define MONTH_REG 0x05         // Range 01-12
#define YEAR_REG 0x06          // Range 00-99
    
//                                 CONTROL REG 
//       | BIT 7 | BIT 6 | BIT 5 | BIT 4 | BIT 3 | BIT 2 | BIT 1 | BIT 0 |  
//       |  OUT  |   0   |   0   |  SQWE |   0   |   0   |  RS1  |  RS0  |
//   
//    Bit 7: Output Control (OUT). This bit controls the output level of the SQW/OUT pin when the square-wave output
//        is disabled. If SQWE = 0, the logic level on the SQW/OUT pin is 1 if OUT = 1 and is 0 if OUT = 0. On initial
//        application of power to the device, this bit is typically set to a 0.
//    Bit 4: Square-Wave Enable (SQWE). This bit, when set to logic 1, enables the oscillator output. The frequency of
//        the square-wave output depends upon the value of the RS0 and RS1 bits. With the square-wave output set to 1Hz,
//        the clock registers update on the falling edge of the square wave. On initial application of power to the device, this
//        bit is typically set to a 0.
//    Bits 1 and 0: Rate Select (RS[1:0]). These bits control the frequency of the square-wave output when the squarewave output has been enabled. The following table lists the square-wave frequencies that can be selected with the
//        RS bits. On initial application of power to the device, these bits are typically set to a 1.
//
//    | RS1 | RS0 | SQW/OUT OUTPUT | SQWE | OUT | 
//    |  0  |  0  |       1Hz      |   1  |  X  |
//    |  0  |  1  |     4.096kHz   |   1  |  X  |
//    |  1  |  0  |     8.192kHz   |   1  |  X  |
//    |  1  |  1  |    32.768kHz   |   1  |  X  |
//    |  X  |  X  |        0       |   0  |  0  |
//    |  X  |  X  |        1       |   0  |  1  |

#define CONTROL_REG 0x07       
    
#define RAM_START_REG 0x08     // Range 56 x 8 ( 00h - FFh ) - Go from address0x08 to 0x3F

#define MAX_DS1307_TIMEOUT 100

#define RTC_DS1307_ADDRESS 0xD0

#define I2C_WRITE_FLAG 0xFE
#define I2C_READ_FLAG  0x01

#define RTC_DS1307_MIN_DELAY_BETWEEN_OPERATIONS 1

RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_start) (int16_t timeout);
RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_stop) (int16_t timeout);
RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_restart) (int16_t timeout);
RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_getRxData) (uint8_t *data, int16_t timeout);
RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_transmitTxData) (uint8_t data, int16_t timeout);
RTC_DS1307_stats (*RTC_DS1307_I2C_wrapper_sendAck) (int16_t timeout);

void (*RTC_DS1307_I2C_wrapper_delay_ms) (uint16_t delay_ms);

RTC_DS1307_stats RTC_DS1307_I2C_config( void ) {
    
    Calendar configCalendar;
    
    uint8_t control_reg;
    
    RTC_DS1307_stats status = RTC_DS1307_ERROR;
    
    status = RTC_DS1307_I2C_read_calendar( &configCalendar );                     if( status != RTC_DS1307_OK ) return status;
    
    configCalendar.tm_sec &= (CLOCK_HALT_BIT | 0xFF); // clock halted;
    
    status = RTC_DS1307_I2C_write_config_reg( SEC_REG, configCalendar.tm_sec );   if( status != RTC_DS1307_OK ) return status;
        
    configCalendar.tm_hour &= ~HOUR_12_OR_24_BIT; // 24 hours format
    
    status = RTC_DS1307_I2C_write_config_reg( HOUR_REG, configCalendar.tm_hour ); if( status != RTC_DS1307_OK ) return status;
     
    control_reg = 0x03;
    
    status = RTC_DS1307_I2C_write_config_reg( CONTROL_REG, control_reg );         if( status != RTC_DS1307_OK ) return status;
    
    return RTC_DS1307_OK;
}

RTC_DS1307_stats RTC_DS1307_I2C_write_calendar ( Calendar *data ){

    Calendar bcdCalendar;
    uint16_t year, month;
    
    RTC_DS1307_stats status = RTC_DS1307_ERROR;
    
    bcdCalendar.tm_sec = 0;
    bcdCalendar.tm_min = 0;
    bcdCalendar.tm_hour = 0;
    bcdCalendar.tm_wday = 0;
    bcdCalendar.tm_mday = 0;
    bcdCalendar.tm_mon = 0;
    bcdCalendar.tm_year = 0;
    
    // Adequação de valores por conta da biblioteca <time.h>
    month = data->tm_mon  + 1;   // RTC conta de 1 a 12, time.h de 0 a 11
    year  = data->tm_year - 100; // RTC conta de 00 a 99, time.h retorna um valor que,
                                 // somado ao ano de 1900, obtêm-se o ano atual. 
    
    if(data->tm_sec  >= 0 && data->tm_sec  <= 59 &&
       data->tm_min  >= 0 && data->tm_min  <= 59 &&
       data->tm_hour >= 0 && data->tm_hour <= 23 &&
       data->tm_wday >= 1 && data->tm_wday <= 7  &&
       data->tm_mday >= 1 && data->tm_mday <= 31 &&
       month         >= 1 && month         <= 12 &&
       year          >= 0 && year          <= 99 ){
        
        RTC_DS1307_CharToBcd( data->tm_sec,  (uint8_t*)&bcdCalendar.tm_sec);
        RTC_DS1307_CharToBcd( data->tm_min,  (uint8_t*)&bcdCalendar.tm_min);
        RTC_DS1307_CharToBcd( data->tm_hour, (uint8_t*)&bcdCalendar.tm_hour);    
        RTC_DS1307_CharToBcd( data->tm_wday, (uint8_t*)&bcdCalendar.tm_wday);
        RTC_DS1307_CharToBcd( data->tm_mday, (uint8_t*)&bcdCalendar.tm_mday);
        RTC_DS1307_CharToBcd(         month, (uint8_t*)&bcdCalendar.tm_mon);
        RTC_DS1307_CharToBcd(          year, (uint8_t*)&bcdCalendar.tm_year);

        status = RTC_DS1307_I2C_start( MAX_DS1307_TIMEOUT );                                            if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( RTC_DS1307_ADDRESS & I2C_WRITE_FLAG, MAX_DS1307_TIMEOUT );  if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( SEC_REG, MAX_DS1307_TIMEOUT );                              if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_sec, MAX_DS1307_TIMEOUT );                   if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_min, MAX_DS1307_TIMEOUT );                   if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_hour, MAX_DS1307_TIMEOUT );                  if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_wday, MAX_DS1307_TIMEOUT );                  if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_mday, MAX_DS1307_TIMEOUT );                  if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_mon, MAX_DS1307_TIMEOUT );                   if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_sendTxData( bcdCalendar.tm_year, MAX_DS1307_TIMEOUT );                  if( status != RTC_DS1307_OK ) return status;

        status = RTC_DS1307_I2C_stop( MAX_DS1307_TIMEOUT );  if( status != RTC_DS1307_OK ) return status;

        RTC_DS1307_delay_ms (RTC_DS1307_MIN_DELAY_BETWEEN_OPERATIONS);

        return RTC_DS1307_OK;
        
    }
    else{
        
        return RTC_DS1307_ERROR;
        
    }      

}

RTC_DS1307_stats RTC_DS1307_I2C_read_calendar( Calendar *data ){

    time_t timeInSeconds = 0;
    
    RTC_DS1307_stats status = RTC_DS1307_ERROR;
    
    status = RTC_DS1307_I2C_start( MAX_DS1307_TIMEOUT );                                           if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_sendTxData( RTC_DS1307_ADDRESS & I2C_WRITE_FLAG, MAX_DS1307_TIMEOUT ); if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_sendTxData( SEC_REG, MAX_DS1307_TIMEOUT );                             if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_restart( MAX_DS1307_TIMEOUT );                                         if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_sendTxData( RTC_DS1307_ADDRESS | I2C_READ_FLAG, MAX_DS1307_TIMEOUT );  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_sec, MAX_DS1307_TIMEOUT );             if( status != RTC_DS1307_OK ) return status;

    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_min, MAX_DS1307_TIMEOUT );             if( status != RTC_DS1307_OK ) return status;

    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_hour, MAX_DS1307_TIMEOUT );            if( status != RTC_DS1307_OK ) return status;

    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_wday, MAX_DS1307_TIMEOUT );            if( status != RTC_DS1307_OK ) return status;
  
    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_mday, MAX_DS1307_TIMEOUT );            if( status != RTC_DS1307_OK ) return status;
  
    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_mon, MAX_DS1307_TIMEOUT );             if( status != RTC_DS1307_OK ) return status;

    RTC_DS1307_I2C_sendAck( MAX_DS1307_TIMEOUT );                                                  if( status != RTC_DS1307_OK ) return status;    
    
    status = RTC_DS1307_I2C_getRxData( (uint8_t*) &data->tm_year, MAX_DS1307_TIMEOUT );            if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_stop( MAX_DS1307_TIMEOUT );                                            if( status != RTC_DS1307_OK ) return status;
    
    RTC_DS1307_delay_ms (RTC_DS1307_MIN_DELAY_BETWEEN_OPERATIONS);
    
    RTC_DS1307_BcdToChar( data->tm_sec, (uint8_t*)&data->tm_sec );
    
    RTC_DS1307_BcdToChar( data->tm_min, (uint8_t*)&data->tm_min );
    
    RTC_DS1307_BcdToChar( data->tm_hour, (uint8_t*)&data->tm_hour );
    
    RTC_DS1307_BcdToChar( data->tm_wday, (uint8_t*)&data->tm_wday );
    
    RTC_DS1307_BcdToChar( data->tm_mday, (uint8_t*)&data->tm_mday );
    
    RTC_DS1307_BcdToChar( data->tm_mon, (uint8_t*)&data->tm_mon );
    
    RTC_DS1307_BcdToChar( data->tm_year, (uint8_t*)&data->tm_year );
    
    if(data->tm_sec  >= 0 && data->tm_sec  <= 59 &&
       data->tm_min  >= 0 && data->tm_min  <= 59 &&
       data->tm_hour >= 0 && data->tm_hour <= 23 &&
       data->tm_wday >= 1 && data->tm_wday <= 7  &&
       data->tm_mday >= 1 && data->tm_mday <= 31 &&
       data->tm_mon  >= 1 && data->tm_mon  <= 12 &&
       data->tm_year >= 0 && data->tm_year <= 99 ){
        
        // Adequação de valores por conta da biblioteca <time.h>
        data->tm_mon  -= 1;   // RTC conta de 1 a 12, time.h de 0 a 11
        data->tm_year += 100; // RTC conta de 00 a 99, time.h retorna um valor que,
                              // somado ao ano de 1900, obtêm-se o ano atual.    
        
        timeInSeconds = mktime(data);
        
        data = localtime(timeInSeconds);
        
        return RTC_DS1307_OK;
        
    }
    else{
        
        return RTC_DS1307_ERROR;
        
    }
        
}

void RTC_DS1307_load_callbacks(    RTC_DS1307_I2C_callback     I2C_start,
                                   RTC_DS1307_I2C_callback     I2C_restart,
                                   RTC_DS1307_I2C_callback     I2C_stop,
                                   RTC_DS1307_I2C_callback     I2C_sendData_uchar,
                                   RTC_DS1307_I2C_callback     I2C_receiveData_uchar,
                                   RTC_DS1307_I2C_callback     I2C_sendAck,
                                   RTC_DS1307_delay_callback   rtc_ds1307_delay_ms) {
    
    RTC_DS1307_I2C_wrapper_start           = (RTC_DS1307_stats (*) (int16_t))           I2C_start;
    RTC_DS1307_I2C_wrapper_stop            = (RTC_DS1307_stats (*) (int16_t))           I2C_stop;
    RTC_DS1307_I2C_wrapper_restart         = (RTC_DS1307_stats (*) (int16_t))           I2C_restart;
    RTC_DS1307_I2C_wrapper_getRxData       = (RTC_DS1307_stats (*) (uint8_t*, int16_t)) I2C_receiveData_uchar;
    RTC_DS1307_I2C_wrapper_transmitTxData  = (RTC_DS1307_stats (*) (uint8_t,  int16_t)) I2C_sendData_uchar;
    RTC_DS1307_I2C_wrapper_sendAck         = (RTC_DS1307_stats (*) (int16_t))           I2C_sendAck;
    
    RTC_DS1307_I2C_wrapper_delay_ms        = (void (*) (uint16_t)) rtc_ds1307_delay_ms;
    
}

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// FUNÇÕES USADAS INTERNAMENTE ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

void RTC_DS1307_delay_ms ( uint16_t ms_delay ) {
    
    (*RTC_DS1307_I2C_wrapper_delay_ms)(ms_delay);
    
}

RTC_DS1307_stats  RTC_DS1307_I2C_start(int16_t timeout) {
    switch( (*RTC_DS1307_I2C_wrapper_start)(timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
}

RTC_DS1307_stats  RTC_DS1307_I2C_restart(int16_t timeout) {
    switch( (*RTC_DS1307_I2C_wrapper_restart)(timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
}

RTC_DS1307_stats  RTC_DS1307_I2C_stop(int16_t timeout) {
    switch( (*RTC_DS1307_I2C_wrapper_stop)(timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
}

RTC_DS1307_stats RTC_DS1307_I2C_getRxData(uint8_t *data, int16_t timeout) {
    switch( (*RTC_DS1307_I2C_wrapper_getRxData)(data, timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
}

RTC_DS1307_stats RTC_DS1307_I2C_sendTxData(uint8_t data, int16_t timeout) {
    switch( (*RTC_DS1307_I2C_wrapper_transmitTxData)(data,timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
}

RTC_DS1307_stats RTC_DS1307_I2C_sendAck(uint8_t timeout) {
    
    switch( (*RTC_DS1307_I2C_wrapper_sendAck)(timeout) ) {
        case RTC_DS1307_TIMEOUT:
            return RTC_DS1307_TIMEOUT;
        break;
        case RTC_DS1307_OK:
            return RTC_DS1307_OK;
        break;
        default:
            return RTC_DS1307_ERROR;
        break;
    }
    
}

void RTC_DS1307_BcdToChar( uint8_t source, uint8_t *output) {

//            |            MSB           |   |      LSB      |
//  *output = ( ( ( source >> 4 ) & 0x07 ) * 10 ) + ( source & 0x0F );
    *output = ( ( ( source >> 4 ) ) * 10 ) + ( source & 0x0F );
    
}

void RTC_DS1307_CharToBcd( uint8_t source, uint8_t *output) {
    
//            |          MSB           | |      LSB      | 
    *output = ( ( ( source / 10) << 4) | ( source % 10 ) );
    
}

RTC_DS1307_stats RTC_DS1307_I2C_write_config_reg( uint8_t reg, uint8_t value ) {
    
    RTC_DS1307_stats status = RTC_DS1307_ERROR;
    
    status = RTC_DS1307_I2C_start( MAX_DS1307_TIMEOUT );                                            if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_sendTxData( RTC_DS1307_ADDRESS & I2C_WRITE_FLAG, MAX_DS1307_TIMEOUT );  if( status != RTC_DS1307_OK ) return status;

    status = RTC_DS1307_I2C_sendTxData( reg, MAX_DS1307_TIMEOUT );                                  if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_sendTxData( value, MAX_DS1307_TIMEOUT );                                if( status != RTC_DS1307_OK ) return status;
    
    status = RTC_DS1307_I2C_stop( MAX_DS1307_TIMEOUT );                                             if( status != RTC_DS1307_OK ) return status;
    
    RTC_DS1307_delay_ms (RTC_DS1307_MIN_DELAY_BETWEEN_OPERATIONS);
    
    return RTC_DS1307_OK;
    
}

