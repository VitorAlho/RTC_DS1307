/* 
 * 
 * File: DS1307.H
 * 
 * Author: Eng. Vitor Alho
 * 
 * Comments: Biblioteca com alto nível de abstração para o RTC DS1307
 * 
 * Data: Dezembro de 2020
 * 
 * Utilize somente as funções listadas abaixo. Demais são utilizadas apenas internamente.
 * 
 * Exemplo de utilização da biblioteca no MPLAB X IDE para o PIC24FJ256DA210
 * 
    
    //  Section: Included Files
    
    #include "mcc_generated_files/system.h"
    #include "i2c1_driver.h"

    #include "DS1307.h"

    
    //  Main application
    

    void delay_ms(uint16_t ms_delay) {

        __delay_ms(ms_delay);

    }

    void testarRTC(void);

    int main(void)
    {
        Calendar calendar, calendar2;

        RTC_DS1307_stats status;

        SYSTEM_Initialize();

        i2c1_driver_init(100000); // 100 KHz

        RTC_DS1307_load_callbacks(  (RTC_DS1307_I2C_callback)    i2c1_driver_start,
                                    (RTC_DS1307_I2C_callback)    i2c1_driver_restart,
                                    (RTC_DS1307_I2C_callback)    i2c1_driver_stop,
                                    (RTC_DS1307_I2C_callback)    i2c1_driver_TXData,
                                    (RTC_DS1307_I2C_callback)    i2c1_driver_getRXData,
                                    (RTC_DS1307_I2C_callback)    i2c1_driver_sendAck,                                
                                    (RTC_DS1307_delay_callback)  delay_ms);

        RTC_DS1307_I2C_config();

        calendar.tm_sec  = 50;
        calendar.tm_min  = 51;
        calendar.tm_hour = 15;
        calendar.tm_wday = 1;
        calendar.tm_mday = 11;
        calendar.tm_mon  = 0;
        calendar.tm_year = 121;

        calendar2.tm_sec  = 0;
        calendar2.tm_min  = 0;
        calendar2.tm_hour = 0;
        calendar2.tm_wday = 0;
        calendar2.tm_mday = 0;
        calendar2.tm_mon  = 0;
        calendar2.tm_year = 0;    

        status = RTC_DS1307_I2C_write_calendar( &calendar );

        while(1){

            __delay_ms(1000);

            status = RTC_DS1307_I2C_read_calendar( &calendar2 );  

        }

        return 1;
    }
 * 
 */

#ifndef _DS1307_H_
#define	_DS1307_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    
/* 
 * Enumerador com os status da comunicação I2C retornado por todas
 * as funções.
 */
typedef enum {    
    RTC_DS1307_TIMEOUT = -1,   
    RTC_DS1307_OK,    
    RTC_DS1307_ERROR,
    RTC_DS1307_ADDRESS_OUT_OF_RANGE
} RTC_DS1307_stats;

/* 
 * Ponteiro de função para receber endereços de memória das funções
 * do protocolo I2C.
 */
typedef RTC_DS1307_stats (*RTC_DS1307_I2C_callback)(int16_t);

typedef RTC_DS1307_stats (*RTC_DS1307_I2C_callbackW) (uint8_t, int16_t);

typedef RTC_DS1307_stats (*RTC_DS1307_I2C_callbackR) (uint8_t*, int16_t);

typedef void (*RTC_DS1307_delay_callback) (uint16_t);

typedef struct tm Calendar;


RTC_DS1307_stats RTC_DS1307_I2C_config( void );

/* 
 * Função que recebe o endereço de memória das funções 
 * do protocolo I2C e carrega dentro da biblioteca para 
 * comunicar com o relógio RTC
 */
void RTC_DS1307_load_callbacks(    RTC_DS1307_I2C_callback     I2C_start,
                                   RTC_DS1307_I2C_callback     I2C_restart,
                                   RTC_DS1307_I2C_callback     I2C_stop,
                                   RTC_DS1307_I2C_callback    I2C_sendData_uchar,
                                   RTC_DS1307_I2C_callback    I2C_receiveData_uchar,
                                   RTC_DS1307_I2C_callback     I2C_sendAck,
                                   RTC_DS1307_delay_callback   rtc_ds1307_delay_ms);

/* 
 * Função que realiza a LEITURA de um dado, em um determinado endereço do relógio
 * RTC.
 */
RTC_DS1307_stats RTC_DS1307_I2C_read_calendar ( Calendar *data );

/* 
 * Função que realiza a ESCRITA de um dado em um determinado endereço dA memória
 * RTC.
 */
RTC_DS1307_stats RTC_DS1307_I2C_write_calendar ( Calendar *data );

///////////////////////////////////////////////////////////////////////////////////////
////////////////////////// FUNÇÕES USADAS INTERNAMENTE ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

/* 
 * Função que sinaliza o início da transmissão MASTER -> SLAVE no barramento
 */
RTC_DS1307_stats  RTC_DS1307_I2C_start(int16_t timeout);

/* 
 * Função que sinaliza o restart da comunicação MASTER -> SLAVE no barramento
 */
RTC_DS1307_stats  RTC_DS1307_I2C_restart(int16_t timeout);

/* 
 * Função que sinaliza o fim (stop) da comunicação MASTER -> SLAVE no barramento
 */
RTC_DS1307_stats  RTC_DS1307_I2C_stop(int16_t timeout);

/* 
 * Função que recebe um dado de 8 bits enviado pelo SLAVE no barramento
 */
RTC_DS1307_stats RTC_DS1307_I2C_getRxData(uint8_t *data, int16_t timeout);

/* 
 * Função que envia um dado de 8 bits para o SLAVE no barramento
 */
RTC_DS1307_stats RTC_DS1307_I2C_sendTxData(uint8_t data, int16_t timeout);

RTC_DS1307_stats RTC_DS1307_I2C_sendAck(uint8_t timeout);

void RTC_DS1307_delay_ms ( uint16_t ms_delay );

void RTC_DS1307_BcdToChar( uint8_t source, uint8_t *output);

void RTC_DS1307_CharToBcd( uint8_t source, uint8_t *output);

RTC_DS1307_stats RTC_DS1307_I2C_write_config_reg( uint8_t reg, uint8_t value );

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

