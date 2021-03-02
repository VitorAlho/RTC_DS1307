#include <stdint.h>
#include <time.h>
#include "DS1307.h"
#include "rtc.h"

Calendar *data2;
Calendar  data;
    
int8_t SetarHoraRTC (int segundo, int minuto, int hora, int dia, int mes, int ano){
    
    
    time_t timeInSeconds;
    
    data.tm_sec = segundo;
    data.tm_min = minuto;
    data.tm_hour = hora;
    data.tm_mday = dia;
    data.tm_mon = mes-1;
    data.tm_year = ano+100;
    
    timeInSeconds = mktime(&data);
    
    data2 = localtime(&timeInSeconds);
    
    return RTC_DS1307_I2C_write_calendar ( data2 );
}

int8_t RTC_calendarRequest(Calendar *data){        
    return RTC_DS1307_I2C_write_calendar ( data );
}