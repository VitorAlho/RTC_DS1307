#ifndef _RTC_H_
#define _RTC_H_

#include <stdint.h>
#include "DS1307.h"

int8_t SetarHoraRTC (int segundo, int minuto, int hora, int dia, int mes, int ano);

int8_t RTC_calendarRequest(Calendar *data);

#endif