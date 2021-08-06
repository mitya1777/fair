/**
******************************************************************************
  * @file           : packed_time.c
  * @brief          : the packed time format using
										: converting time from data stored in RTC registers
										: (for STM32 microcontrollers)
******************************************************************************
**/

#include "packed_time.h"

uint32_t packed_time_fixation(void)	{
	
	uint8_t mark_years, mark_months, mark_days,
					mark_hours, mark_minutes, mark_seconds;
	uint8_t leap_year = RESET;
	uint8_t leaps_quantity = RESET;
	uint8_t sec_in_minute = 60;
	uint16_t sec_in_hour = 3600;
	uint32_t sec_in_day = 86400;
	uint32_t sec_in_year, sec_in_months = RESET;
	uint32_t time_pack = RESET;

	mark_years  = (((RTC -> DR & 0xF00000) >> 0x14) * 0x0A) + ((RTC -> DR & 0xF0000) >> 0x10);					//	conversion the years number from BCD to positional numeral system
	mark_months = (((RTC -> DR & 0x1000) >> 0x0C) * 0x0A) + ((RTC -> DR & 0xF00) >> 0x08);						//	conversion the months number from BCD to positional numeral system
	mark_days   = (((RTC -> DR & 0x70) >> 0x04) * 0x0A) + (RTC -> DR & 0x0F);									//	conversion the days number from BCD to positional numeral system

	mark_hours   = (((RTC -> TR & 0x300000) >> 0x14) * 0x0A) + ((RTC -> TR & 0xF0000) >> 0x10);					//	conversion the hours number from BCD to positional numeral system
	mark_minutes = (((RTC -> TR & 0x7000) >> 0x0C) * 0x0A) + ((RTC -> TR & 0xF00) >> 0x08);						//	conversion the minutes number from BCD to positional numeral system
	mark_seconds = (((RTC -> TR & 0x70) >> 0x04) * 0x0A) + (RTC -> TR & 0x0F);									//	conversion the seconds number from BCD to positional numeral system

	if(((mark_years + 2000) % 4) != 0x00)	{																	//	leap year checking
			leap_year = 0x00;
	}
	else	{
		if(((mark_years + 2000) % 100) != 0x00)	{
			leap_year = 0x01;
		}
		else	{
			if(((mark_years + 2000) % 400) != 0x00)	{
				leap_year = 0x00;
			}
			else	{
				leap_year = 0x01;
			}
		}
	}

	sec_in_year = leap_year == 0x01 ? 31622400 : 31536000;														//	the number of seconds depends on the leap year

	for(uint8_t i = 0x01; i < mark_months; i ++)	{															//	сalculation of the number of seconds depending on the number of past months
		if(i == 0x01 || i == 0x03 || i == 0x05 || i == 0x07 || i == 0x08 || i == 0x0A)	{
			sec_in_months += 2678400;
		}
		else if(i == 0x04 || i == 0x06 || i == 0x09 || i == 0x0B)	{
			sec_in_months += 2592000;
		}
		else	{
			sec_in_months += ((leap_year == 0x01) ? 2505600 : 2419200);
		}
	}

	leaps_quantity = mark_years / 4 - ((leap_year == 0x01) ? 0x01 : 0x00);										//	past leap years calculation at the moment

	time_pack = (mark_years - 1 -leaps_quantity) * 31536000 +													//	the packed time format implication
							 leaps_quantity * 31622400 +
							 sec_in_months +
							(mark_days - 1) * sec_in_day +
							 mark_hours * sec_in_hour +
							 mark_minutes * sec_in_minute +
							 mark_seconds;

	return time_pack;
}
