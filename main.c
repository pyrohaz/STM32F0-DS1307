#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_i2c.h>

/*
 * DS1307 RTC example program using the STM32F0 Discovery board
 * available from STMicroelectronics
 *
 * Author: Harris Shallcross
 * Year: 03/02/2015
 *
 *Interfacing the DS1307 RTC through I2C.
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

#define DS_SCK		GPIO_Pin_10
#define DS_SDA		GPIO_Pin_11

#define DS_SCKPS	GPIO_PinSource10
#define DS_SDAPS	GPIO_PinSource11
#define DS_GAF		GPIO_AF_1
#define DS_GPIO		GPIOB

#define DS_I2C		I2C2
#define DS_ADDR		(0x68<<1)

//DS1307 Registers
#define DS_SECS		0x00
#define DS_MINS		0x01
#define DS_HOUR		0x02
#define DS_DAY		0x03
#define DS_DATE		0x04
#define DS_MNTH		0x05
#define DS_YEAR		0x06
#define DS_CTRL		0x07

volatile uint32_t MSec = 0;

const char C_TIME[] = __TIME__;
const char C_DATE[] = __DATE__;

typedef struct DS_RTC{
	uint8_t Second, Minute, Hour;
	uint8_t Day, Date, Month, Year;
} DS_RTC;

void Delay(uint32_t D){
	uint32_t MSS = MSec;
	while((MSec-MSS)<D) asm volatile("nop");
}

void SysTick_Handler(void){
	MSec++;
}

void I2C_WrReg(uint8_t Reg, uint8_t Dat){
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(DS_I2C, DS_ADDR, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(DS_I2C, Reg);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_TCR) == RESET);

	I2C_TransferHandling(DS_I2C, DS_ADDR, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(DS_I2C, Dat);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(DS_I2C, I2C_FLAG_STOPF);
}

uint8_t I2C_RdReg(uint8_t Reg){
	uint8_t Dat;
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_BUSY) == SET);

	I2C_TransferHandling(DS_I2C, DS_ADDR, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_TXIS) == RESET);

	I2C_SendData(DS_I2C, Reg);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_TC) == RESET);

	I2C_TransferHandling(DS_I2C, DS_ADDR, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);
	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_RXNE) == RESET);

	Dat = I2C_ReceiveData(DS_I2C);

	while(I2C_GetFlagStatus(DS_I2C, I2C_FLAG_STOPF) == RESET);
	I2C_ClearFlag(DS_I2C, I2C_FLAG_STOPF);

	return Dat;
}

int16_t DS_RdRAM(uint8_t Addr){
	if(Addr>0x3F) return -1;
	return I2C_RdReg(Addr+0x08);
}

int16_t DS_WrRAM(uint8_t Addr, uint8_t Dat){
	if(Addr>0x3F) return -1;
	I2C_WrReg(Addr+0x08, Dat);
	return 0;
}

void DS_SetTime(DS_RTC *R){
	uint8_t Seconds, TSeconds;
	uint8_t Minutes, TMinutes;
	uint8_t Hours, THours;
	uint8_t Date, TDate;
	uint8_t Month, TMonth;
	uint8_t Year, TYear;

	TSeconds = R->Second/10;
	Seconds = R->Second%10;

	TMinutes = R->Minute/10;
	Minutes = R->Minute%10;

	THours = R->Hour/10;
	Hours = R->Hour%10;

	TDate = R->Date/10;
	Date = R->Date%10;

	TMonth = R->Month/10;
	Month = R->Month%10;

	TYear = R->Year/10;
	Year = R->Year%10;

	I2C_WrReg(DS_SECS, Seconds | (TSeconds<<4));
	I2C_WrReg(DS_MINS, Minutes | (TMinutes<<4));
	I2C_WrReg(DS_HOUR, (Hours | (THours<<4)) | 0x40); //24 Hours
	I2C_WrReg(DS_DAY, R->Day);
	I2C_WrReg(DS_DATE, Date | (TDate<<4));
	I2C_WrReg(DS_MNTH, Month | (TMonth<<4));
	I2C_WrReg(DS_YEAR, Year | (TYear<<4));
}

void DS_GetTime(DS_RTC *R){
	uint8_t BSecond, BMinute, BHour;
	uint8_t BDay, BDate, BMonth, BYear;

	BSecond = I2C_RdReg(DS_SECS);
	BMinute = I2C_RdReg(DS_MINS);
	BHour = I2C_RdReg(DS_HOUR);
	BDay = I2C_RdReg(DS_DAY);
	BDate = I2C_RdReg(DS_DATE);
	BMonth = I2C_RdReg(DS_MNTH);
	BYear = I2C_RdReg(DS_YEAR);

	R->Second = (BSecond&0x0F) + 10*((BSecond>>4)&0x07);
	R->Minute = (BMinute&0x0F) + 10*((BMinute>>4)&0x07);
	R->Hour = (BHour&0x0F) + 10*((BHour>>4)&0x03);
	R->Day = (BDay&0x07);
	R->Date = (BDate&0x0F) + 10*((BDate>>4)&0x03);
	R->Month = (BMonth&0x0F) + 10*((BMonth>>4)&0x01);
	R->Year = (BYear&0x0F) + 10*(BYear>>4);
}

void DS_StructInit(DS_RTC *R){
	R->Day = 1;
	R->Date = 0;
	R->Month = 0;
	R->Year = 0;
	R->Hour = 0;
	R->Minute = 0;
	R->Second = 0;
}

GPIO_InitTypeDef G;
I2C_InitTypeDef I;

int main(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	SysTick_Config(SystemCoreClock/1000);

	G.GPIO_Pin = DS_SDA | DS_SCK;
	G.GPIO_OType = GPIO_OType_OD;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Speed = GPIO_Speed_Level_1;
	G.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(DS_GPIO, &G);

	GPIO_PinAFConfig(DS_GPIO, DS_SDAPS, DS_GAF);
	GPIO_PinAFConfig(DS_GPIO, DS_SCKPS, DS_GAF);

	I.I2C_Ack = I2C_Ack_Enable;
	I.I2C_AcknowledgedAddress = 0x00;
	I.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I.I2C_DigitalFilter = 0x00;
	I.I2C_Mode = I2C_Mode_I2C;
	I.I2C_OwnAddress1 = 0x01;
	I.I2C_Timing = 0x10805F87;
	I2C_Init(DS_I2C, &I);
	I2C_Cmd(DS_I2C, ENABLE);

	Delay(10);

	//Disable square wave output
	I2C_WrReg(DS_CTRL, 0x00);

	DS_RTC R;

	DS_StructInit(&R);

	//R.Date = ((C_DATE[4]-'0')*10) + (C_DATE[5]-'0');
	if(C_DATE[4] == ' ') R.Date = (C_DATE[5]-'0');
	else R.Date = ((C_DATE[4]-'0')*10) + (C_DATE[5]-'0');
	R.Month = 2;
	R.Year = ((C_DATE[9]-'0')*10) + (C_DATE[10]-'0');

	R.Hour = ((C_TIME[0]-'0')*10) + (C_TIME[1]-'0');
	R.Minute = ((C_TIME[3]-'0')*10) + (C_TIME[4]-'0');
	R.Second = ((C_TIME[6]-'0')*10) + (C_TIME[7]-'0');

	DS_WrRAM(0x00, 123);

	//DS_SetTime(&R);

	uint8_t Dat, SO = 99;

	while(1)
	{
		DS_GetTime(&R);
		Dat = DS_RdRAM(0x00);

		if(R.Second != SO){
			SO = R.Second;
		}
	}
}
