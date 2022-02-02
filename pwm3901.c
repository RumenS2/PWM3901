/*
 * pwm3901.c
 *
 *  Created on: 02.02.2022
 *      Author: ?? datasheet
 */
#include <pwm3901.h>

uint8_t ReadReg(uint8_t Adr);
void WriteReg(uint8_t Adr,uint8_t Val);
void initRegisters(void);

uint8_t PWM3901_Reset(void)
{
  uint8_t stat=0x00;
  Lo_RST_EC3;Hi_NCS_EC1;Hi_MOSI_EC5;Hi_SCLK_EC6;
  TIM2->CNT=0; //insert reset 300us
  while (TIM2->CNT<(uint32_t)(72*300)){;} //timer counts with 72MHz
  Hi_RST_EC3;TIM2->CNT=0;//remove reset 50ms delay
  while (TIM2->CNT<(uint32_t)(72UL*50000UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz

  // Power on reset
//  WriteReg(0x3A, 0x5A);

  TIM2->CNT=0;//5ms delay
  while (TIM2->CNT<(uint32_t)(72UL*5000UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz

  // Test the SPI communication, checking chipId and inverse chipId
  uint8_t chipId = ReadReg(0x00);
  uint8_t chiprev = ReadReg(0x01);
 uint8_t dIpihc = ReadReg(0x5F);

  if (chipId != 0x49) {stat=0x01;goto exii;}
  if (chiprev != 0x00) {stat=0x02;goto exii;}
  if (dIpihc !=0xB6) {stat=0x04;goto exii;}

  // Reading the motion registers one time
  ReadReg(0x02);
  ReadReg(0x03);
  ReadReg(0x04);
  ReadReg(0x05);
  ReadReg(0x06);

  TIM2->CNT=0;//1ms delay
  while (TIM2->CNT<(uint32_t)(72UL*1000UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz

   initRegisters();
   TIM2->CNT=0;// 50ms delay
   while (TIM2->CNT<(uint32_t)(72UL*50000UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz
exii:
	return stat;
}


inline void t45usDelay(void) //45us delay
{
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(72*45)){;} //timer counts with 72MHz
}
inline void t35usDelay(void) //20us delay
{
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(72*35)){;} //timer counts with 72MHz
}
inline void tsrad35usDelay(void) //20us delay
{
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(72*20)){;} //timer counts with 72MHz
}


uint8_t PWM3901_WriteReadByte(uint8_t by)  //WRITE if by|0x80, READ by&0x7f
{
 for (int x=0;x<8;x++)
 {
   Lo_SCLK_EC6;TIM2->CNT=0;
   if (by&0x80) Hi_MOSI_EC5;else Lo_MOSI_EC5;
   while (TIM2->CNT<(uint32_t)(20)){;} //timer counts with 72MHz, i.e. 13ns steps
   Hi_SCLK_EC6;TIM2->CNT=0;
   while (TIM2->CNT<(uint32_t)(20)){;} //timer counts with 72MHz, i.e. 13ns steps
   by<<=1;
   if (Get_MISO_EC2) by|=0x01;
 }
 return by;
}


uint8_t ReadReg(uint8_t Adr)
{
	  Lo_NCS_EC1;
	  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(10)){;} //timer counts with 72MHz, i.e. 13ns steps
	  	  PWM3901_WriteReadByte(Adr);
	  	  tsrad35usDelay();

//	  	  tsrad35usDelay();  //????????????????????????
//	  	  tsrad35usDelay();  //????????????????????

	  	  uint8_t z=PWM3901_WriteReadByte(0);
	  	  t45usDelay();
	  Hi_NCS_EC1;
	return z;
}

void WriteReg(uint8_t Adr,uint8_t Val)
{
  Lo_NCS_EC1;
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(10)){;} //timer counts with 72MHz, i.e. 13ns steps
  PWM3901_WriteReadByte(Adr|0x80); //send address
  PWM3901_WriteReadByte(Val); //send data
  t45usDelay(); //delay befor CS high ???
  Hi_NCS_EC1;
//  t45usDelay(); //delay befor CS high ?????????????????
//  t45usDelay(); //delay befor CS high ????????????????????
//  t45usDelay(); //delay befor CS high ??????????????????
//  t45usDelay(); //delay befor CS high ???????????????
//  t45usDelay(); //delay befor CS high ??????????????????
}

void ReadMot(uint8_t* arr)
{
  uint8_t z;
  	  z=ReadReg(0x02);
      *arr=z;arr++;
      if ((z&0x80)==0) {goto exii;} //no motion and ??? save time?
    *arr=ReadReg(0x03); arr++;
    *arr=ReadReg(0x04); arr++;
    *arr=ReadReg(0x05); arr++;
    *arr=ReadReg(0x06);
exii:
return;
}

void ReadBurstMotion(uint8_t* arr, uint8_t nreg)   //12 def?
{
  Lo_NCS_EC1;
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(10)){;} //timer counts with 72MHz, i.e. 13ns steps
  PWM3901_WriteReadByte(0x16); //send address
  t45usDelay();
  for (int x=0; x<nreg; x++)
  {
    *arr=PWM3901_WriteReadByte(0xff);
    if ((*arr&0x80)==0) goto exii; ///no motion => ????????? save time?
    arr++;
  }
exii:
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(10)){;} //timer counts with 72MHz, i.e. 13ns steps
  Hi_NCS_EC1;
  TIM2->CNT=0;while (TIM2->CNT<(uint32_t)(36)){;} //not needed but to be guaranteed minimum inactive time
}

void LedOnOf(uint8_t ll)
{
	  TIM2->CNT=0;//0.5ms delay //?or more
	  while (TIM2->CNT<(uint32_t)(72UL*500UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz
  if (ll) ll=0x1C;
  WriteReg(0x7F, 0x14);
  WriteReg(0x6F, ll);
  WriteReg(0x7F, 0x00);
}

// Performance optimisation registers
void initRegisters(void)
{
  WriteReg(0x7F, 0x00);
  WriteReg(0x61, 0xAD);
  WriteReg(0x7F, 0x03);
  WriteReg(0x40, 0x00);
  WriteReg(0x7F, 0x05);
  WriteReg(0x41, 0xB3);
  WriteReg(0x43, 0xF1);
  WriteReg(0x45, 0x14);
  WriteReg(0x5B, 0x32);
  WriteReg(0x5F, 0x34);
  WriteReg(0x7B, 0x08);
  WriteReg(0x7F, 0x06);
  WriteReg(0x44, 0x1B);
  WriteReg(0x40, 0xBF);
  WriteReg(0x4E, 0x3F);
  WriteReg(0x7F, 0x08);
  WriteReg(0x65, 0x20);
  WriteReg(0x6A, 0x18);
  WriteReg(0x7F, 0x09);
  WriteReg(0x4F, 0xAF);
  WriteReg(0x5F, 0x40);
  WriteReg(0x48, 0x80);
  WriteReg(0x49, 0x80);
  WriteReg(0x57, 0x77);
  WriteReg(0x60, 0x78);
  WriteReg(0x61, 0x78);
  WriteReg(0x62, 0x08);
  WriteReg(0x63, 0x50);
  WriteReg(0x7F, 0x0A);
  WriteReg(0x45, 0x60);
  WriteReg(0x7F, 0x00);
  WriteReg(0x4D, 0x11);
  WriteReg(0x55, 0x80);
  WriteReg(0x74, 0x1F);
  WriteReg(0x75, 0x1F);
  WriteReg(0x4A, 0x78);
  WriteReg(0x4B, 0x78);
  WriteReg(0x44, 0x08);
  WriteReg(0x45, 0x50);
  WriteReg(0x64, 0xFF);
  WriteReg(0x65, 0x1F);
  WriteReg(0x7F, 0x14);
  WriteReg(0x65, 0x60);
  WriteReg(0x66, 0x08);
  WriteReg(0x63, 0x78);
  WriteReg(0x7F, 0x15);
  WriteReg(0x48, 0x58);
  WriteReg(0x7F, 0x07);
  WriteReg(0x41, 0x0D);
  WriteReg(0x43, 0x14);
  WriteReg(0x4B, 0x0E);
  WriteReg(0x45, 0x0F);
  WriteReg(0x44, 0x42);
  WriteReg(0x4C, 0x80);
  WriteReg(0x7F, 0x10);
  WriteReg(0x5B, 0x02);
  WriteReg(0x7F, 0x07);
  WriteReg(0x40, 0x41);
  WriteReg(0x70, 0x00);

  TIM2->CNT=0;//100ms delay
  while (TIM2->CNT<(uint32_t)(72UL*100000UL)){LL_IWDG_ReloadCounter(IWDG);} //timer counts with 72MHz

  WriteReg(0x32, 0x44);
  WriteReg(0x7F, 0x07);
  WriteReg(0x40, 0x40);
  WriteReg(0x7F, 0x06);
  WriteReg(0x62, 0xF0);
  WriteReg(0x63, 0x00);
  WriteReg(0x7F, 0x0D);
  WriteReg(0x48, 0xC0);
  WriteReg(0x6F, 0xD5);
  WriteReg(0x7F, 0x00);
  WriteReg(0x5B, 0xA0);
  WriteReg(0x4E, 0xA8);
  WriteReg(0x5A, 0x50);
  WriteReg(0x40, 0x80);
}

/*
// init data for 3901
const uin8_t ini3901_1[] =
{
    { 0x7F, 0x00 },
    { 0x61, 0xAD },
    { 0x7F, 0x03 },
    { 0x40, 0x00 },
    { 0x7F, 0x05 },
    { 0x41, 0xB3 },
    { 0x43, 0xF1 },
    { 0x45, 0x14 },
    { 0x5B, 0x32 },
    { 0x5F, 0x34 },
    { 0x7B, 0x08 },
    { 0x7F, 0x06 },
    { 0x44, 0x1B },
    { 0x40, 0xBF },
    { 0x4E, 0x3F },
    { 0x7F, 0x08 },
    { 0x65, 0x20 },
    { 0x6A, 0x18 },
    { 0x7F, 0x09 },
    { 0x4F, 0xAF },
    { 0x5F, 0x40 },
    { 0x48, 0x80 },
    { 0x49, 0x80 },
    { 0x57, 0x77 },
    { 0x60, 0x78 },
    { 0x61, 0x78 },
    { 0x62, 0x08 },
    { 0x63, 0x50 },
    { 0x7F, 0x0A },
    { 0x45, 0x60 },
    { 0x7F, 0x00 },
    { 0x4D, 0x11 },
    { 0x55, 0x80 },
    { 0x74, 0x1F },
    { 0x75, 0x1F },
    { 0x4A, 0x78 },
    { 0x4B, 0x78 },
    { 0x44, 0x08 },
    { 0x45, 0x50 },
    { 0x64, 0xFF },
    { 0x65, 0x1F },
    { 0x7F, 0x14 },
    { 0x65, 0x60 },
    { 0x66, 0x08 },
    { 0x63, 0x78 },
    { 0x7F, 0x15 },
    { 0x48, 0x58 },
    { 0x7F, 0x07 },
    { 0x41, 0x0D },
    { 0x43, 0x14 },
    { 0x4B, 0x0E },
    { 0x45, 0x0F },
    { 0x44, 0x42 },
    { 0x4C, 0x80 },
    { 0x7F, 0x10 },
    { 0x5B, 0x02 },
    { 0x7F, 0x07 },
    { 0x40, 0x41 },
    { 0x70, 0x00 }
};


// Delay 100 ms before resuming the below register writes
const uin8_t ini3901_2[] =
{
    { 0x32, 0x44 },
    { 0x7F, 0x07 },
    { 0x40, 0x40 },
    { 0x7F, 0x06 },
    { 0x62, 0xF0 },
    { 0x63, 0x00 },
    { 0x7F, 0x0D },
    { 0x48, 0xC0 },
    { 0x6F, 0xD5 },
    { 0x7F, 0x00 },
    { 0x5B, 0xA0 },
    { 0x4E, 0xA8 },
    { 0x5A, 0x50 },
    { 0x40, 0x80 },
};
*/

