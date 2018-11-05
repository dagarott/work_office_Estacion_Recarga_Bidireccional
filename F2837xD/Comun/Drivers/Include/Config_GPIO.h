/*
 * Config_GPIO.h
 *
 *  Created on: 18 abr. 2018
 *      Author: jeniher
 */

#ifndef DRIVERS_INCLUDE_CONFIG_GPIO_H_
#define DRIVERS_INCLUDE_CONFIG_GPIO_H_

void Config_GPIO();

//CPU1 COMUNICATION
// Serigrafia PCB       GPIO
#define P6_1_RX         18 //PIN 9  /GPIO 18  CANA_RX
#define P6_1_TX         19 //PIN 11 /GPIO 19  CANA_TX
#define J1_TX           12 //RS233 TX
#define J1_RX           13 //RS232 RX

//CPU2 COMUNICATION
// Serigrafia PCB       GPIO
#define P6_2_RX         20 //PIN 12 /GPIO 20  CANB_RX
#define P6_2_TX         21 //PIN 13 /GPIO 21  CANB_TX

//CPU2 OUTPUTs/INPUTs
// Serigrafia PCB      GPIO
#define P9_8           65 //Output
#define P9_7           64 //Output
#define P9_6           63 //Output
#define P9_5           62 //Output
#define P9_4           61 //Output
#define P9_3           41 //Output
#define P9_2           58 //Output
#define P9_1           59 //Output
#define P4_1           17 //Input
#define P4_2           99 //Input

//CPU1 OUTPUTs/INPUTs
// Serigrafia PCB      GPIO
#define P9_9           11 //Output
#define P9_10          86 //Output
#define P4_4           87 //Input
#define P4_5           2  //Input
#define P4_3           70 //Input. Emergency Stop9

//NOT USED YET!!
//Serigrafia PCB       GPIO
#define P4_6           3  //Input
#define P4_7           4  //Input
#define P9_12          89 //Output
#define P9_11          91 //Output
#define P2_1           AdcbResultRegs.ADCRESULT4   //ADCINB4
#define P2_2           AdcbResultRegs.ADCRESULT1   //ADCINB1
#define P5_ADC         AdcbResultRegs.ADCRESULT0 //ADCINB0
#define P2_3           AdcbResultRegs.ADCRESULT2 //ADCINB2
#define P2_4           AdcbResultRegs.ADCRESULT3 //ADCINB3
#define P5_IN          90 //INPUT ADC
#define P10_TX         14 //UART TX
#define P10_RX         15 //UART
#define P10_POL        16 //SELECT DIRECTION tx/rx del transceiver CAN-UART
#define P8_RX          43 //RS485 RX
#define P8_TX          42 //RS485 TX
#define P8_RE          10 //RS485 Receive Enable Output
#define P8_DE          92 //RS485 Driver Enable

#endif /* DRIVERS_INCLUDE_CONFIG_GPIO_H_ */
