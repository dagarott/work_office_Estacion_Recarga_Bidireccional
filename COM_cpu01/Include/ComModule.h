/*
 * ComModule.h
 *
 *  Created on: 15 oct. 2018
 *      Author: dagaro
 */

#ifndef COMMODULE_H_
#define COMMODULE_H_

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "Inc_Drivers.h"
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "can.h"
#include "systick.h"

#include "Config_CAN.h"
#include "Diccionario_CANOpen.h"
#include "FIFO.h"

#define CAN_OBJ_ID_PS 2   //Power Suply
#define CAN_OBJ_ID_ADC 3  //ADC

extern tCanMsg Diccionario_CanOpen[];
extern enum Indice_Diccionario_TPO OD_Index;
extern tCANMsgObject sRXPowerSupply_CANOpenMsg;
extern tCANMsgObject sRXADC_CANOpenMsg;

void Init_CANOpenMsgFIFO(void);
void Set_PowerSupplyMailbox(void);
void Set_ADCMailbox(void);
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx, FIFO MsgToTx);
uint16_t Transmit_CANOPenMsg(FIFO MsgToTx);
//void Set_CANIntHandler(void);
//interrupt void CANIntHandler(void);

#endif /* COMMODULE_H_ */
