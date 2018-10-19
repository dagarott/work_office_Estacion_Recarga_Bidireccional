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

#define MSG_DATA_LENGTH  8
#define NumMsg  10 
#define PS_NODE_ID 0x630    //Power Module 0        


extern tCanMsg Diccionario_CanOpen[];
extern enum Indice_Diccionario_TPO OD_Index;

void Init_CANOpenMsgFIFO(void);
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx);
uint16_t Transmit_CANOPenMsg(void);

#endif /* COMMODULE_H_ */
