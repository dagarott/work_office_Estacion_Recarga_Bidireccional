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

#define CAN_OBJ_ID_PS 2  //Power Suply
#define CAN_OBJ_ID_ADC 3 //ADC
//#define ENABLE_ADC  0x01010001
#define ENABLE_ADC  0x01000101
#define DISABLE_ADC 0x01000000
#define COM_NODE_ID 0x601
#define ADC_NODE_ID 0x603
#define PS_NODE_ID 0x630


extern FIFO FIFO_PowerSupplyTX; //FIFO Tx defined for Power Supply
extern FIFO FIFO_PowerSupplyRX; //FIFO Tx defined for Power Supply
extern FIFO FIFO_AdcTX;         //FIFO Tx defined for ADC
extern FIFO FIFO_AdcRX;         //FIFO Rx defined for ADC
extern FIFO FIFO_PcRX;          //FIFO Tx defined for Industrial PC
extern FIFO FIFO_PcTX;          //FIFO Rx defined for Industrial PC

extern tCanMsg Diccionario_CanOpen[];
extern enum Indice_Diccionario_TPO OD_Index;
extern tCANMsgObject sRXPowerSupply_CANOpenMsg;
extern tCANMsgObject sRXADC_CANOpenMsg;

/**
 * @brief Flag used during transmission between Adc, Power Suplpy
 *and Comunication PCB .These flags declare status for 
 *available data present on Rx FIFOs and also errors from 
 *comunication process.
 */
typedef struct sFlagsCom
{
   union
   {
        uint16_t AllFlags;
        struct
        {
            uint16_t AdcDataAvailable:1;
            uint16_t PsDataAvailable:1;
            uint16_t ErrorAdcCom:1;
            uint16_t ErrorPsCom:1;
        }Flags;
   }StatusFlags;
}FlagsCom_t;

extern FlagsCom_t StatusCom;

/**
 * @brief Struct used for save Voltage, Current and Temperature values
 * from ADC PCB
 * 
 */
typedef struct sAdcValues
{
    int32_t VoltageValue;
    uint32_t CurrentValue;
    int16_t TempPositive;
    int16_t TempNegative;
}AdcValues_t;

extern AdcValues_t AdcValuesSaved;

void Init_CANOpenMsgFIFO(void);
void Set_PowerSupplyMailbox(void);
void Set_ADCMailbox(void);
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx, FIFO *ptr_MsgToTx, uint32_t DataToTx, uint16_t Node_ID);
sEstadoFIFO Transmit_CANOPenMsg(FIFO MsgToTx);
//void Set_CANIntHandler(void);
//interrupt void CANIntHandler(void);

#endif /* COMMODULE_H_ */
