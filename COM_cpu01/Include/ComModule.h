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

#define MAILBOX_ONE 2 // Two different mailbox CAN for receive data from ADC
#define MAILBOX_TWO 3 // and Power Supply
//#define ENABLE_ADC  0x01010001
#define ENABLE_ADC 0x01000101
#define DISABLE_ADC 0x01000000
#define COM_NODE_ID 0x01
#define ADC_NODE_ID 0x03
#define PS_NODE_ID 0x30
#define TIMEOUT_CAN_RX 25 //SysTick == 10. TIMEOUT_RX_CAN= SysTick*25=250ms

extern FIFO FIFO_PowerSupplyTx; //FIFO Tx defined for Power Supply
//extern FIFO FIFO_PowerSupplyRX;   //FIFO Tx defined for Power Supply
extern FIFO FIFO_AdcTx; //FIFO Tx defined for ADC
//extern FIFO FIFO_AdcRX;           //FIFO Rx defined for ADC
extern FIFO FIFO_CanRx; //Unique CAN FIFO defined for store CAN
                        //messages from Power Supply and ADC
extern FIFO FIFO_PcRx;  //FIFO Tx defined for Industrial PC
extern FIFO FIFO_PcTx;  //FIFO Rx defined for Industrial PC

extern tCanMsg Diccionario_CanOpen[];
extern enum Indice_Diccionario_TPO OD_Index;
extern tCANMsgObject sMailboxOneCANOpenMsg;
extern tCANMsgObject sMailboxTwoCANOpenMsg;

/**
 * @brief 
 * 
 */
typedef struct sFlagsError
{
    union{
        uint16_t AllFlags;
        struct{
            uint16_t CmdNotExist:1;
            uint16_t AccessCmdForbidden:1;
        } Flags;
    } StatusFlags;
} FlagsError_t;
/**
 * @brief Flag used during transmission between Adc, Power Suplpy
 *  and Comunication PCB .These flags declare status for 
 *  available data present on Rx FIFOs and also errors from 
 *  comunication process.
 */
typedef struct sFlagsCom
{
    union {
        uint16_t AllFlags;
        struct
        {
            uint16_t TransmittedCanMsg : 1;
            uint16_t AdcDataAvailable : 1;
            uint16_t PSDataAvailable : 1;
            uint16_t ErrorAdcCom : 1;
            uint16_t ErrorPsCom : 1;
            uint16_t ErrorCom : 1;
        } Flags;
    } StatusFlags;
} FlagsCom_t;

extern FlagsCom_t StatusCom;
extern volatile uint32_t ulTimeOutCANRx;
extern volatile uint32_t ulSysTickFlag;
/**
 * @brief Variable used for save Voltage, Current and Temperature values
 * from ADC PCB. Voltage/Current values saved in two format, integer and
 * float point
 * 
 */
typedef struct sAdcValues
{
    uint32_t VoltageValue; //Value used by Chademo logic
    float floatVolatageValue;
    int32_t CurrentValue; //Value used by Chademo logic
    float floatCurrentValue;
    int16_t TempPositive;
    int16_t TempNegative;
    uint16_t NegativeCurrentValue;
} AdcValues_t;

extern AdcValues_t AdcValuesSaved;

void Init_CANOpenMsgFIFOs(void);
void Set_MailboxOne(void);
void Set_MailboxTwo(void);
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx, FIFO *ptr_MsgToTx, uint32_t DataToTx, uint16_t Node_ID);
sEstadoFIFO Transmit_CANOPenMsg(FIFO MsgToTx);
void Scheduler(void);

#endif /* COMMODULE_H_ */
