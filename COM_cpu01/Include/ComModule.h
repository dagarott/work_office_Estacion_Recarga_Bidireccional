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
#include "Config_SCI.h"
#include "Diccionario_CANOpen.h"
#include "FIFO.h"

#define MAILBOX_ONE 2 // Two different mailbox CAN for receive data from ADC
#define MAILBOX_TWO 3 // and Power Supply
#define ENABLE_ADC 0x01000101
#define DISABLE_ADC 0x01000000
#define ENABLE_PS 0x00000001
#define COM_NODE_ID 0x01
#define ADC_NODE_ID 0x03
#define PS_NODE_ID 0x30
#define TIMEOUT_CAN_RX 1 //SysTick == 10ms. TIMEOUT_RX_CAN= SysTick*1=10ms

//Hardware dependent. Powersupply features
#define V2G500V15A_MAX_VOLTAGE 5000 //Model works with resolution of 0.1V/bit. Then 5000 = 5000/10 = 500V
#define V2G500V15A_MIN_VOLTAGE 500  //Model works with resolution of 0.1V/bit. Then 500 = 500/10 = 50V
#define V2G500V15A_CURRENT 150      //Model works with resolution of 0.1A/bit. then 150/10 = 15A
#define LENGHT_SECONDS_RAMP 4       //Total number of seconds of ramp up/down process
#define MS_STEP_RAMP 500


#define CPU01_TO_CPU02_PASSMSG 0x0003FC00 // CPU01 TO CPU02 MSG RAM (256 words)
#define CPU02_TO_CPU01_PASSMSG 0x0003F800 // CPU02 TO CPU01 MSG RAM (256 words)
#define END_PASSMSG 0x00003FF00

extern FIFO FIFO_CanTx; //Unique CAN FIFO defined for store CAN tx
extern FIFO FIFO_CanRx; //Unique CAN FIFO defined for store CAN rx
                        //messages from Power Supply and ADC
extern FIFO FIFO_PcRx;  //FIFO Tx defined for Industrial PC
extern FIFO FIFO_PcTx;  //FIFO Rx defined for Industrial PC

extern tCanMsg Diccionario_CanOpen[];
extern enum Indice_Diccionario_TPO OD_Index;
extern tCANMsgObject sMailboxOneCANOpenMsg;
extern tCANMsgObject sMailboxTwoCANOpenMsg;

/**
 * @brief Struct to store different errors flags for technical use 
 * 
 */
typedef struct sFlagsError
{
    union {
        uint16_t AllFlags;
        struct
        {
            uint16_t ObjectIndexError : 1;
            uint16_t AccessCmdError : 1;
            uint16_t SubIndexError : 1;
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
            uint16_t AccessModeRead : 1;
            uint16_t AccessModeWrite : 1;
            uint16_t TransmittedCanMsg : 1;
            uint16_t DataAvailable : 1;
            uint16_t DataAnalyzed : 1;
            uint16_t ErrorAdcCom : 1;
            uint16_t ErrorPsCom : 1;
            uint16_t ErrorCom : 1;
        } Flags;
    } StatusFlags;
} FlagsCom_t;

extern FlagsCom_t StatusCom;
extern volatile uint32_t ulTimeOutCANRx;
extern volatile uint16_t SysTickFlag;
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
    union {
        uint16_t AllFlags;
        struct
        {
            // Flags used during CAN comunication
            uint16_t VoltageAnswerFromAdc : 1;
            uint16_t CurrentAnswerFromAdc : 1;
        } Flags;
    } StatusFlags;
} AdcValues_t;

//extern AdcValues_t AdcValuesSaved;

/**
 * @brief 
 * 
 */
typedef struct sPowerSupplyValues
{
    unsigned char Model[4];       //Store device name/model
    int16_t CurrentDeviceValue;   //Values according to model
    uint16_t VoltageDeviceValue;  //Values according to model
    uint16_t PowerModuleStatus;   //[0-15]bit status
                                  //Bit /   Description /   Value
                                  // 0      Ps On  / 0: OFF; 1: ON
                                  // 1      Communication / 0: OK COM; 1: ERROR COM
                                  // 2      Updated Value / 0: NO; 1: YES
                                  // 3      Initiated Ps  / 0: NO; 1: YES
    uint16_t DCOutputVoltage;     //0x2107 Actual output voltage 0.1V/step
    int16_t DCOutputCurrrent;     //0x2108 Actual output current 0.1A/step
    uint16_t DCOutputVSetpoint;   //0x2109 Set point output voltage 0.1A/step
    int16_t DCOutputISetpoint;    //0x210A Set point output current 0.1A/step
    uint16_t DCBusVoltage;        //0x210D DC Bus Voltage
    int16_t DCBusVoltageMeasured; //0x212A DC Bus voltage filtered
    uint16_t ActualVoltageValue;
    int16_t ActualCurrentValue;

    union {
        uint16_t AllFlags;
        struct
        {
            // Flags used during CAN comunication
            uint16_t AnswerFromPs : 1;
            uint16_t AnswerDeviceName : 1;
            uint16_t AnswerDeviceNameError : 1;
            uint16_t AnswerVSet : 1;
            uint16_t AnswerVSetError : 1;
            uint16_t AnswerISet : 1;
            uint16_t AnswerISetError : 1;
            uint16_t AnswerEnablePs : 1;
            uint16_t AnswerReadVoutput : 1;
            uint16_t AnswerReadIoutput : 1;           
        } Flags;
    } StatusFlags;
} PowerSupplyValues_t;



//
//  Flags IPC de sincronizacion con CPU1 y CPU2
//
#define IPC_Start IpcRegs.IPCSTS.bit.IPC1
#define Set_IPC_Start() IPCLtoRFlagSet(IPC_FLAG1)
#define Clear_IPC_Start() IPCLtoRFlagClear(IPC_FLAG1)
#define IPC_Stop IpcRegs.IPCSTS.bit.IPC2
#define Set_IPC_Stop() IPCLtoRFlagSet(IPC_FLAG2)
#define Clear_IPC_Stop() IPCLtoRFlagClear(IPC_FLAG2)
#define IPC_Error IpcRegs.IPCSTS.bit.IPC3
#define Set_IPC_Error() IPCLtoRFlagSet(IPC_FLAG3)
#define Clear_IPC_Error() IPCLtoRFlagClear(IPC_FLAG3)
#define IPC_CPU1toCPU2 IpcRegs.IPCSTS.bit.IPC4
#define Set_IPC_CPU1toCPU2() IPCLtoRFlagSet(IPC_FLAG4)
#define Clear_IPC_CPU1toCPU2() IPCLtoRFlagClear(IPC_FLAG4)
#define IPC_CPU2toCPU1 IpcRegs.IPCSTS.bit.IPC5
#define Set_IPC_CPU2toCPU1() IPCLtoRFlagSet(IPC_FLAG5)
#define Clear_IPC_CPU2toCPU1() IPCLtoRFlagClear(IPC_FLAG5)
#define IPC_Potencia_Ready IpcRegs.IPCSTS.bit.IPC6
#define Set_IPC_Potencia_Ready() IPCLtoRFlagSet(IPC_FLAG6)
#define Clear_IPC_Potencia_Ready() IPCLtoRFlagClear(IPC_FLAG6)
#define IPC_V2G_Compatible IpcRegs.IPCSTS.bit.IPC7
#define Set_IPC_V2G_Compatible() IPCLtoRFlagSet(IPC_FLAG7)
#define Clear_IPC_V2G_Compatible() IPCLtoRFlagClear(IPC_FLAG7)
#define IPC_EVPS_Inversor IpcRegs.IPCSTS.bit.IPC8
#define Set_IPC_EVPS_Inversor() IPCLtoRFlagSet(IPC_FLAG8)
#define Clear_IPC_EVPS_Inversor() IPCLtoRFlagClear(IPC_FLAG8)

typedef struct sIPCRegisterValues
{
    uint16_t Start : 1;
    uint16_t EndProcess : 1;
    uint16_t Emergency : 1;
    uint16_t DataModifiedByCPU1 : 1;
    uint16_t DataModifiedByCPU2 : 1;

} IPCregisterValues_t;

/**
*******************************************************************************
\typedef        tModoEstacion
\brief          Tipo definido para la identificación del modo de actuar la
                estacion.
\enum           eModoEstacion
\brief          Enumerado definido para la identificación del modo de actuar la
                estacion.
******************************************************************************/
typedef enum eStationMode
{
    /** Sin definido aún */
    Modo_Sin_Definir = 0,
    /** Modo cargador de VE    */
    Cargardor,
    /** Modo inversor, injeccion a red   */
    Inversor,

} tStationMode;

/**
 * @brief Public Functions
 * 
 */
void Init_CANOpenMsgFIFOs(void);
void Set_MailboxOne(void);
void Set_MailboxTwo(void);
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                              FIFO *ptr_MsgToTx, uint32_t DataToTx,
                              uint16_t Idx_Node, uint8_t WR);
uint16_t Set_CANOpenErrorMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                                   FIFO *ptr_MsgToTx, uint32_t DataToTx,
                                   uint16_t Idx_Node);
sEstadoFIFO Transmit_CANOPenMsg(FIFO MsgToTx);
void Scheduler(void);

#endif /* COMMODULE_H_ */
