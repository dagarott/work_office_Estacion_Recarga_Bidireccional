/**
***************************************************************************
\defgroup       DRIVERS

\brief          Modulo de inicializacion de los perifericos del
                TMS320F2877s

                Microcontroladores en los que se ha probado:
                TMS320F2877s

**************************************************************************/
/**
***************************************************************************
\file           CONFIG_CAN.c
\brief          Modulo de configuracion de los CAN del microcontrolador
                TMS320F2877s.

\author         Jesus Nieto Hervas
\version        V0.0
\date           16/05/2018
**************************************************************************/

//
// Included
//
#include "F28x_Project.h" // Device Headerfile and Examples Include File
#include "Inc_Drivers.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "can.h"

//
// Defines
//
#define MSG_DATA_LENGTH 8
#define TX_MSG_OBJ_ID 1
#define RX_MSG_OBJ_ID 2

//
// Estructuras
//
//extern FIFO FIFO_CANB;
// Buzones CAN RX para protocolo Chademo
// extern tCANMsgObject sRXCANMsg_100;
// extern unsigned char rxMsgData_100[MSG_DATA_LENGTH];
//
// Variables Globals
//
volatile unsigned long i;
volatile uint32_t errorFlag = 0;
unsigned char txMsgData[8];
unsigned char rxMsgData[8];
unsigned char rxMsgData2[8];
extern tCANMsgObject sTXCANMessage;
tCANMsgObject sRXCANMessage;
tCANMsgObject sRXCANMessage2;

/**
***************************************************************************
\fn         Config_CAN
\brief      Funcion principal de configuracion del CAN_A. Esta funcion
            es llamada desde el "main".

\param[in]  void
\return     void

**************************************************************************/
void Config_CANA(uint32_t BitRate)
{
    // Initialize the CAN controllers
    CANInit(CANA_BASE);

    // Setup CAN to be clocked off the PLL output clock
    CANClkSourceSelect(CANA_BASE, 0); // 500kHz CAN-Clock

    //               Num CAN | Frec de Reloj en Hz | BitRate en Bit/s
    // CANBitRateSet(ui32Base, ui32SourceClock     , ui32BitRate)
    CANBitRateSet(CANA_BASE, (uint32_t)T_clk, BitRate * 1000);

    // Enable interrupts on the CAN B peripheral.
    CANIntEnable(CANA_BASE, CAN_INT_MASTER);

    // Enable CAN Global Interrupt line0
    //
    CANGlobalIntEnable(CANA_BASE, CAN_GLB_INT_CANINT0);

    // Start CAN module A and B operations
    CANEnable(CANA_BASE);

}
// FIN Config_CANA

/**
***************************************************************************
\fn         Config_CAN
\brief      Funcion principal de configuracion del CAN_A. Esta funcion
            es llamada desde el "main".

\param[in]  void
\return     void

**************************************************************************/
void Config_CANB(uint32_t BitRate)
{
    // Initialize the CAN controllers
    CANInit(CANB_BASE);

    // Setup CAN to be clocked off the PLL output clock
    CANClkSourceSelect(CANB_BASE, 0); // 500kHz CAN-Clock

    //               Num CAN | Frec de Reloj en Hz | BitRate en Bit/s
    // CANBitRateSet(ui32Base, ui32SourceClock     , ui32BitRate)
    CANBitRateSet(CANB_BASE, (uint32_t)T_clk, BitRate * 1000);

    // Enable interrupts on the CAN B peripheral.
    CANIntEnable(CANB_BASE, CAN_INT_MASTER);

    // Enable CAN Global Interrupt line0
    //
    CANGlobalIntEnable(CANB_BASE, CAN_GLB_INT_CANINT0);

    // Start CAN module A and B operations
    CANEnable(CANB_BASE);

}
void Transmitir_CANA(uint32_t Ptr_Datos, uint16_t ID_CAN)
{
    memcpy (&sTXCANMessage.pucMsgData, &Ptr_Datos, sizeof(sTXCANMessage.pucMsgData));

    sTXCANMessage.ui32MsgID = (uint32_t)ID_CAN;

    CANMessageSet(CANA_BASE, TX_MSG_OBJ_ID, &sTXCANMessage, MSG_OBJ_TYPE_TX);


} // FIN Transmitir CAN B

void Recibir_CANB()
{
}

void Init_Buzones(tCANMsgObject CANBuzon, tMsgObjType TipoMsg, uint32_t NumBuzon)
{
    CANMessageSet(CANB_BASE, NumBuzon, &CANBuzon, TipoMsg);
}
//
// FIN DEL ARCHIVO
//
