//###########################################################################
//
// FILE:   COM_cpu01.c
//
// TITLE:  Comunicaciones con el Vehiculo Electrico mediante el protocolo de
// comunicaciones CAHDEMO v2.0 V2G con el microcontrolador de TI F2837xD.
//
//! \addtogroup Comunicaciones CHADEMO
//! <h1> Comunicaciones CHADEMO </h1>
//!
//!
//  CVS => v1.0 -> Inicio del SW.
//      => vX.X ->
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: 24/09/2018 $
//###########################################################################

/* SYSTEM CODE BEGIN Includes*/
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include <stdint.h>
/* SYSTEM CODE END Includes */

/* USER CODE BEGIN Includes */
#include "Inc_Drivers.h"
#include "DEF_Global.h"
#include "ComModule.h"
/* USER CODE END Includes */

uint32_t tmp = 0;

/**
 * @brief  Main funtio, init system mHW
 * 
 */
void main(void)
{
    uint16_t status = 0;

    Init_HW(); /* Initialize all the HW*/
    Hablitar_ISR();
    Config_CANB (500); //500kbit/s

/* USER CODE BEGIN */
    Set_PowerSupplyMailbox();
    Set_ADCMailbox();
    Init_CANOpenMsgFIFO();

#ifdef DEBUG
    OD_Index = Vo_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    tmp = 0x11223344;
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX,tmp,PS_NODE_ID);
    OD_Index = Io_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    tmp = 0x55550000;
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX,tmp,PS_NODE_ID);
    OD_Index = TempPos_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    tmp = 0x78965432;
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX,tmp,PS_NODE_ID);
    OD_Index = TempNeg_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    tmp = 0x77885522;
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX,tmp,PS_NODE_ID);
    Transmit_CANOPenMsg(FIFO_PowerSupplyTX);
#endif
    DELAY_US(5000);
    OD_Index = Config_ADC; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_AdcTX,ENABLE_ADC,ADC_NODE_ID);
    Transmit_CANOPenMsg(FIFO_AdcTX);


/* USER CODE END */
    
    for (;;)
    {
    }
}

//
// End of file
//
