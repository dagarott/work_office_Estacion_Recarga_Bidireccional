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
#include "Chademo.h"
#include "ComModule.h"

/* USER CODE END Includes */
#define ISR_ENABLE
extern FIFO FIFO_PowerSupplyTX;
/*
 *
 */
void main(void)
{
    Init_HW(); /* Initialize all the HW*/

#ifdef ISR_ENABLE
    Hablitar_ISR();
#endif

    Config_CANB (500); //500kbit/s

    uint16_t status = 0;
    //Set_CANIntHandler();
    Set_PowerSupplyMailbox();
    Set_ADCMailbox();
    Init_CANOpenMsgFIFO();
    OD_Index = Vo_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX);
    OD_Index = Io_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX);
    OD_Index = TempPos_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX);
    OD_Index = TempNeg_Chademo; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_PowerSupplyTX);
    Transmit_CANOPenMsg(FIFO_PowerSupplyTX);
    
    for (;;)
    {
        //Chademo ();
    }
}

//
// End of file
//
