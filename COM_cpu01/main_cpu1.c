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

uint16_t count1, count2;
uint16_t sdataC[9];    // Send data for SCI-A
tCANMsgObject sTXCANMessage;

/**
 * @brief  Main function, initialize clock system, HW, etc.
 * 
 */
void main(void)
{
    Init_HW(); /* Initialize HW*/
    Hablitar_ISR();
    InitPeripherals();

    /* USER CODE BEGIN */
    Init_CANOpenMsgFIFOs();
    Set_MailboxOne();
    Set_MailboxTwo();

    //PowerSupplyValues.RequiredOnOffProcess = true;
    AdcValuesSaved.RequiredOnOffProcess =true;
    
    /* USER CODE END */

    for (;;)
    {
        if (SysTickFlag == true)
        {
            SysTickFlag = false;
            Scheduler(); //Triggered every 1ms
        }
    }
}

//
// End of file
//
