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

tCANMsgObject sTXCANMessage;
/**
 * @brief  Main function, initialize clock system, HW, etc.
 * 
 */
void main(void)
{

    uint16_t status = 0;

    Init_HW(); /* Initialize all the HW*/
    Hablitar_ISR();
    Config_CANA(500); //500kbit/s

    /* USER CODE BEGIN */
    Init_CANOpenMsgFIFOs();
    Set_MailboxOne();
    Set_MailboxTwo();

    //Power Supply OFF. This sentence execute only once.
    //Check communication with power supply and set known values
    status = InitPowerSupply();

    /* USER CODE END */

    for (;;)
    {
        if (SysTickFlag == true)
        {
            SysTickFlag = false;
            Scheduler(); //Triggered every 10ms
        }

    }
}

//
// End of file
//
