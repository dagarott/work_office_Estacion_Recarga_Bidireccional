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

#define DEBUG

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
    Config_CANA(500); //500kbit/s

/* USER CODE BEGIN */
    Init_CANOpenMsgFIFOs();
    Set_MailboxOne();
    Set_MailboxTwo();

    OD_Index = Config_ADC; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
    status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx,ENABLE_ADC,RSDO+ADC_NODE_ID);
    Transmit_CANOPenMsg(FIFO_CanTx);
/* USER CODE END */
    
    for (;;)
    {
        //if(ulSysTickFlag)
        //{
         //   ulSysTickFlag=false;
            AnalyzeCanMsg(); //Triggered every 10ms
       // }
            
    }
}

//
// End of file
//
