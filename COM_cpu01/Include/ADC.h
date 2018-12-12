#ifndef ADC_H_
#define ADC_H_

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "Inc_Drivers.h"
#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "can.h"
#include "systick.h"

#include "ComModule.h"
#include "Config_CAN.h"
#include "Config_SCI.h"
#include "Diccionario_CANOpen.h"
#include "FIFO.h"

#define ENABLE_ADC 0x01000101
#define DISABLE_ADC 0x01000000
#define CHKCOM_ADC 0x01000000
#define ADC_NODE_ID 0x03

/**
 * @brief Variable used for save Voltage, Current and Temperature values
 * from ADC PCB. Voltage/Current values saved in two format, integer and
 * float point
 * 
 */
typedef struct sAdcValues
{
    uint32_t VoltageValue; //Value used by Chademo logic
    int32_t CurrentValue; //Value used by Chademo logic
    int16_t TempPositive;
    int16_t TempNegative;
    uint16_t NegativeCurrentValue;
    uint16_t AdcModuleStatus;     //[0-15]bit status
                                  //Bit /   Description /   Value
                                  // 0      Adc PCB On  / 0: OFF; 1: ON
                                  // 1      Communication Adc PCB / 0: OK COM; 1: ERROR COM
                                  // 2      Updated Value from Adc PCB / 0: NO; 1: YES
                                  // 3      Initiated Adc PCB  / 0: NO; 1: YES
    uint16_t RequiredOnOffProcess; //Used to inform that Adc PCB must be switched ON/OFF

    union {
        uint16_t AllFlags;
        struct
        {
            // Flags used during CAN comunication
            uint16_t DisableAnswerFromADC :1; 
            uint16_t VoltageAnswerFromAdc : 1;
            uint16_t CurrentAnswerFromAdc : 1;
            uint16_t ChkComAnswerFromAdc : 1;
        } Flags;
    } StatusFlags;
} AdcValues_t;

extern AdcValues_t AdcValuesSaved;

uint16_t AdcCheckCom(void);
uint16_t AdcSetEnableDisable(bool EnableAdc);
void AdcEnableDisable(bool EnableAdc);

#endif /* ADC_H_ */

