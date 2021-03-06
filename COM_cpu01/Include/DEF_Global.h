/*
 * DEF_Global.h
 *
 *  Created on: 20 abr. 2018
 *      Author: jeniher
 */

#ifndef DEF_GLOBAL_H_
#define DEF_GLOBAL_H_

//
// Included Files
//
#include "F28x_Project.h"

//
// Variables publicas
//
typedef enum
{
    FALSE = 0,
    TRUE
} BOOL;

typedef union {
    uint32_t DATOS_32bits;
    struct
    {
        uint32_t Parte_Alta : 16;
        uint32_t Parte_Baja : 16;
    } DATOS_16bits;
    struct
    {
        uint32_t Parte_Alta2 : 8;
        uint32_t Parte_Alta1 : 8;
        uint32_t Parte_Baja2 : 8;
        uint32_t Parte_Baja1 : 8;
    } DATOS_8bits;
    struct
    {
        uint32_t Parte_Alta2B : 4;
        uint32_t Parte_Alta2A : 4;
        uint32_t Parte_Alta1B : 4;
        uint32_t Parte_Alta1A : 4;
        uint32_t Parte_Baja2B : 4;
        uint32_t Parte_Baja2A : 4;
        uint32_t Parte_Baja1B : 4;
        uint32_t Parte_Baja1A : 4;
    } DATOS_4bits;
} DATOS;

#define Stop P4_3
#define StatusStop() GPIO_ReadPin(Stop)
#define PsEnable P9_9
#define PsEnable_ON() GPIO_WritePin(PsEnable, 1)
#define PsEnable_OFF() GPIO_WritePin(PsEnable, 0)
#define EarthTest P9_10
#define EarthTest_ON() GPIO_WritePin(EarthTest, 1)
#define EarthTest_OFF() GPIO_WritePin(EarthTest, 0)
#define EarthEmergency P4_4
#define StatusEarthEmergency() GPIO_ReadPin(EarthEmergency)
#define ContactorNegativo P9_7
#define ContactorNegativoON() GPIO_WritePin(ContactorNegativo, 1)
#define ContactorNegativoOFF() GPIO_WritePin(ContactorNegativo, 0)
#define RelayPreCharge P9_5
#define RelayPreChargeON() GPIO_WritePin(RelayPreCharge, 1)
#define RelayPreChargeOFF() GPIO_WritePin(RelayPreCharge, 0)
#define ContactorCharge P9_8        //To diode 
#define ContactorChargeON() GPIO_WritePin(ContactorCharge, 1)
#define ContactorChargeOFF() GPIO_WritePin(ContactorCharge, 0)
#define ContactorPositive P9_6
#define ContactorPositiveON() GPIO_WritePin(ContactorPositive, 1)
#define ContactorPositiveOFF() GPIO_WritePin(ContactorPositive, 0)
#define RelayFan P9_4        //Used for cooling diode
#define RelayFanON() GPIO_WritePin(RelayFan, 1)
#define RelayFanOFF() GPIO_WritePin(RelayFan, 0)

#endif /* DEF_GLOBAL_H_ */
