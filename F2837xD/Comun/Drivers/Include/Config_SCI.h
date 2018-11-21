/*
 * Config_SCIA.h
 *
 *  Created on: 16 may. 2018
 *      Author: jeniher
 */

#ifndef DRIVERS_INCLUDE_CONFIG_SCI_H_
#define DRIVERS_INCLUDE_CONFIG_SCI_H_

/**
*******************************************************************************
\typedef        tNumSCI
\brief          Tipo definido para la identificación del SCI del micro
\enum           eNumSCI
\brief          Enumerado definido para la identificación del SCI
******************************************************************************/
typedef enum eNumSCI
{
    /** SCI A */
    SCI_A = 0,
    /** SCI B */
    SCI_B,
    /** SCI C */
    SCI_C,
    /** SCI D */
    SCI_D,
}tNumSCI;


void Config_SCIA    (uint32_t BaudRate);
void Config_SCIC    (uint32_t BaudRate);
void Config_SCIB    (uint32_t BaudRate);
void Transmitir_SCI (tNumSCI SCIx, uint16_t Data);

#endif /* DRIVERS_INCLUDE_CONFIG_SCI_H_ */

