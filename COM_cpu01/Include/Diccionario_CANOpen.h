/*
 * Diccionario_CANOpen.h
 *
 *  Created on: 17 sept. 2018
 *      Author: jeniher
 */

#ifndef INCLUDE_DICCIONARIO_CANOPEN_H_
#define INCLUDE_DICCIONARIO_CANOPEN_H_

#include <stdint.h>

/**
 *******************************************************************************
 \typedef        sCanMsg
 \brief          Typedef for CANOpen Messages
 ******************************************************************************/
typedef struct sCanMsg
{
    /**  CD, Command Byte. Access mode, Read or Write */
    char Modo_Acceso;
    /** OD, Object Dictionary index */
    uint16_t ID;
    /** SI. SubIndex Message*/
    char SubIndice;
} tCanMsg;

/**
 ********************************************************************************
 \typedef        enum Indice_Diccionario_TPO
 \brief          Enumeramos el indice del Diccionario de los mensajes TPO
 *******************************************************************************/
enum Indice_Diccionario_TPO
{
//  0x100x
    Nombre_dispositivo,
    Hardware_dispositivo,
    Software_dispositivo,

//  0x101x
    Identity,

//  0x210x
    Module_Enable,
    Module_Status,
    Module_Temperature,
    Uac_Input,
    Iac_Input,
    Udc_Out,
    Idc_Out,
    Udc_Out_Setpoint,
    Idc_Out_Setpoint,
    Udc_Bus,
    I_Solar_Out,

//  0x212x
    Uac_Input_Average,
    Uac_Input_L1,
    Uac_Input_L2,
    Uac_Input_L3,
    Iac_Input_L1,
    Iac_Input_L2,
    Iac_Input_L3,
    UBus,
    UBus_OVP,

    //  0x213x
    Warning_Status,
    Error_Source,
    Temp_Ambient,
    Temp_Sec,
    Temp_Prim,
    Temp_Trafo,
//  0x214x,
    Udc_Setpoint_Real,
    Idc_Setpoint_Real,
//  0x215x
    Switch_OFF_Reason,
//  0x2FFx
    Set_Node_ID,
    Restart_Module,

//  0x9000
    Vo_Chademo,         //Voltage value read by ADC from Power Supply
    Io_Chademo,         //Current value read by ADC from Power Supply
    TempPos_Chademo,    //Positive Temperature value read by ADC
    TempNeg_Chademo,    //Negative Temperature value read by ADC
#ifdef CCS_Protocol
//  0x9001
    Vo_CCS,             //Voltage value read by ADC from Power Supply
    Io_CCS,             //Current value read by ADC from Power Supply
    TempPos_CCS,        //Positive Temperature value read by ADC
    TempNeg_CCS,        //Negative Temperature value read by ADC
#endif
//  0x9002
    Config_ADC,         //Enable/disable ADC measure and select protocol 
                        //CHADEMO or CCS 
    ChkCom_ADC,         //Used for check comunication with ADC PCB                              
//  0xFFFF
    FIN_Diccionario,
};

tCanMsg  Consulta_Diccionario        (uint16_t indice);
uint16_t Econtrar_Indice_Diccionario (uint16_t ID, char subID);
uint32_t datos_char_to_int           (uint16_t Datos_Convertir[8]);
#endif /* INCLUDE_DICCIONARIO_CANOPEN_H_ */
