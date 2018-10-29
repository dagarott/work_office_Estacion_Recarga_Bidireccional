/*
 * Diccionario_CANOpen.c
 *
 *  Created on: 17 sept. 2018
 *      Author: jeniher
 */

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "Diccionario_CANOpen.h"
#include "Config_CAN.h"

// Esctructura para sacar la inforamcion del diccionario del CANopen
//
//TODO: Check why it gives me an error this declaration during compilation
//tCanMsg *D2;

/*******************************************************************************
 Definimos los datos para lel diccionario del CANOpen
 IMPORTANTE : El protocolo CANopen utiliza el formato Little-endian
 *******************************************************************************/

/**
 ********************************************************************************
 \struct     Envio datos Diccionario CANOpen

 \brief      --------------------------------------------------------
 |   Indice  |   SubIndice   |   LEN | Valor por Defecto |
 --------------------------------------------------------
 *******************************************************************************/
/*  Para facilitar el acceso a esta tabla esta definida el incide en la
 "estrctura enum Indice_Diccionario_TPO" en Diccionario_CANopen_DS18.h    */

/*  IMPORTANTE : Si se modifica esta tabla, modificar
 enum Indice_Diccionario_TPO en Diccionario_CANopen_DS18.h                */

tCanMsg Diccionario_CanOpen[] = {

    //  Indice                                           //Indice_Diccionario_TPO
    //------------------------------------------------------------------------------
    //  0x100x
    //------------------------------------------------------------------------------
    {OD_READ, 0x1008, 0x00 },            //Nombre_dispositivo
    {OD_READ, 0x1009, 0x00 },            //Hardware_dispositivo
    {OD_READ, 0x100A, 0x00 },            //Software_dispositivo
    //------------------------------------------------------------------------------
    //  0x101x
    //------------------------------------------------------------------------------
    {OD_READ, 0x1018, 0x00},            //Identity
    //------------------------------------------------------------------------------
    //  0x210x
    //------------------------------------------------------------------------------
    {OD_READ + OD_WRITE, 0x2100, 0x00}, //Module_Enable
    {OD_READ, 0x2101, 0x00},            //Module_Status
    {OD_READ, 0x2104, 0x00},            //Module_Temperature
    {OD_READ + OD_WRITE, 0x2105, 0x00}, //Uac_Input
    {OD_READ, 0x2106, 0x00},            //Iac_Input
    {OD_READ, 0x2107, 0x00},            //Udc_Out
    {OD_READ, 0x2108, 0x00},            //Idc_Out
    {OD_READ + OD_WRITE, 0x2109, 0x00}, //Udc_Out_Setpoint
    {OD_READ + OD_WRITE, 0x210A, 0x00}, //Idc_Out_Setpoint
    {OD_READ, 0x210D, 0x00},            //Udc_Bus
    {OD_READ, 0x210E, 0x00},            //I_Solar_Out
    //------------------------------------------------------------------------------
    //  0x212x
    //------------------------------------------------------------------------------
    {OD_READ, 0x2120, 0x00},            //Uac_Input_Average
    {OD_READ, 0x2121, 0x00},            //Uac_Input_L1
    {OD_READ, 0x2122, 0x00},            //Uac_Input_L2
    {OD_READ, 0x2123, 0x00},            //Uac_Input_L3
    {OD_READ, 0x2124, 0x00},            //Iac_Input_L1
    {OD_READ, 0x2125, 0x00},            //Iac_Input_L2
    {OD_READ, 0x2126, 0x00},            //Iac_Input_L3
    {OD_READ, 0x212A, 0x00},            //UBus
    {OD_READ, 0x212B, 0x00},            //UBus_OVP
    //------------------------------------------------------------------------------
    //  0x213x
    //------------------------------------------------------------------------------
    {OD_READ, 0x2130, 0x00},            //Warning_Status
    {OD_READ, 0x2132, 0x00},            //Error_Source
    {OD_READ, 0x2136, 0x00},            //Temp_Ambient
    {OD_READ, 0x2137, 0x00},            //Temp_Sec
    {OD_READ, 0x2138, 0x00},            //Temp_Prim
    {OD_READ, 0x2139, 0x00},            //Temp_Trafo
    //------------------------------------------------------------------------------
    //  0x21x
    //------------------------------------------------------------------------------
    {OD_READ, 0x2194, 0x00},            //Udc_Setpoint_Real
    {OD_READ, 0x21A4, 0x00},            //Idc_Setpoint_Real
    //------------------------------------------------------------------------------
    //  0x215x
    //------------------------------------------------------------------------------
    {OD_READ, 0x2150, 0x00},            //Switch_OFF_Reason
    //------------------------------------------------------------------------------
    //  0x2FFx
    //------------------------------------------------------------------------------
    {OD_READ + OD_WRITE, 0x2FF0, 0x00}, //Set_Node_ID
    {OD_READ + OD_WRITE, 0x2FFF, 0x00}, //Restart_Module

    //------------------------------------------------------------------------------
    //  0x9000  Used by ADC sensor PCB. Chademo Protocol
    //------------------------------------------------------------------------------
    {OD_WRITE, 0x9000, 0x00}, //Vo_Chademo. Voltage value read by ADC from Power Supply
    {OD_WRITE, 0x9000, 0x01}, //Io_Chademo. Current value read by ADC from Power Supply
    {OD_WRITE, 0x9000, 0x02}, //TempPos_Chademo. Positive Temperature value read by ADC
    {OD_WRITE, 0x9000, 0x03}, //TempNeg_Chademo. Negative Temperature value read by ADC

#ifdef CCS_Protocol
    //------------------------------------------------------------------------------
    //  0x9001  Used by ADC sensor PCB. CCS Protocol
    //------------------------------------------------------------------------------
    {OD_WRITE, 0x9001, 0x00}, //Vo_CCS. Voltage value read by ADC from Power Supply
    {OD_WRITE, 0x9001, 0x01}, //Io_CCS. Current value read by ADC from Power Supply
    {OD_WRITE, 0x9001, 0x02}, //TempPos_CCS. Positive Temperature value read by ADC
    {OD_WRITE, 0x9001, 0x03}, //TempNeg_CCS. Negative Temperature value read by ADC
#endif
    //------------------------------------------------------------------------------
    //  0x9002  Enable/Disable ADC sensor PCB.
    //------------------------------------------------------------------------------
     {OD_WRITE, 0x9002, 0x00},  //Enable/disable ADC measure and select protocol 
                                //CHADEMO or CCS
    //------------------------------------------------------------------------------
    //  0xFFFF
    //------------------------------------------------------------------------------
    {0xFF, 0xFFFF, 0xFF}, //FIN_Diccionario
};
/* FIN Diccionario  */
