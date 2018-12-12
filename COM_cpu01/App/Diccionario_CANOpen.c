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
    {OD_READ_4BYTES, 0x1008, 0x04 },            //Nombre_dispositivo
    {OD_READ_4BYTES, 0x1009, 0x04 },            //Hardware_dispositivo
    {OD_READ_4BYTES, 0x100A, 0x04 },            //Software_dispositivo
    //------------------------------------------------------------------------------
    //  0x101x
    //------------------------------------------------------------------------------
    {OD_READ, 0x1018, 0x04},            //Identity
    //------------------------------------------------------------------------------
    //  0x210x
    //------------------------------------------------------------------------------
    {OD_READ + OD_WRITE, 0x2100, 0x00}, //Module_Enable
    {OD_READ, 0x2101, 0x00},            //Module_Status
    {OD_READ, 0x2104, 0x00},            //Module_Temperature
    {OD_READ_2BYTES + OD_WRITE_2BYTES, 0x2105, 0x00}, //Uac_Input
    {OD_READ, 0x2106, 0x00},            //Iac_Input
    {OD_READ_2BYTES, 0x2107, 0x00},            //Udc_Out
    {OD_READ_2BYTES, 0x2108, 0x00},            //Idc_Out
    {OD_READ_2BYTES + OD_WRITE_2BYTES, 0x2109, 0x00}, //Udc_Out_Setpoint
    {OD_READ_2BYTES + OD_WRITE_2BYTES, 0x210A, 0x00}, //Idc_Out_Setpoint
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
    {OD_WRITE_4BYTES, 0x9000, 0x00}, //Vo_Chademo. Voltage value read by ADC from Power Supply
    {OD_WRITE_4BYTES, 0x9000, 0x01}, //Io_Chademo. Current value read by ADC from Power Supply
    {OD_WRITE_4BYTES, 0x9000, 0x02}, //TempPos_Chademo. Positive Temperature value read by ADC
    {OD_WRITE_4BYTES, 0x9000, 0x03}, //TempNeg_Chademo. Negative Temperature value read by ADC

#ifdef CCS_Protocol
    //------------------------------------------------------------------------------
    //  0x9001  Used by ADC sensor PCB. CCS Protocol
    //------------------------------------------------------------------------------
    {OD_WRITE_4BYTES, 0x9001, 0x00}, //Vo_CCS. Voltage value read by ADC from Power Supply
    {OD_WRITE_4BYTES, 0x9001, 0x01}, //Io_CCS. Current value read by ADC from Power Supply
    {OD_WRITE_4BYTES, 0x9001, 0x02}, //TempPos_CCS. Positive Temperature value read by ADC
    {OD_WRITE_4BYTES, 0x9001, 0x03}, //TempNeg_CCS. Negative Temperature value read by ADC
#endif
    //------------------------------------------------------------------------------
    //  0x9002  Enable/Disable ADC sensor PCB.
    //------------------------------------------------------------------------------
    {OD_WRITE, 0x9002, 0x00},  //Enable/disable ADC measurement and select protocol 
                                //CHADEMO or CCS
    //------------------------------------------------------------------------------
    //  0x9003  Used to check communication with ADC sensor PCB.
    //------------------------------------------------------------------------------                            
    
    {OD_READ, 0x9003, 0x00 },   //Used for check communication with ADC PCB  
    //------------------------------------------------------------------------------
    //  0xFFFF
    //------------------------------------------------------------------------------
    {0xFF, 0xFFFF, 0xFF}, //FIN_Diccionario
};
/* FIN Diccionario  */
// Esctructura para sacar la inforamcion del diccionario del CANopen
tCanMsg *D2;

/**
*******************************************************************************
\fn                     Consulta_Diccionario
\brief                  Funcion que indicamos el indice de la taabla, este indice
 esta definido en el *.h, y devolvemos la estructura del del protocolo
  comunicacion del bus CANOpen consultada.

 \param[in]         indice: El indice de la tabla de las estructuras del protocolo
  comunicacion del bus CANOpen
 \return                    tCanMsg: La estructura del CANOpen solicitado

******************************************************************************/
tCanMsg Consulta_Diccionario (uint16_t indice){
     *D2 = Diccionario_CanOpen[indice];
     return *D2;
 }//FIN Consulta_Diccionario

//uint16_t Modificar_Mem_Diccionario (uint16_t indice, uint32_t NuevoDato){
//
//     //Obtenemos la direccion de memoria SRAM
//     D2 = &Diccionario_CanOpen[indice];
//
//     if (D2->Modo_Acceso > OD_READ){
//         D2->Buf = NuevoDato;
//
//         return 1;
//     }
//     else{
//         return 0;
//     }
// }
//
//uint32_t Consulta_Mem_Diccionario (uint16_t indice){
//
//     D2 = &Diccionario_CanOpen[indice];
//
//     //Si la direccion de memoria es solo lectura en el BUF tiene la informacion, pero si
//// permite escritura en el BUF esta la direccion SRAM donde esta la direccion
//     if (D2->Modo_Acceso >= OD_READ){
//         return  D2->Buf;
//     }
// }

uint16_t Econtrar_Indice_Diccionario (uint16_t ID, char subID){
    uint16_t indice = 0x00;

    do {
        //Obtenemos la direccion de memoria SRAM
        D2 = &Diccionario_CanOpen[indice];

       if (ID == D2->ID && subID == D2->SubIndice)
           break;
       else
       indice ++;
    } while (0xFFFF != D2->ID);

   if (0xFFFF == D2->ID){
       return 0xFFFF; //Si ha llegado al final del diccionario
   }

   //return (indice - 0x01);
   return indice;
}//Econtrar_Indice_Diccionario

/**
*******************************************************************************
\fn                         datos_char_to_int

\brief                  Funcion que transforma el array recibido por CAN en enteros.
 Depediendo de cuantos datos se reciba, segun "Command Byte", se trata mas bytes
o menos

\param[in]          Datos_Convertir -> Array de char que se recibe por el bus CAN
\return                 uintt32_t -> datos convertidos en enteros de 32 bits
******************************************************************************/
uint32_t datos_char_to_int (uint16_t Datos_Convertir[8])
{
     uint32_t itemp_32=0;

    switch (Datos_Convertir[0])
        {
        case OD_READ:
        case OD_WRITE:
        case OD_READ_4BYTES:
            {
                itemp_32 = 0x000000FF & (uint32_t)(Datos_Convertir[4]) ;
                itemp_32 = 0x0000FFFF & (((uint32_t)(Datos_Convertir[5]) << 8) + itemp_32);
                itemp_32 = 0x00FFFFFF & (((uint32_t)(Datos_Convertir[6]) << 16) + itemp_32);
                itemp_32 = 0xFFFFFFFF & (((uint32_t)(Datos_Convertir[7]) << 24) + itemp_32);
                break;
            }// FIN case 4 Bytes

        case OD_READ_2BYTES:
        case OD_WRITE_2BYTES:
            {
                itemp_32 = 0x000000FF & (uint32_t)(Datos_Convertir[4]) ;
                itemp_32 = 0x0000FFFF & (((uint32_t)(Datos_Convertir[5]) << 8) + itemp_32);
                break;
            }// FIN case 2 Bytes

        case OD_READ_1BYTES:
        case OD_WRITE_1BYTES:
            {
            itemp_32 = 0x000000FF & (uint32_t)(Datos_Convertir[4]) ;
            break;
            }// FIN case 1 Bytes

        default :
            itemp_32 =  0xFFFFFFFF ;

        }//FIN switch

        return itemp_32;
} //FIN datos char a int
/***** FIN ARCHIVO *****/
