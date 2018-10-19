/*
 * ComModule.c
 *
 *  Created on: 11 oct. 2018
 *      Author: dagaro
 */

/* USER CODE BEGIN Includes */
#include "ComModule.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FIFO PowerSupplyMsgTX; //FIFO Tx defined for Power Supply
FIFO PowerSupplyMsgRX; //FIFO Rx defined for Power Supply
FIFO PCMsgRX;          //FIFO Tx defined for Industrial PC
FIFO PCMsgTX;          //FIFO Rx defined for Industrial PC
tCANMsgObject sTXCANOpenMsg;
/* USER CODE END PV */

/* USER CODE BEGIN NPV */
/* Non Private variables ---------------------------------------------------------*/
enum Indice_Diccionario_TPO OD_Index = FIN_Diccionario;
/* USER CODE END NPV */

/**
 * @brief Make CANOpen FIFO with known values as length and depth
 * 
 */
void Init_CANOpenMsgFIFO(void)
{

    Init_FIFO(&PowerSupplyMsgTX, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                               // ID = 1 byte
                                                               // MSG_DATA_LENGTH= 8 bytes
                                                               // NumMsg = Depth of the stack
    Init_FIFO(&PowerSupplyMsgRX, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                               // ID = 1 byte
                                                               // MSG_DATA_LENGTH= 8 bytes
                                                               // NumMsg = Depth of the stack
}
/**
 * @brief  Build a message in the correct CANOpen Protocol way and stack it in FIFO,
 *         waiting to be transmitted by CAN peripheral
 * @param Indice_Diccionario_TPO 
 * @param ptrMsg 
 * @param ptrFIFO 
 * @return unsigned char 
 */
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx)
{

    uint32_t tmp = 0;
    uint16_t CANMsg[10];
    uint16_t *ptrMsg;
    sEstadoFIFO status = PILA_OK;

    memset(CANMsg, 0x00, 10); //Set array to zero for first iteration

    ptrMsg = &CANMsg;

    if (ptrMsg == NULL) //For sanity, checking pointer
        return (0x00);  //NULL pointer. Error

    *(ptrMsg++) = PS_NODE_ID;           //0x630(default from manufacturer)
    *(ptrMsg++) = Diccionario_CanOpen[Idx].Modo_Acceso;               //Command Byte, Read or Write operation 
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) & 0x00FF); //Object Dictionary Index
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) >> 8);     //Stored as little endian
    *(ptrMsg++) = ((Diccionario_CanOpen[Idx].SubIndice));             //Stored SunIndex
    tmp = Diccionario_CanOpen[Idx].Buf;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data.Stored as little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Stored as little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Stored as little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Stored as little endian

    ptrMsg = &CANMsg;

    memcpy((void *)(PowerSupplyMsgTX.New_Datos), (void *)(ptrMsg), MSG_DATA_LENGTH+1); //Message in correct format
                                                                       //stored in one item of FIFO struct
    status = Encolar_FIFO(&PowerSupplyMsgTX);                          //Finally CAN message is queued on the stack

    if (status == PILA_LLENA)
        return (0x00);

    memset(CANMsg, 0x00, 10); //Reset array to zero for the next time

    return (0x01); //All OK
}

uint16_t Transmit_CANOPenMsg(void)
{
    if (Desencolar_FIFO(&PowerSupplyMsgTX) == PILA_OK) //Are there messages to send?
    {     
        sTXCANOpenMsg.ui32MsgID = *(PowerSupplyMsgTX.Datos_Recibidos++);    //Node_ID , default 0x630
        sTXCANOpenMsg.ui32MsgIDMask = 0;
        sTXCANOpenMsg.ui32Flags = 0;
        sTXCANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
        memcpy((void *)sTXCANOpenMsg.pucMsgData, (void *)PowerSupplyMsgTX.Datos_Recibidos, MSG_DATA_LENGTH);

        CANMessageSet(CANB_BASE, 1, &sTXCANOpenMsg, MSG_OBJ_TYPE_TX);
    }
}