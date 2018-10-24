/*
 * ComModule.c
 *
 *  Created on: 11 oct. 2018
 *      Author: dagaro
 */

/* USER CODE BEGIN Includes */
#include "ComModule.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* Private defines -----------------------------------------------------------*/
#define BitRate 500
#define MSG_DATA_LENGTH 8
#define NumMsg 10
#define PS_NODE_ID 0x630  //Power Supply Module 0
#define ADC_NODE_ID 0x603 //PC 1

#define ENABLE_ADC Set_CANOpenMsg_To_Tx()


FlagCom *ptr_StatusCom;
*ptr_StatusCom=0; //Clear all bits

/* USER CODE END PD */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FIFO FIFO_PowerSupplyTX;                 //FIFO Tx defined for Power Supply
FIFO FIFO_PowerSupplyRX;                 //FIFO Rx defined for Power Supply
FIFO FIFO_AdcTX;                         //FIFO Tx defined for Power Supply
FIFO FIFO_AdcRX;                         //FIFO Rx defined for Power Supply
FIFO FIFO_PcRX;                          //FIFO Tx defined for Industrial PC
FIFO FIFO_PcTX;                          //FIFO Rx defined for Industrial PC
tCANMsgObject sTX_CANOpenMsg;            //Can message objet for tx
tCANMsgObject sTXADC_CANOpenMsg;         //Can message objet for tx
tCANMsgObject sRXPowerSupply_CANOpenMsg; //Can message object for rx from PS module
tCANMsgObject sRXADC_CANOpenMsg;         //Can message object for rx from ADC module
volatile unsigned long g_bErrFlag = 0;   // A flag to indicate that some
// transmission error occurred.

typedef struct ObjectTx
{
    uint16_t OB;      //Last Object Sent with CAN peripheral
    uint16_t Node_ID; //Id of Node that has sent last Object
} LastObjectTx;

LastObjectTx PowerSupply_LastObjectTx = {0x00, 0x00};
/* USER CODE END PV */

/* USER CODE BEGIN NPV */
/* Non Private variables ---------------------------------------------------------*/
enum Indice_Diccionario_TPO OD_Index = FIN_Diccionario;
/* USER CODE END NPV */

/**
 * @brief Makes a new CANOpen FIFO with known values such as length and depth
 * 
 */
void Init_CANOpenMsgFIFO(void)
{

    Init_FIFO(&FIFO_PowerSupplyTX, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                                 // ID = 1 byte
                                                                 // MSG_DATA_LENGTH= 8 bytes
                                                                 // NumMsg = Depth of the stack
    Init_FIFO(&FIFO_PowerSupplyRX, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
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
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                              FIFO *ptr_MsgToTx, uint32_t DataToTx)
{

    uint32_t tmp = 0;
    uint16_t CANMsg[10]; //Temporary Array to store data to be sent
    uint16_t *ptrMsg;
    sEstadoFIFO status = PILA_OK; //Default value

    memset(CANMsg, 0x00, 10); //Set all array to zero for sanity

    ptrMsg = &CANMsg;

    if (ptrMsg == NULL) //For sanity, checking pointer
        return (0x00);  //NULL pointer. Error

    *(ptrMsg++) = PS_NODE_ID;                                         //0x630(default from manufacturer)
    *(ptrMsg++) = Diccionario_CanOpen[Idx].Modo_Acceso;               //Command Byte, Read or Write operation
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) & 0x00FF); //Object Dictionary Index
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) >> 8);     //Stored as little endian
    *(ptrMsg++) = ((Diccionario_CanOpen[Idx].SubIndice));             //Stored SubIndex

    tmp = DataToTx;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); // Data are saves as little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); // Data are saves little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); // Data are saves little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); // Data are savesas little endian

    ptrMsg = &CANMsg; //Restart pointer to first position of buffer
    memcpy((void *)(ptr_MsgToTx->New_Datos), (void *)(ptrMsg),
           MSG_DATA_LENGTH + 1); //Message in correct format
                                 //stored in one item of FIFO struct

    status = Encolar_FIFO(ptr_MsgToTx);

    if (status == PILA_LLENA)
        return (status);

    memset(CANMsg, 0x00, 10); //Reset array to zero for next time

    return (0x01); //All OK
}
/**
 * @brief Function for transmit message over CAN peripheral
 * 
 * @param MsgToTx 
 * @return sEstadoFIFO 
 */
sEstadoFIFO Transmit_CANOPenMsg(FIFO MsgToTx)
{
    sEstadoFIFO status = PILA_VACIA; //Variable set to a default value
    FIFO *ptr_MsgToTx;
    ptr_MsgToTx = &MsgToTx;

    if (MsgToTx.Estado_PILA == PILA_OK) //Are there messages to send?
    {
        do
        {
            status = Desencolar_FIFO(&MsgToTx);

            if ((status == PILA_RESET) || (status == PILA_LLENA)) //TODO: Check these conditions
                return (status);
            //|------------------FIFO.Datos_Recibidos-------------------|
            //|---------------------------------------------------------|
            //| Node_ID      | Modo_Acceso | Object Index | SI | B4-B7  |
            //|---------------------------------------------------------|
            //|CAN Struct    |              CAN Struct                  |
            //|item MsgID    |              CAN item pucMsgData         |
            //|---------------------------------------------------------|
            // First direction pointed by *ptr_MsgToTxDatos_Recibidos
            // is Node_ID, thus it is post-incremented to point to
            // Modo_Acceso FIFO item. Then the rest of CANOpen message
            // datas, included Modo_Acceso, are copied on item putcMsgData
            // of CAN structure by memcpy() function.
            sTX_CANOpenMsg.ui32MsgID = *(ptr_MsgToTx->Datos_Recibidos++);
            sTX_CANOpenMsg.ui32MsgIDMask = 0;
            sTX_CANOpenMsg.ui32Flags = 0;
            sTX_CANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
            memcpy((void *)sTX_CANOpenMsg.pucMsgData,
                   (void *)ptr_MsgToTx->Datos_Recibidos, MSG_DATA_LENGTH);
            CANMessageSet(CANB_BASE, 1, &sTX_CANOpenMsg, MSG_OBJ_TYPE_TX);
            DELAY_US(1000); // TODO: It may not be necessary, analyze. Maybe put here a TimeOut routine that wait
                            // for a pending response from Power Supply or ADC module, otherwise set an Error

        } while (MsgToTx.Msg_pendientes != 0);
    }
    else
        return (status);
}
/**
 * @brief 
 * 
 */
void Set_PowerSupplyMailbox(void)
{
    sRXPowerSupply_CANOpenMsg.ui32MsgID = PS_NODE_ID;
    sRXPowerSupply_CANOpenMsg.ui32MsgIDMask = 0x1FFFFFFF;
    sRXPowerSupply_CANOpenMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sRXPowerSupply_CANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
    sRXPowerSupply_CANOpenMsg.pucMsgData = &FIFO_PowerSupplyRX.New_Datos;
    CANMessageSet(CANB_BASE, CAN_OBJ_ID_PS, &sRXPowerSupply_CANOpenMsg, MSG_OBJ_TYPE_RX);
}
/**
 * @brief 
 * 
 */
void Set_ADCMailbox(void)
{
    sRXADC_CANOpenMsg.ui32MsgID = ADC_NODE_ID;
    sRXADC_CANOpenMsg.ui32MsgIDMask = 0x1FFFFFFF;
    sRXADC_CANOpenMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sRXADC_CANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
    sRXADC_CANOpenMsg.pucMsgData = &FIFO_AdcRX.New_Datos;
    CANMessageSet(CANB_BASE, CAN_OBJ_ID_ADC, &sRXADC_CANOpenMsg, MSG_OBJ_TYPE_RX);
}

