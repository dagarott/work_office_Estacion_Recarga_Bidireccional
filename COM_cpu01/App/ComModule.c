/**
 * \file ComModule.c
 * \brief
 * \details
 * \author dagaro
 * \version 1.0
 * \date 2018
 * \pre
 * \bug
 * \warning
 * \copyright ITE
 */

/* USER CODE BEGIN Includes */
#include "ComModule.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
/* Private defines -----------------------------------------------------------*/
#define BitRate 500
#define MSG_DATA_LENGTH 8
#define NumMsg 10

/* USER CODE END PD */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FIFO FIFO_PowerSupplyTx; //FIFO Tx defined for Power Supply
//FIFO FIFO_PowerSupplyRX;               //FIFO Rx defined for Power Supply
FIFO FIFO_AdcTx; //FIFO Tx defined for Adc
//FIFO FIFO_AdcRX;                       //FIFO Rx defined for ADC
FIFO FIFO_CanRx;              //Unique CAN FIFO defined for store CAN
                              //messages from Power Supply and ADC
FIFO FIFO_PcRx;               //FIFO Tx defined for Industrial PC
FIFO FIFO_PcTx;               //FIFO Rx defined for Industrial PC
tCANMsgObject sTX_CANOpenMsg; //Can message objet for tx
//tCANMsgObject sTXADC_CANOpenMsg;        //Can message objet for tx
tCANMsgObject sMailboxOneCANOpenMsg; //Can message object for rx
tCANMsgObject sMailboxTwoCANOpenMsg; //Can message object for rx
unsigned char rxPsMsgData[8];
unsigned char rxAdcMsgData[8];
volatile uint32_t ulTimeOutCANRx;

typedef struct sObjectTx
{
    uint16_t OB;      //Last Object Sent with CAN peripheral
    uint16_t Node_ID; //Id of Node that has sent last Object
} LastObjectTx_t;

LastObjectTx_t PowerSupply_LastObjectTx = {0x00, 0x00};

AdcValues_t AdcValuesSaved = {0, 0.0, 0, 0.0, 0, 0, 0};

/* USER CODE END PV */

/* USER CODE BEGIN NPV */
/* Non Private variables ---------------------------------------------------------*/
enum Indice_Diccionario_TPO OD_Index = FIN_Diccionario;
FlagsCom_t StatusCom = {0x00};
FlagsError_t StatusErrors = {0x00};
/* USER CODE END NPV */

/*  BEGIN MISCELANIOUS FUNCTIONS */

/**
 * @brief Makes a new CANOpen FIFO with known values such as length and depth
 * 
 */
void Init_CANOpenMsgFIFOs(void)
{

    Init_FIFO(&FIFO_PowerSupplyTx, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                                 // ID = 1 byte
                                                                 // MSG_DATA_LENGTH= 8 bytes
                                                                 // NumMsg = Depth of the stack
    Init_FIFO(&FIFO_AdcTx, MSG_DATA_LENGTH + 1, NumMsg);         // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                                 // ID = 1 byte
                                                                 // MSG_DATA_LENGTH= 8 bytes
                                                                 // NumMsg = Depth of the stack
    Init_FIFO(&FIFO_CanRx, MSG_DATA_LENGTH + 1, NumMsg);         // Common reception FIFO for ADC and Power Supply
                                                                 // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                                 // ID = 1 byte
                                                                 // MSG_DATA_LENGTH= 8 bytes
                                                                 // NumMsg = Depth of the stack
}
/**
 * @brief  Build a message in the correct CANOpen Protocol way and stack it in FIFO,
 *         waiting to be transmitted by CAN peripheral
 * 
 * @param Indice_Diccionario_TPO 
 * @param ptr_MsgToTx 
 * @param DataToTx 
 * @return uint16_t 
 */
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                              FIFO *ptr_MsgToTx, uint32_t DataToTx,
                              uint16_t Idx_Node)
{

    uint32_t tmp = 0;
    uint16_t CANMsg[10]; //Temporary Array to store data to be sent
    uint16_t *ptrMsg;
    sEstadoFIFO status = PILA_OK; //Default value

    memset(CANMsg, 0x00, 10); //Set all array to zero for sanity

    ptrMsg = CANMsg;

    if (ptrMsg == NULL) //For sanity, checking pointer
        return (0x00);  //NULL pointer. Error

    *(ptrMsg++) = Idx_Node;
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

    ptrMsg = CANMsg; //Restart pointer to first position of buffer
    memcpy((void *)(ptr_MsgToTx->New_Datos), (void *)(ptrMsg),
           MSG_DATA_LENGTH + 1); //Message in correct format
                                 //stored in one item of FIFO struct

    status = Encolar_FIFO(ptr_MsgToTx);

    if (status == PILA_LLENA)
        return (0x01);

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
            //TODO: Add condition to check that, "Not send anew CAN frame
            //if no exit a previously succes tx"
            CANMessageSet(CANA_BASE, 1, &sTX_CANOpenMsg, MSG_OBJ_TYPE_TX);
            DELAY_US(1000);                                    //little delay between CAN tx
            StatusCom.StatusFlags.Flags.TransmittedCanMsg = 1; //Start CAN tx timeOut exception
        } while (MsgToTx.Msg_pendientes != 0);
    }
    else
        return (status);
}
/**
 * @brief Create mailbox for Power Supply
 * 
 */
void Set_MailboxOne(void)
{

    sMailboxOneCANOpenMsg.ui32MsgID = TSDO + PS_NODE_ID;
    sMailboxOneCANOpenMsg.ui32MsgIDMask = 0x1FFFFFFF;
    sMailboxOneCANOpenMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMailboxOneCANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
    sMailboxOneCANOpenMsg.pucMsgData = rxPsMsgData;
    CANMessageSet(CANA_BASE, MAILBOX_ONE, &sMailboxOneCANOpenMsg,
                  MSG_OBJ_TYPE_RX);
}
/**
 * @brief Create mailbox for ADC
 * 
 */
void Set_MailboxTwo(void)
{

    sMailboxTwoCANOpenMsg.ui32MsgID = RSDO + COM_NODE_ID;
    sMailboxTwoCANOpenMsg.ui32MsgIDMask = 0x1FFFFFFF;
    sMailboxTwoCANOpenMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sMailboxTwoCANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
    sMailboxTwoCANOpenMsg.pucMsgData = rxAdcMsgData;
    CANMessageSet(CANA_BASE, MAILBOX_TWO, &sMailboxTwoCANOpenMsg,
                  MSG_OBJ_TYPE_RX);
}

/*
 uint16_t Get_ADCValues(AdcValues_t ptr_*ADCValues)
 {


 }*/

/*  END MISCELANIOUS FUNCTIONS */

/*  BEGIN CONTROL LOGIC FUNCTIONS */
/*
/**
 * @brief Simple task scheduler
 * 
 */
void AnalyzeAdcCanMsg(void)
{
    if ((StatusCom.StatusFlags.Flags.TransmittedCanMsg == true) &&
        (StatusCom.StatusFlags.Flags.AdcDataAvailable != true))
    {
        ulTimeOutCANRx++;
        if (ulTimeOutCANRx == TIMEOUT_CAN_RX)
        {
            StatusCom.StatusFlags.Flags.ErrorCom = true;
            ulTimeOutCANRx = 0;
        }
    }
    if (StatusCom.StatusFlags.Flags.AdcDataAvailable == true)
    {
        uint16_t SubIndex = 0; //Temporal variable to save SubIndex
        uint16_t ObjIndex = 0; //Temporal variable to save Object Index
        uint16_t AccessMode = 0;
        uint16_t DataSaved[8];        //Temporal array where store data received from CAN
        uint32_t AdcValue = 0;        //
        uint16_t DictionaryIndex = 0; //Temporal variable used for check ObjectIndex
                                      //and AccesCmd
        memset(DataSaved, 0x00, 8);   //Reset array to zero

        if (FIFO_CanRx.Estado_PILA != PILA_VACIA) //Check just in case
        {
            Desencolar_FIFO(&FIFO_CanRx);
            //Move data from FIFO to variable DataSaved for post-processing
            memcpy((void *)DataSaved, (void *)FIFO_CanRx.Datos_FIFO, sizeof(DataSaved));

            AccessMode = (uint16_t)DataSaved[0];
            ObjIndex = (uint16_t)DataSaved[1] + (uint16_t)(DataSaved[2] << 8);
            SubIndex = (uint16_t)DataSaved[3];
            AdcValue = datos_char_to_int(DataSaved);
            // AdcValue = (uint32_t)DataSaved[7] << 24;
            // AdcValue += (uint32_t)DataSaved[6] << 16;
            // AdcValue += (uint32_t)DataSaved[5] << 8;
            // AdcValue += (uint32_t)DataSaved[4];

            DictionaryIndex = Econtrar_Indice_Diccionario(ObjIndex, SubIndex);

            if (DictionaryIndex == 0xFFFF)
            {
                StatusErrors.StatusFlags.Flags.CmdNotExist=true;
                
            }



                switch (ObjIndex)
                {
                case 0x9000: // Get Voltage/Current values from ADC

                    if ((AccessMode == OD_WRITE_4BYTES) || (AccessMode == OD_READ + OD_WRITE))
                    {
                        if (SubIndex == 0x0000) //Vout from ADC
                        {
                            AdcValuesSaved.VoltageValue = AdcValue;                   //Raw value for Chademo logic
                            AdcValuesSaved.floatVolatageValue = (float)AdcValue / 10; //0.1 A/bit
                        }
                        else if (SubIndex == 0x0001) //Iout from ADC
                        {
                            AdcValuesSaved.CurrentValue = AdcValue; //Raw value for Chademo logic
                            //Get sign of the Current Value
                            if ((AdcValuesSaved.CurrentValue & 0x80000000) == 0x80000000)
                            {
                                AdcValuesSaved.NegativeCurrentValue = true; //Negative number
                                // If it is negative, make it positive by inverting the bits
                                // and adding one.
                                AdcValue = ~AdcValue; //Bitwise negation of all bits
                                AdcValue += 0x01;
                            }

                            AdcValuesSaved.floatCurrentValue = (float)(AdcValue / 10); //0.1 A/bit
                        }
                    }
                    else if (AccessMode == OD_ERROR)
                        StatusCom.StatusFlags.Flags.ErrorCom = true;
                    break;
                }
        }
        else
        {
            StatusCom.StatusFlags.Flags.ErrorAdcCom = true;
        }
    }
}
/*  END CONTROL LOGIC FUNCTIONS */
