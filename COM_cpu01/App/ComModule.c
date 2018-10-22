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
FIFO PowerSupplyMsgTX;                   //FIFO Tx defined for Power Supply
FIFO PowerSupplyMsgRX;                   //FIFO Rx defined for Power Supply
FIFO PCMsgRX;                            //FIFO Tx defined for Industrial PC
FIFO PCMsgTX;                            //FIFO Rx defined for Industrial PC
tCANMsgObject sTX_CANOpenMsg;            //Can message objet for tx
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
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx, FIFO *ptr_MsgToTx)
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
    tmp = Diccionario_CanOpen[Idx].Buf;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Data are saves as little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Data are saves little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Data are saves little endian
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF); //Buffer Data. Data are savesas little endian

    ptrMsg = &CANMsg;                                                                //Restart pointer to first position of buffer
    memcpy((void *)(ptr_MsgToTx->New_Datos), (void *)(ptrMsg), MSG_DATA_LENGTH + 1); //Message in correct format
                                                                                     //stored in one item of FIFO struct
    //status = Encolar_FIFO(&ptr_MsgToTx);
    status = Encolar_FIFO(&PowerSupplyMsgTX);

    if (status == PILA_LLENA)
        return (status);

    memset(CANMsg, 0x00, 10); //Reset array to zero for next time

    return (0x01); //All OK
}
/**
 * @brief Function transmit message over CAN peripheral
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
            memcpy((void *)sTX_CANOpenMsg.pucMsgData, (void *)ptr_MsgToTx->Datos_Recibidos, MSG_DATA_LENGTH);
            CANMessageSet(CANB_BASE, 1, &sTX_CANOpenMsg, MSG_OBJ_TYPE_TX);
            DELAY_US(1000); // TODO: It may not be necessary, analyze. Maybe put here a TimeOut routine that wait
                            // for a pending response from Power Supply or ADC module, otherwise set an Error

        } while (MsgToTx.Msg_pendientes != 0);
    }
    else
        return (status);
}

/**
 * @brief Get message received from ADC or Power supply modules
 * 
 * @param ptr_MsgFromRx 
 */
// void Get_CANopenMsg_From_Rx(FIFO *ptr_MsgFromRx)
// {
// }
/**
 * @brief From TI examples. This function is the interrupt handler for the CAN
 *        peripheral. It checks for the cause of the interrupt
 * 
 * @return interrupt CANIntHandler 
 */
interrupt void CANIntHandler(void)
{
    unsigned long ulStatus;

    // Read the CAN interrupt status to find the cause of the interrupt
    ulStatus = CANIntStatus(CANB_BASE, CAN_INT_STS_CAUSE);

    // If the cause is a controller status interrupt, then get the status
    if (ulStatus == CAN_INT_INT0ID_STATUS)
    {
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        ulStatus = CANStatusGet(CANB_BASE, CAN_STS_CONTROL);
        //Check to see if an error occurred.
        if (((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 7) &&
            ((ulStatus & ~(CAN_ES_TXOK | CAN_ES_RXOK)) != 0))
        {
            g_bErrFlag = 1; // Set a flag to indicate some errors may have occurred.
        }
    }
    // Check if the cause is message object 2, which what we are using for
    // receiving messages from Power Supply module.
    else if (ulStatus == 2)
    {

        CANMessageGet(CANB_BASE, 2, &sRXPowerSupply_CANOpenMsg, true);
        CANIntClear(CANB_BASE, 2); // Clear the message object interrupt.
        g_bErrFlag = 0;            //Since the message was sent, clear any error flags.
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }

    CANGlobalIntClear(CANB_BASE, CAN_GLB_INT_CANINT0);
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
/**
 * @brief From TI example. re-map ISR for CAN peripheral
 *        within this file 
 * 
 */
void Set_CANIntHandler(void)
{
    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
//    DINT;
//
//    //
//    // Initialize the PIE control registers to their default state.
//    // The default state is all PIE interrupts disabled and flags
//    // are cleared.
//    // This function is found in the F2837xD_PieCtrl.c file.
//    //
//    InitPieCtrl();
//
//    //
//    // Disable CPU interrupts and clear all CPU interrupt flags:
//    //
//    IER = 0x0000;
//    IFR = 0x0000;
//
//    //
//    // Initialize the PIE vector table with pointers to the shell Interrupt
//    // Service Routines (ISR).
//    // This will populate the entire table, even if the interrupt
//    // is not used in this example.  This is useful for debug purposes.
//    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
//    // This function is found in F2837xD_PieVect.c.
//    //
//    InitPieVectTable();
//
//    //
//    // Interrupts that are used in this example are re-mapped to
//    // ISR functions found within this file.
//    // Register interrupt handler in RAM vector table
//    //
//    EALLOW;
//    PieVectTable.CANB0_INT = CANIntHandler;
//    EDIS;
//
//    //
//    // Enable the CAN interrupt on the processor (PIE).
//    //
//    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
//    IER |= 0x0100; // M_INT9
//    EINT;

    sRXPowerSupply_CANOpenMsg.ui32MsgID = 1;                     // CAN message ID - use 1
    sRXPowerSupply_CANOpenMsg.ui32MsgIDMask = 0;                 // no mask needed for TX
    sRXPowerSupply_CANOpenMsg.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER; // enable interrupt on RX
    sRXPowerSupply_CANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;      // size of message is 8
    sRXPowerSupply_CANOpenMsg.pucMsgData = &PowerSupplyMsgRX.New_Datos;          // ptr to message content

    CANMessageSet(CANB_BASE, 2, &sRXPowerSupply_CANOpenMsg, MSG_OBJ_TYPE_RX);
}
