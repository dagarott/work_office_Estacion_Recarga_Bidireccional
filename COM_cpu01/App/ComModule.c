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
#define NumMsg 20

/* USER CODE END PD */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void AnalyzeCanMsg(void);
uint16_t Set_CANOpenErrorMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                                   FIFO *ptr_MsgToTx, uint32_t DataToTx,
                                   uint16_t Idx_Node);

void TimeOutRxCanMsg(void);

/* USER CODE END PFP */

/* USER CODE BEGIN PV */
/* Private variables---------------------------------------------------------*/
FIFO FIFO_CanTx; //Unique CAN FIFO defined for store CAN tx
FIFO FIFO_CanRx; //Unique CAN FIFO defined for store CAN rx
FIFO FIFO_PcRx;  //FIFO Tx defined for Industrial PC
FIFO FIFO_PcTx;  //FIFO Rx defined for Industrial PC

tCANMsgObject sTX_CANOpenMsg;        //Can message objet for tx
tCANMsgObject sMailboxOneCANOpenMsg; //Can message object for rx
tCANMsgObject sMailboxTwoCANOpenMsg; //Can message object for rx

unsigned char rxPsMsgData[8] = {0};  //Array to store data from CAN object
unsigned char rxAdcMsgData[8] = {0}; //Array to store data from CAN object

volatile uint32_t ulTimeOutCANRx; //Variable used by function TimeOut
uint16_t Count1ms = 0;            //Variable used by scheduler
uint16_t Count10ms = 0;           //Variable used by scheduler
uint16_t Count20ms = 0;           //Variable used by scheduler
uint16_t Count50ms = 0;           //Variable used by scheduler
uint16_t Count100ms = 0;          //Variable used by scheduler
uint16_t Count200ms = 0;          //Variable used by scheduler
uint16_t Count500ms = 0;          //Variable used by scheduler
uint16_t Count1s = 0;             //Variable used by scheduler
uint16_t Count2s = 0;             //Variable used by scheduler
uint16_t Count5s = 0;             //Variable used by scheduler

typedef struct sObjectTx
{
    uint16_t OB;      //Last Object Sent with CAN peripheral
    uint16_t Node_ID; //Id of Node that has sent last Object
} LastObjectTx_t;

LastObjectTx_t PowerSupply_LastObjectTx = {0x00, 0x00};

AdcValues_t AdcValuesSaved = {0, 0.0, 0, 0.0, 0, 0, 0, 0};

PowerSupplyValues_t PowerSupplyValues = {" ", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint16_t CurrentProcess = 0; //

/* USER CODE END PV */

/* USER CODE BEGIN NPV */
/* Non Private variables ---------------------------------------------------------*/
enum Indice_Diccionario_TPO OD_Index = FIN_Diccionario;
FlagsCom_t StatusCom = {0x00};
FlagsError_t StatusErrors = {0x00};
/* USER CODE END NPV */

/*-----------------------------------BEGIN MISCELANIOUS FUNCTIONS--------------------------------------*/

/**
 * @brief Makes a new CANOpen FIFO with known values such as length and depth
 * 
 */
void Init_CANOpenMsgFIFOs(void)
{

    //Init_FIFO(&FIFO_PowerSupplyTx, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
    // ID = 1 byte
    // MSG_DATA_LENGTH= 8 bytes
    // NumMsg = Depth of the stack
    Init_FIFO(&FIFO_CanTx, MSG_DATA_LENGTH + 1, NumMsg); // NumWords = ID + MSG_DATA_LENGTH = 9  bytes
                                                         // ID = 1 byte
                                                         // MSG_DATA_LENGTH= 8 bytes
                                                         // NumMsg = Depth of the stack
    Init_FIFO(&FIFO_CanRx, MSG_DATA_LENGTH + 1, NumMsg); // Common reception FIFO for ADC and Power Supply
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
 * @param Access Mode     --> Values could be (0)not apply/write_xbytes/read_xbytres
 *                        There are objects with both mode set, read_xbytes+write_xbytes,
 *                        in dictionary acces mode item, then in order to set correct CAN
 *                        frame is necessary to select one acces mode.
 *                        In the other hand, we also have object with only one mode, then 
 *                        reading directly from dictionary acces mode item it is enough
 * @return uint16_t 
 */
uint16_t Set_CANOpenMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
                              FIFO *ptr_MsgToTx, uint32_t DataToTx,
                              uint16_t Idx_Node, uint8_t AccesMode)
{

    int32_t tmp = 0;
    uint16_t CANMsg[10]; //Temporary Array to store data to be sent
    uint16_t *ptrMsg;
    sEstadoFIFO status = PILA_OK; //Default value

    memset(CANMsg, 0x00, 10); //Set all array to zero for sanity

    ptrMsg = CANMsg;

    if (ptrMsg == NULL) //For sanity, checking pointer
        return (0x00);  //NULL pointer. Error

    *(ptrMsg++) = Idx_Node;

    if (AccesMode == 0)
    {
        *(ptrMsg++) = Diccionario_CanOpen[Idx].Modo_Acceso; //Command Byte (CD), Read,Write operation
    }
    else
    {
        *(ptrMsg++) = AccesMode; //Command Byte (CD) Read/write operation
    }

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
 * @brief
 * 
 * @param Indice_Diccionario_TPO 
 * @param ptr_MsgToTx 
 * @param DataToTx 
 * @param Idx_Node 
 * @return uint16_t 
 */
uint16_t Set_CANOpenErrorMsg_To_Tx(enum Indice_Diccionario_TPO Idx,
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
    *(ptrMsg++) = OD_ERROR;                                           //Command Byte (CD), OR_ERROR=0x80
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) & 0x00FF); //Object Dictionary Index stored as
    *(ptrMsg++) = (uint16_t)((Diccionario_CanOpen[Idx].ID) >> 8);     //little endian
    *(ptrMsg++) = ((Diccionario_CanOpen[Idx].SubIndice));             //Stored SubIndex

    // Data are saves as little endian
    tmp = DataToTx;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF);
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF);
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF);
    tmp = tmp >> 8;
    *(ptrMsg++) = (uint16_t)(tmp & 0x00FF);

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
    sEstadoFIFO status = PILA_OK; //Variable set to a default value
    //FIFO *ptr_MsgToTx;
    //ptr_MsgToTx = &MsgToTx;

    //if (MsgToTx.Estado_PILA != PILA_VACIA) //Are there messages to send?
    //{
    //do
    //{
    //status = Desencolar_FIFO(&MsgToTx);

    //if ((status == PILA_RESET) || (status == PILA_LLENA)) //TODO: Check these conditions
    //    return (status);
    //|------------------FIFO.Datos_Recibidos-------------------|
    //|---------------------------------------------------------|
    //| Node_ID      | Modo_Acceso | Object Index | SI | B4-B7  |
    //|---------------------------------------------------------|
    //|CAN Struct    |              CAN Struct                  |
    //|item MsgID    |              CAN item pucMsgData         |
    //|---------------------------------------------------------|
    // First direction pointed by *ptr_MsgToTx->Datos_Recibidos
    // is Node_ID, thus it is post-incremented to point to
    // Modo_Acceso FIFO item. Then the rest of CANOpen message
    // datas, included Modo_Acceso, are copied on item putcMsgData
    // of CAN structure by memcpy() function.
    //sTX_CANOpenMsg.ui32MsgID = *(ptr_MsgToTx->Datos_Recibidos)++;
    sTX_CANOpenMsg.ui32MsgID = MsgToTx.Datos_Recibidos[0];
    sTX_CANOpenMsg.ui32MsgIDMask = 0;
    sTX_CANOpenMsg.ui32Flags = 0;
    sTX_CANOpenMsg.ui32MsgLen = MSG_DATA_LENGTH;
    memcpy(sTX_CANOpenMsg.pucMsgData, MsgToTx.Datos_Recibidos + 1,
           MSG_DATA_LENGTH);
    //TODO: Add condition to check that, "Not send a new CAN frame
    //if no exit a previously succes tx"
    CANMessageSet(CANA_BASE, 1, &sTX_CANOpenMsg, MSG_OBJ_TYPE_TX);
    while (CanaRegs.CAN_ES.bit.TxOk != 0x01)
    {
        //CanaRegs.CAN_ES.bit.TxOk
    }; //Wait until CAN tx finish
        //DELAY_US(6000);
        //Start CAN tx timeOut exception
    StatusCom.StatusFlags.Flags.TransmittedCanMsg = 1;
    //TODO: Check if we shall put a delay() instead of while (...)

    // } while (MsgToTx.Msg_pendientes != 0);
    //}
    //else
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

/**
 * @brief Enable ADC PCB sending CAN message 
 * 
 */
uint16_t InitAdc(void)
{
    static uint16_t stateInitAdc = 0;
    uint16_t status = 0x01; //By default all ok
    enum OD_Index;

    switch (stateInitAdc)
    {
    case 0:
        OD_Index = Config_ADC; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
        status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, ENABLE_ADC,
                                      RSDO + ADC_NODE_ID,
                                      0);
        if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
        {
            status = Desencolar_FIFO(&FIFO_CanTx);
            Transmit_CANOPenMsg(FIFO_CanTx);
        }
        stateInitAdc = 1;
        break;

    case 1: //Check if we have a TimeOut reception from PS, if so,
        //set an error
        if (StatusCom.StatusFlags.Flags.DataAvailable)
        {
            stateInitAdc = 2;
        }
        else if (StatusCom.StatusFlags.Flags.ErrorCom)
        {
            //TODO: Set an error flag, stop everything and display it in user interface
        }
        break;

    case 2: //No TimeOut reception from PS, then analyze CAN message received
        //AnalyzeCanMsg();
        if (AdcValuesSaved.StatusFlags.Flags.VoltageAnswerFromAdc)
        {
            AdcValuesSaved.StatusFlags.Flags.VoltageAnswerFromAdc = false;
            stateInitAdc = 2;
        }
        else if (AdcValuesSaved.StatusFlags.Flags.CurrentAnswerFromAdc)
        {
            AdcValuesSaved.StatusFlags.Flags.CurrentAnswerFromAdc = false;
            stateInitAdc = 3;
        }
        else
        {
            //TODO: Set an error flag, stop everything and display it in user interface
            return (0x00);
        }
        break;

    case 3:
        //All ok
        stateInitAdc = 0;
        status = 0x01;
        break;

    default:
        //TODO: Set an error flag, stop everything and display it in user interface
        status = 0x00;
        break;
    }
    return (status);
}
/**
 * @brief Initialize Power Supply. First check communication with PowerSuply,
 *        then set current/voltage to zero, finally save those values on two
 *        variables
 *
 * @return uint16_t 
 */
uint16_t InitPowerSupply(void)
{
    static uint16_t stateInitPs = 0;
    uint16_t status = 0x01; //By default all ok
    enum OD_Index;

    switch (stateInitPs)
    {
    case 0: //Send a Device Name CAN message to test communication with PS
        PsEnable_ON();
        OD_Index = Nombre_dispositivo;
        status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, 0,
                                      RSDO + PS_NODE_ID,
                                      OD_READ_2BYTES);
        if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
        {
            status = Desencolar_FIFO(&FIFO_CanTx);
            Transmit_CANOPenMsg(FIFO_CanTx);
        }
        stateInitPs = 1;
        break;

    case 1: //Check if we have a TimeOut reception from PS, if so,
        //set an error
        if (StatusCom.StatusFlags.Flags.DataAvailable)
        {
            StatusCom.StatusFlags.Flags.DataAvailable = false;
            stateInitPs = 2;
        }
        else if (StatusCom.StatusFlags.Flags.ErrorCom)
        {
            //TODO: Set an error flag, stop everything and display it in user interface
            status = 0x00;
        }
        break;

    case 2: //No TimeOut reception from PS, then analyze CAN message received
        //AnalyzeCanMsg();
        if ((OD_Index == Nombre_dispositivo) && (PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName))
        {
            //Correct CAN message received
            PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName = false;
            stateInitPs = 3;
        }
        else if ((OD_Index == Udc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerVSet))
        {
            PowerSupplyValues.StatusFlags.Flags.AnswerVSet = false;
            stateInitPs = 4;
        }
        else if ((OD_Index == Idc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerISet))
        {
            PowerSupplyValues.StatusFlags.Flags.AnswerISet = false;
            stateInitPs = 5;
        }
        break;

    case 3: //Next set V to zero values, just in case
        OD_Index = Udc_Out_Setpoint;
        status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, 0x0000F401,
                                      RSDO + PS_NODE_ID,
                                      OD_WRITE_2BYTES);
        if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
        {
            status = Desencolar_FIFO(&FIFO_CanTx);
            Transmit_CANOPenMsg(FIFO_CanTx);
        }
        stateInitPs = 1;
        break;

    case 4: //Next set I to zero values, just in case
        OD_Index = Idc_Out_Setpoint;
        status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, 0,
                                      RSDO + PS_NODE_ID,
                                      OD_WRITE_2BYTES);
        if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
        {
            status = Desencolar_FIFO(&FIFO_CanTx);
            Transmit_CANOPenMsg(FIFO_CanTx);
        }
        stateInitPs = 1;
        break;

    case 5:
        PowerSupplyValues.ActualCurrentValue = 0x0000F401;
        PowerSupplyValues.ActualVoltageValue = 0;
        PowerSupplyValues.PowerModuleStatus = 0x01; //Charger ON
        stateInitPs = 0;
        status = 0X01;
        break;

    default:
        //TODO: Set an error flag, stop everything and display it in user interface
        status = 0x00;
        break;
    }

    return (status);
}
/*
 * @brief Set voltage from power supply to desired value 
 * 
 * @param VoltageRequest 
 * @return uint16_t 
 */
uint16_t PsSetVoltageCurrent(uint16_t VoltageRequest, int16_t CurrentRequest,
                             bool EnablePs)
{
    static uint16_t stateSetPsVI = 0;
    uint16_t status = 0x01; //By default all ok
    uint16_t tmpVoltageValue = 0;
    int16_t tmpCurrentValue = 0;
    static uint16_t CounterVoltageIterations = 9;
    static uint16_t CounterCurrentIterations = 9;
    float SlopeVoltage = 0.0;
    float SlopeCurrent = 0.0;
    enum OD_Index;

    switch (stateSetPsVI)
    {
    case 0: //Send a Device Name CAN message to test communication with PS
        //this CAN message is also used as keep alive frame
        if (EnablePs == false)
        {
            PsEnable_OFF();
            stateSetPsVI = 0;
            break;
        }
        else if (EnablePs == true)
        {
            PsEnable_ON();
        }
        OD_Index = Nombre_dispositivo;
        status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, 0,
                                      RSDO + PS_NODE_ID,
                                      OD_WRITE_2BYTES);
        Transmit_CANOPenMsg(FIFO_CanTx);
        stateSetPsVI = 1;
        break;

    case 1: //Check if we have a TimeOut reception from PS, if so,
        //set an error
        if (StatusCom.StatusFlags.Flags.DataAvailable)
        {
            stateSetPsVI = 2;
        }
        else if (StatusCom.StatusFlags.Flags.ErrorCom)
        {
            //TODO: Set an error flag, stop everything and display it in user interface
            return (0x00);
        }
        break;

    case 2: //No TimeOut reception from PS, then analyze CAN message received
        //AnalyzeCanMsg();
        if ((OD_Index == Nombre_dispositivo) && (PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName))
        {
            //Correct CAN message received
            PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName = false;
            stateSetPsVI = 3;
        }
        else if ((OD_Index == Udc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerVSet))
        {
            PowerSupplyValues.StatusFlags.Flags.AnswerVSet = false;
            stateSetPsVI = 4;
        }
        else if ((OD_Index == Idc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerISet))
        {
            PowerSupplyValues.StatusFlags.Flags.AnswerISet = false;
            stateSetPsVI = 5;
        }
        break;

    case 3: //Analize desired current/voltage values. If they are different from previous
        //current/voltage values, then calculate necessary values to ramp up/down
        //and go to next state. If not, go to first state and send keep alive frame every 500ms

        /* if (VoltageRequest < 0) //Not possible. but just in case
         {
         //TODO: Set an error flag, stop everything and display it in user interface
         status=0x00;
         break;
         }*/
        if (VoltageRequest != PowerSupplyValues.ActualVoltageValue)
        {
            if (VoltageRequest > V2G500V15A_VOLTAGE)
            {
                //Maximum value allowed
                VoltageRequest = V2G500V15A_VOLTAGE;
            }
            //Linear Interpolation. We calculate slope as:
            //Slope = ((Requested Value - Actual Values) / Seconds to ramp up/down)
            //Function, PsSetVoltageCurrent() is triggered every 500ms using our scheduler,
            //then after all iterations has been executed, ramp up or down process will have
            //lasted four seconds. Total number of steps-iteration = 8, each
            //step-iteration take 500ms.
            SlopeVoltage = ((VoltageRequest - PowerSupplyValues.ActualVoltageValue) /
                            LENGHT_SECONDS_RAMP); // slope V/s
            CounterVoltageIterations = 0;
            stateSetPsVI = 4;
        }

        else if (CurrentRequest != PowerSupplyValues.ActualCurrentValue)
        {
            if ((CurrentRequest & 0x80000000) == 0x80000000)
            {
                //Negative current
                if (CurrentRequest < -(V2G500V15A_VOLTAGE))
                {
                    //Maximum value allowed
                    CurrentRequest = -(V2G500V15A_CURRENT);
                }
            }
            else
            {
                //Positive current
                if (CurrentRequest > V2G500V15A_VOLTAGE)
                {
                    //Maximum value allowed
                    CurrentRequest = V2G500V15A_CURRENT;
                }
            }
            //Linear Interpolation. We calculate slope as:
            //Slope = ((Requested Value - Actual Values) / Total step-iteration)
            //Total steps-iteration = 8, each step-iteration take 500ms.
            //Function, PsSetVoltageCurrent() is triggered every 500ms using our scheduler,
            //then after all iterations has been executed, ramp up or down process will have
            //lasted four seconds
            SlopeCurrent = ((CurrentRequest - PowerSupplyValues.ActualCurrentValue) /
                            LENGHT_SECONDS_RAMP); // slope V/I
            CounterCurrentIterations = 0;
            stateSetPsVI = 4;
        }
        else if ((VoltageRequest == PowerSupplyValues.ActualVoltageValue) && (CurrentRequest == PowerSupplyValues.ActualCurrentValue))
        {
            //No changes on V/I to be done, then go to case 0 and send keep alive frame
            stateSetPsVI = 0;
        }
        break;

    case 4: //Send new values of voltage
        if (CounterVoltageIterations < 7)
        {
            //Start increment voltage. First 7 iterations
            tmpVoltageValue =
                (PowerSupplyValues.ActualVoltageValue + ((CounterVoltageIterations * MS_STEP_RAMP) * SlopeVoltage));
            OD_Index = Udc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx,
                                          tmpVoltageValue,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            Transmit_CANOPenMsg(FIFO_CanTx);
            CounterVoltageIterations++;
            stateSetPsVI = 1;
        }
        else if (CounterVoltageIterations == 7)
        {
            //Last iteration, we set the final values as value requested by system
            OD_Index = Udc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, VoltageRequest,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            Transmit_CANOPenMsg(FIFO_CanTx);
            CounterVoltageIterations++;
            stateSetPsVI = 1;
        }
        else if (CounterVoltageIterations == 8)
        {
            //Finished ramp up/down for voltage, reset step-iterations counter
            //and go to ramp up/dowm for current
            CounterVoltageIterations = 9;
            stateSetPsVI = 5;
        }
        break;

    case 5: //Send new values of current

        if (CounterCurrentIterations < 7)
        {
            tmpCurrentValue =
                (PowerSupplyValues.ActualCurrentValue + ((CounterCurrentIterations * MS_STEP_RAMP) * SlopeCurrent));
            OD_Index = Idc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx,
                                          tmpCurrentValue,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            Transmit_CANOPenMsg(FIFO_CanTx);
            CounterVoltageIterations++;
            stateSetPsVI = 1;
        }
        else if (CounterCurrentIterations == 7)
        {
            //Last iteration, we set the final values as value requested by system
            OD_Index = Idc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(OD_Index, &FIFO_CanTx, CurrentRequest,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            Transmit_CANOPenMsg(FIFO_CanTx);
            CounterCurrentIterations = 8;
            stateSetPsVI = 1;
        }
        else if (CounterCurrentIterations == 8)
        {
            //Finished ramp up/down for current, reset step-iteration conter
            //and go to next state to save final values
            CounterCurrentIterations = 9;
            stateSetPsVI = 6;
        }
        break;

    case 6:
        PowerSupplyValues.ActualCurrentValue = CurrentRequest;
        PowerSupplyValues.ActualVoltageValue = VoltageRequest;
        stateSetPsVI = 0;
        status = 0X01;
        break;

    default:
        //TODO: Set an error flag, stop everything and display it in user interface
        status = 0x00;
        break;
    }

    return (status);
}
/**
 * @brief 
 * 
 */
void Scheduler(void)
{
    uint16_t status = 0x01; //By default all ok

    Count1ms++;
    if (Count1ms >= 10)
    {
        Count1ms = 0;
        Count10ms++;
        //Every 10ms
        TimeOutRxCanMsg();
    }
/*    if(Count10ms == 5)
    {
        //Every 50ms


    }*/
    if (Count10ms >= 10)
    {
        Count10ms = 0;
        Count100ms++;
        //Every 100ms
        if ((StatusCom.StatusFlags.Flags.ErrorCom != true) && (FIFO_CanRx.Estado_PILA != PILA_VACIA))
               {
                   //No timeout in Rx and there are data on FIFO Rx to analyze
                   AnalyzeCanMsg();
               }
       
    }
    if (Count100ms >= 5)
    {
        Count100ms = 0;
        Count500ms++;
        //Every 500ms
        if (PowerSupplyValues.PowerModuleStatus != 0x01)
        {
            //Power Supply OFF. This sentence execute only once.
            //Check com. with power supply and set values
            status = InitPowerSupply();
        }
        
    }
    if (Count500ms >= 2)
    {
        Count500ms = 0;
        Count1s++;
        //Every 1000ms
    }
    if (Count1s >= 5)
    {
        Count1s = 0;
        Count5s++;
        //Every 5000ms
    }
    if (Count5s >= 2)
    {
        Count5s = 0;
        //Every 10000ms
    }
}
/**
 * @brief 
 * 
 */
void TimeOutRxCanMsg(void)
{
    if ((StatusCom.StatusFlags.Flags.TransmittedCanMsg == true) && (StatusCom.StatusFlags.Flags.DataAvailable != true))
    {
        ulTimeOutCANRx++;
        if (ulTimeOutCANRx == TIMEOUT_CAN_RX)
        {
            StatusCom.StatusFlags.Flags.TransmittedCanMsg = false;
            StatusCom.StatusFlags.Flags.DataAvailable = false;
            StatusCom.StatusFlags.Flags.ErrorCom = true;
            ulTimeOutCANRx = 0;
        }
    }
}
/*-----------------------------------END MISCELANIOUS FUNCTIONS--------------------------------------*/

/*----------------------------------BEGIN CONTROL LOGIC FUNCTIONS------------------------------------*/
/**
 * @brief
 * 
 */
void AnalyzeCanMsg(void)
{
    uint16_t SubIndex = 0; //Temporal variable to save SubIndex
    uint16_t ObjIndex = 0; //Temporal variable to save Object Index
    uint16_t AccessMode = 0;
    uint16_t DataSaved[9] = {0x00}; //Temporal array where store data received from CAN
    uint32_t TmpValue = 0;          //
    uint16_t DictionaryIndex = 0;   //Temporal variable used for check ObjectIndex
                                    //and AccesCmd
    uint16_t NodeIdRx = 0;          //Temporal variable to store Id from node transmitter

    //TODO: Check this
    //memset(DataSaved, 0x00, 9); //Reset array to zero

    Desencolar_FIFO(&FIFO_CanRx);
    //Move data from FIFO to variable DataSaved for post-processing
    memcpy((void *)DataSaved, (void *)FIFO_CanRx.Datos_Recibidos, 9);

    NodeIdRx = (uint16_t)DataSaved[0];
    AccessMode = (uint16_t)DataSaved[1];
    ObjIndex = (uint16_t)DataSaved[2] + (uint16_t)(DataSaved[3] << 8);
    SubIndex = (uint16_t)DataSaved[4];
    //AdcValue = datos_char_to_int(DataSaved[5]);
    TmpValue = (uint32_t)DataSaved[8] << 24;
    TmpValue += (uint32_t)DataSaved[7] << 16;
    TmpValue += (uint32_t)DataSaved[6] << 8;
    TmpValue += (uint32_t)DataSaved[5];

    DictionaryIndex = Econtrar_Indice_Diccionario(ObjIndex, SubIndex);

    if (DictionaryIndex == FIN_Diccionario)
    {
        // Requested Object index doesn´t exit, then set flag error and tx Error CAN msg
        StatusErrors.StatusFlags.Flags.ObjectIndexError = true;
        Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                  RSDO + NodeIdRx);
    }
    else
    {
        // Requested Object index received exist
        if (Diccionario_CanOpen[DictionaryIndex].Modo_Acceso == AccessMode) //TODO: Check if necessary this condition
        {
            // Requested Access mode for that Object index exist
            if ((AccessMode >= OD_READ) && (AccessMode <= OD_READ_4BYTES))
            {
                StatusCom.StatusFlags.Flags.AccessModeRead = true;
                StatusCom.StatusFlags.Flags.AccessModeWrite = false;
            }
            else if ((AccessMode >= OD_WRITE) && (AccessMode <= OD_WRITE_4BYTES))
            {
                StatusCom.StatusFlags.Flags.AccessModeRead = false;
                StatusCom.StatusFlags.Flags.AccessModeWrite = true;
            }
            else if ((AccessMode >= OD_WRITE + OD_READ) && (AccessMode <= OD_WRITE_4BYTES + OD_READ_4BYTES))
            {
                StatusCom.StatusFlags.Flags.AccessModeRead = true;
                StatusCom.StatusFlags.Flags.AccessModeWrite = true;
            }
            //TODO: Check  Access Command Error 0x80
            else
            {
                //Requested Access Mode doesn´t exit, then set flag error and tx Error CAN msg
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
                //TODO: No transmission done, only set CAN error frame
            }
        }
    }

    switch (ObjIndex)
    {
    case 0x9000: // Get Voltage/Current values from ADC

        if (SubIndex == 0x00) //Vout from ADC
        {
            //Request Access Mode correct
            if (StatusCom.StatusFlags.Flags.AccessModeRead)
            {
                //Do nothing
            }
            else if (StatusCom.StatusFlags.Flags.AccessModeWrite)
            {
                AdcValuesSaved.VoltageValue = TmpValue;                     //Raw value for Chademo logic
                AdcValuesSaved.floatVolatageValue = (float)TmpValue / 10.0; //0.1 A/bit
                AdcValuesSaved.StatusFlags.Flags.VoltageAnswerFromAdc =
                    true;
            }
            else if ((StatusCom.StatusFlags.Flags.AccessModeRead) && ((StatusCom.StatusFlags.Flags.AccessModeWrite)))
            {
                //Do nothing
            }
        }
        else
        {
            //Requested Access Mode incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.AccessCmdError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        if (SubIndex == 0x01) //Iout from ADC
        {

            //Request Access Mode correct
            if (StatusCom.StatusFlags.Flags.AccessModeRead)
            {
                //Do nothing
            }
            else if (StatusCom.StatusFlags.Flags.AccessModeWrite)
            {
                AdcValuesSaved.CurrentValue = TmpValue; //Raw value for Chademo logic
                //Get sign of the Current Value
                if ((AdcValuesSaved.CurrentValue & 0x80000000) == 0x80000000)
                {
                    AdcValuesSaved.NegativeCurrentValue = true; //Negative number
                    // If it is negative, make it positive by inverting the bits
                    // and adding one.
                    TmpValue = ~TmpValue; //Bitwise negation of all bits
                    TmpValue += 0x01;
                }
                else
                {
                    AdcValuesSaved.NegativeCurrentValue = false; //No negative number
                }
                //AdcValuesSaved.floatCurrentValue = (float)TmpValue / 10.0; //0.1 A/bit
                AdcValuesSaved.StatusFlags.Flags.CurrentAnswerFromAdc =
                    true;
            }
            else if ((StatusCom.StatusFlags.Flags.AccessModeRead) && ((StatusCom.StatusFlags.Flags.AccessModeWrite)))
            {
                //Do nothing
            }
        }
        else
        {
            //Requested Access Mode incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.AccessCmdError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x1008: //Power Supply Device Name

        if (SubIndex == 0x04)
        {
            //Check answer from Power Supply. "V2G500V15A" model return the string "_V2G"
            if (TmpValue == 0x47325620) //Chars values of string "G2V_", little-endian of "_V2G"
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName = true;
            }
            else
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerDeviceNameError =
                    false;
            }
        }
        else
        {
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
        }
        break;

    case 0x2109: //Output voltage set point

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x60)
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerVSet = true;
            }
        }
        break;

    case 0x210A: //Output current set point

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x60)
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerISet = true;
            }
        }
        break;

    default:
        StatusErrors.StatusFlags.Flags.ObjectIndexError = true;
        Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                  RSDO + NodeIdRx);
        break;
    }
}
/*  END CONTROL LOGIC FUNCTIONS */
