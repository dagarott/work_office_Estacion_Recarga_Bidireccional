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
uint16_t InitAdc(void);
uint16_t InitPowerSupply(void);
uint16_t PsSetVoltageCurrent(uint16_t VoltageRequest, int16_t CurrentRequest);
uint16_t PsEnableDisable(bool EnablePs);
uint16_t PsKeepAlive(void);
uint16_t PsReadOutputVI(void);
uint16_t FSM_FastCharge(void);
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
uint16_t Count250ms = 0;          //Variable used by scheduler
uint16_t Count500ms = 0;          //Variable used by scheduler
uint16_t Count1s = 0;             //Variable used by scheduler
uint16_t Count2s = 0;             //Variable used by scheduler
uint16_t Count4s = 0;             //Variable used by scheduler

typedef struct sObjectTx
{
    uint16_t OB;      //Last Object Sent with CAN peripheral
    uint16_t Node_ID; //Id of Node that has sent last Object
} LastObjectTx_t;

LastObjectTx_t PowerSupply_LastObjectTx = {0x00, 0x00}; //TODO: May it be erased?

AdcValues_t AdcValuesSaved = {0, 0.0, 0, 0.0, 0, 0, 0, 0};

PowerSupplyValues_t PowerSupplyValues = {" ", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                         0, 0};

FsmStatus_t FsmStatus = {NoneFSM, NoneFSM};

uint16_t CurrentProcess = 0; //TODO: May it be erased?

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
/*
 uint16_t Set_UartMsg_To_Tx(uint16_t type, uint16_t *data)
 {

 }*/
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
/*
 uint16_t Set_PCMsg_To_Tx()
 {

 }
 */
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
    static enum Indice_Diccionario_TPO Adc_Init_OD_Index;

    switch (stateInitAdc)
    {
    case 0:
        Adc_Init_OD_Index = Config_ADC; //CAN command array Index. Commands present in Diccionario_CANOpen.c file
        status = Set_CANOpenMsg_To_Tx(Adc_Init_OD_Index, &FIFO_CanTx,
                                      ENABLE_ADC,
                                      RSDO + ADC_NODE_ID, 0);
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
    static enum Indice_Diccionario_TPO Ps_Init_OD_Index;

    if ((PowerSupplyValues.PowerModuleStatus & 0x08) != 0x08) //We want execute this code only once,
                                                              //so that, the first time bit 3 is set to zero
                                                              //and at the end of the code we set that bit
                                                              //to one
    {
        switch (stateInitPs)
        {
        case 0: //Send a Device Name CAN message to test communication with PS
            PsEnable_ON();
            Ps_Init_OD_Index = Nombre_dispositivo;
            status = Set_CANOpenMsg_To_Tx(Ps_Init_OD_Index, &FIFO_CanTx, 0,
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
            //set an error, otherwise set next state
            //if (StatusCom.StatusFlags.Flags.DataAnalyzed)
            //{
            //    StatusCom.StatusFlags.Flags.DataAnalyzed = false;

            if ((StatusCom.StatusFlags.Flags.ErrorCom == false) &&
                (StatusErrors.StatusFlags.Flags.AccessCmdError == false) &&
                (StatusErrors.StatusFlags.Flags.SubIndexError == false) &&
                (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {

                if ((Ps_Init_OD_Index == Nombre_dispositivo) && (PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName))
                {
                    //Correct CAN message received
                    PowerSupplyValues.StatusFlags.Flags.AnswerDeviceName =
                        false;
                    stateInitPs = 2;
                }
                else if ((Ps_Init_OD_Index == Udc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerVSet))
                {
                    PowerSupplyValues.StatusFlags.Flags.AnswerVSet = false;
                    stateInitPs = 3;
                }
                else if ((Ps_Init_OD_Index == Idc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerISet))
                {
                    PowerSupplyValues.StatusFlags.Flags.AnswerISet = false;
                    stateInitPs = 4;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom || StatusErrors.StatusFlags.Flags.AccessCmdError ||
                     StatusErrors.StatusFlags.Flags.SubIndexError ||
                     StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                //TODO: State assigned for trigger "Error Machine"
                stateInitPs = 6;
                status = 0x00;
            }
            else
            {
                stateInitPs = 1;
            }
            break;

        case 2: //Next set V to zero value, just in case
            Ps_Init_OD_Index = Udc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(Ps_Init_OD_Index, &FIFO_CanTx, 0x01F4,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            stateInitPs = 1;
            break;

        case 3: //Next set I to zero value, just in case
            Ps_Init_OD_Index = Idc_Out_Setpoint;
            status = Set_CANOpenMsg_To_Tx(Ps_Init_OD_Index, &FIFO_CanTx, 0x00,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            stateInitPs = 1;
            break;

        case 4:
            //All done. save actual values and set bit "Initiated Power Supply" to inform
            //the system that Power supply is set with knows values, Voltage=0/Current=0
            PowerSupplyValues.ActualCurrentValue = 0x00;
            PowerSupplyValues.ActualVoltageValue = 0x00;
            PowerSupplyValues.PowerModuleStatus |= 0x08; //Bit /   Description /   Value
                                                         // 0      Ps Enable  / 0: OFF; 1: ON
                                                         // 1      Communication / 0: OK COM; 1: ERROR COM
                                                         // 2      Updated Value / 0: NO; 1: YES
                                                         // 3      Initiated Power Supply  / 0: NO; 1: YES

            stateInitPs = 0;
            status = 0X01;
            break;

        case 5: //TODO: State asigned for trigger "Error Machine"

            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            status = 0x00;
            break;
        }
    }
    return (status);
}
/*
 * @brief Set voltage from power supply to desired value 
 * 
 * @param VoltageRequest 
 * @return uint16_t 
 */
uint16_t PsSetVoltageCurrent(uint16_t VoltageRequest, int16_t CurrentRequest)
{
    static uint16_t stateSetPsVI = 0;
    uint16_t status = 0x01; //By default all ok
    static uint16_t tmpVoltageValue = 0;
    static int16_t tmpCurrentValue = 0;
    static uint16_t CounterVoltageIterations = Number_Steps_Ramp + 1;
    static uint16_t CounterCurrentIterations = Number_Steps_Ramp + 1;
    static float SlopeVoltage = 0.0;
    static float SlopeCurrent = 0.0;
    static enum Indice_Diccionario_TPO Ps_Set_OD_Index;

    if ((PowerSupplyValues.PowerModuleStatus & 0x01) == 0x01) //Check if Power Supply is ON
    {

        switch (stateSetPsVI)
        {
        case 0: //Analize desired current/voltage values. If they are different from previous
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
                if (VoltageRequest > V2G500V15A_MAX_VOLTAGE)
                {
                    //Maximum value allowed
                    VoltageRequest = V2G500V15A_MAX_VOLTAGE;
                }
                //Linear Interpolation. We calculate slope as:
                //Slope = ((Requested Value - Actual Values) / Seconds to ramp up/down)
                //Function, PsSetVoltageCurrent(), is triggered every 500ms using our scheduler.
                //After all iterations has been executed, ramp up or down process must have
                //lasted 4 seconds. Total number of steps-iteration = 8, each
                //step-iteration takes 500ms.
                SlopeVoltage = (VoltageRequest - PowerSupplyValues.ActualVoltageValue) /
                               Length_Seconds_Ramp; // slope V/ms
                //CounterVoltageIterations = 0; //TODO: Valor original permitia hacer n-iteraciones
                CounterVoltageIterations = (Number_Steps_Ramp - 1); //TODO: Cambiado para que se ejecute una
                                                                    //sola iteracion en tensión
                PowerSupplyValues.PowerModuleStatus &= 0xFB;        //Values not updated yet, then set to zero
                                                                    //this bit
                stateSetPsVI = 2;
            }

            if (CurrentRequest != PowerSupplyValues.ActualCurrentValue)
            {
                if ((CurrentRequest & 0x80000000) == 0x80000000)
                {
                    //Negative current
                    if (CurrentRequest < -(V2G500V15A_CURRENT))
                    {
                        //Maximum value allowed
                        CurrentRequest = -(V2G500V15A_CURRENT);
                    }
                }
                else
                {
                    //Positive current
                    if (CurrentRequest > V2G500V15A_CURRENT)
                    {
                        //Maximum value allowed
                        CurrentRequest = V2G500V15A_CURRENT;
                    }
                }
                //Linear Interpolation. We calculate slope as:
                //Slope = ((Requested Value - Actual Values) / Total step-iteration)
                //Total steps-iteration = 108, each step-iteration take 200ms.
                //Function, PsSetVoltageCurrent() is triggered every 200ms using our scheduler,
                //then after all iterations has been executed, ramp up or down process will have
                //lasted four seconds
                SlopeCurrent = (CurrentRequest - PowerSupplyValues.ActualCurrentValue) /
                               Length_Seconds_Ramp; // slope I/ms
                CounterCurrentIterations = 0;
                PowerSupplyValues.PowerModuleStatus &= 0xFB; //Values not updated yet, then set to zero
                                                             //this bit
                stateSetPsVI = 2;
            }
            else if ((VoltageRequest == PowerSupplyValues.ActualVoltageValue) && (CurrentRequest == PowerSupplyValues.ActualCurrentValue))
            {
                //No changes on V/I to be done, then go to case 4
                stateSetPsVI = 4;
            }
            else
            {
                //Error
                stateSetPsVI = 5;
                status = 0x00;
            }
            break;

        case 1: //Check if we have a TimeOut reception from PS. Check if data received
                //has been analyzed
                //if (StatusCom.StatusFlags.Flags.DataAnalyzed)
                //{
                // StatusCom.StatusFlags.Flags.DataAnalyzed = false;

            if ((StatusCom.StatusFlags.Flags.ErrorCom == false) &&
                (StatusErrors.StatusFlags.Flags.AccessCmdError == false) &&
                (StatusErrors.StatusFlags.Flags.SubIndexError == false) &&
                (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                if ((Ps_Set_OD_Index == Udc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerVSet))
                {
                    PowerSupplyValues.StatusFlags.Flags.AnswerVSet = false;
                    stateSetPsVI = 2;
                }
                else if ((Ps_Set_OD_Index == Idc_Out_Setpoint) && (PowerSupplyValues.StatusFlags.Flags.AnswerISet))
                {
                    PowerSupplyValues.StatusFlags.Flags.AnswerISet = false;
                    stateSetPsVI = 3;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom || StatusErrors.StatusFlags.Flags.AccessCmdError ||
                     StatusErrors.StatusFlags.Flags.SubIndexError ||
                     StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                //TODO: State asigned for trigger "Error Machine"
                stateSetPsVI = 5;
                status = 0x00;
            }
            else
            {
                stateSetPsVI = 1;
            }
            break;

        case 2: //Send new values of voltage

            // if (CounterVoltageIterations < (Number_Steps_Ramp - 1))
            // {
            //     //Start increment voltage. First 7 iterations
            //     tmpVoltageValue = (PowerSupplyValues.ActualVoltageValue
            //             + (CounterVoltageIterations * Ms_Step_ramp
            //                     * SlopeVoltage));
            //     Ps_Set_OD_Index = Udc_Out_Setpoint;
            //     status = Set_CANOpenMsg_To_Tx(Ps_Set_OD_Index, &FIFO_CanTx,
            //                                   tmpVoltageValue,
            //                                   RSDO + PS_NODE_ID,
            //                                   OD_WRITE_2BYTES);
            //     if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            //     {
            //         status = Desencolar_FIFO(&FIFO_CanTx);
            //         Transmit_CANOPenMsg(FIFO_CanTx);
            //     }
            //     stateSetPsVI = 1; //We go to state one, where we check if an Ack for this command
            //                       //it has been sent by the Power Supply
            //     CounterVoltageIterations++;
            // }
            // else if (CounterVoltageIterations == (Number_Steps_Ramp - 1))
            // {
            //     //Last iteration, we set the final values as value requested by system
            //     Ps_Set_OD_Index = Udc_Out_Setpoint;
            //     status = Set_CANOpenMsg_To_Tx(Ps_Set_OD_Index, &FIFO_CanTx,
            //                                   VoltageRequest,
            //                                   RSDO + PS_NODE_ID,
            //                                   OD_WRITE_2BYTES);
            //     if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            //     {
            //         status = Desencolar_FIFO(&FIFO_CanTx);
            //         Transmit_CANOPenMsg(FIFO_CanTx);
            //     }
            //     stateSetPsVI = 1; //We go to state one, where we check if an Ack for this command
            //                       //it has been sent by the Power Supply
            //     CounterVoltageIterations++;
            // }
            // else if (CounterVoltageIterations == Number_Steps_Ramp)
            // {
            //     //At the end of the ramp up/down voltage state, reset step-iterations counter
            //     //and go to ramp up/dowm current state if needed
            //     CounterVoltageIterations = (Number_Steps_Ramp + 1);
            //     PowerSupplyValues.ActualVoltageValue = VoltageRequest;
            //     stateSetPsVI = 3;
            // }
            // else
            // {
            //     stateSetPsVI = 3;
            // }
            if (CounterVoltageIterations == (Number_Steps_Ramp - 1))
            {
                //Last iteration, we set the final values as value requested by system
                Ps_Set_OD_Index = Udc_Out_Setpoint;
                status = Set_CANOpenMsg_To_Tx(Ps_Set_OD_Index, &FIFO_CanTx,
                                              VoltageRequest,
                                              RSDO + PS_NODE_ID,
                                              OD_WRITE_2BYTES);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
                stateSetPsVI = 1; //We go to state one, where we check if an Ack for this command
                                  //it has been sent by the Power Supply
                CounterVoltageIterations++;
            }
            else if (CounterVoltageIterations == Number_Steps_Ramp)
            {
                //At the end of the ramp up/down voltage state, reset step-iterations counter
                //and go to ramp up/dowm current state if needed
                CounterVoltageIterations = (Number_Steps_Ramp + 1);
                PowerSupplyValues.ActualVoltageValue = VoltageRequest;
                stateSetPsVI = 3;
            }
            else
            {
                stateSetPsVI = 3;
            }
            break;

        case 3: //Send new values of current

            if (CounterCurrentIterations < (Number_Steps_Ramp - 1))
            {
                tmpCurrentValue = (PowerSupplyValues.ActualCurrentValue + (CounterCurrentIterations * Ms_Step_ramp * SlopeCurrent));
                Ps_Set_OD_Index = Idc_Out_Setpoint;
                status = Set_CANOpenMsg_To_Tx(Ps_Set_OD_Index, &FIFO_CanTx,
                                              tmpCurrentValue,
                                              RSDO + PS_NODE_ID,
                                              OD_WRITE_2BYTES);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
                stateSetPsVI = 1;
                CounterCurrentIterations++;
            }
            else if (CounterCurrentIterations == (Number_Steps_Ramp - 1))
            {
                //Last iteration, we set the final values as value requested by system
                Ps_Set_OD_Index = Idc_Out_Setpoint;
                status = Set_CANOpenMsg_To_Tx(Ps_Set_OD_Index, &FIFO_CanTx,
                                              CurrentRequest,
                                              RSDO + PS_NODE_ID,
                                              OD_WRITE_2BYTES);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
                stateSetPsVI = 1;
                CounterCurrentIterations++;
            }
            else if (CounterCurrentIterations == Number_Steps_Ramp)
            {
                //Completed ramp up/down for current, we reset step-iteration counter
                //and we go to last state to save final values, and set flag Power Enable
                //to true
                CounterCurrentIterations = (Number_Steps_Ramp + 1);
                PowerSupplyValues.ActualCurrentValue = CurrentRequest;
                stateSetPsVI = 0;
            }
            else
            {
                stateSetPsVI = 0;
            }
            break;

        case 4:                                          //Process finished, set "Update Value" bit to one to inform the system
            PowerSupplyValues.PowerModuleStatus |= 0x04; // 0      Power Supply Enable  / 0: OFF; 1: ON
                                                         // 1      Communication / 0: OK COM; 1: ERROR COM
                                                         // 2      Updated Value / 0: NO; 1: YES
                                                         // 3      Initiated Power Supply  / 0: NO; 1: YES

            stateSetPsVI = 0;
            status = 0X01;
            break;

        case 5: //TODO: State asigned for trigger "Error Machine"
            stateSetPsVI = 0;
            status = 0x00;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            stateSetPsVI = 0;
            status = 0x00;
            break;
        }
    }
    return (status);
}
/**
 * @brief  To have Power Supply in ON state it is necessary to send CANbus data with 
 *         an interval of at least <1000ms. We send an enable command as CAN message.
 * 
 * @return uint16_t 
 */
uint16_t PsKeepAlive(void)
{
    uint16_t status = 0x01; //By default all ok
    static uint_fast16_t stateKeepAlive = 0;
    static enum Indice_Diccionario_TPO Ps_Alive_OD_Index;

    if ((PowerSupplyValues.PowerModuleStatus & 0x01) == 0x01) //Check if Power Supply is ON
    {
        switch (stateKeepAlive)
        {
        case 0:

            Ps_Alive_OD_Index = Module_Enable;
            status = Set_CANOpenMsg_To_Tx(Ps_Alive_OD_Index, &FIFO_CanTx,
                                          ENABLE_PS,
                                          RSDO + PS_NODE_ID,
                                          OD_WRITE_2BYTES);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            stateKeepAlive = 1;
            break;

        case 1:

            //if ((StatusCom.StatusFlags.Flags.DataAnalyzed)
            //        && (PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs))
            if ((StatusCom.StatusFlags.Flags.ErrorCom == false) &&
                (StatusErrors.StatusFlags.Flags.AccessCmdError == false) &&
                (StatusErrors.StatusFlags.Flags.SubIndexError == false) &&
                (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                if (PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs)
                {
                    //Confirmed reception of ACk for Enable Power Supply command
                    //StatusCom.StatusFlags.Flags.DataAnalyzed = false;
                    PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs = false;
                    stateKeepAlive = 0;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom || StatusErrors.StatusFlags.Flags.AccessCmdError ||
                     StatusErrors.StatusFlags.Flags.SubIndexError ||
                     StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                //TODO: State asigned for trigger "Error Machine"
                stateKeepAlive = 2;
            }
            else
            {
                stateKeepAlive = 1;
            }
            break;

        case 2: //TODO: State assigned for trigger "Error Machine"
            status = 0x00;
            stateKeepAlive = 0;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            //TODO: State assigned for trigger "Error Machine"
            status = 0x00;
            break;
        }
    }
    return (status);
}
uint16_t PsReadOutputVI(void)
{

    uint16_t status = 0x01; //By default all ok
    static uint16_t stateReadOutput = 0;
    static uint16_t counterAverage = 0;
    static enum Indice_Diccionario_TPO Ps_ReadOut_OD_Index;

    if ((PowerSupplyValues.PowerModuleStatus & 0x01) == 0x01) //Check if Power Supply is ON
    {
        switch (stateReadOutput)
        {
        case 0: //

            Ps_ReadOut_OD_Index = Udc_Out;
            status = Set_CANOpenMsg_To_Tx(Ps_ReadOut_OD_Index, &FIFO_CanTx,
                                          0x00,
                                          RSDO + PS_NODE_ID,
                                          OD_READ);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            Ps_ReadOut_OD_Index = Idc_Out;
            status = Set_CANOpenMsg_To_Tx(Ps_ReadOut_OD_Index, &FIFO_CanTx,
                                          0x00,
                                          RSDO + PS_NODE_ID,
                                          OD_READ);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            stateReadOutput = 1;
            break;

        case 1:

            //if (StatusCom.StatusFlags.Flags.DataAnalyzed)
            //{
            //StatusCom.StatusFlags.Flags.DataAnalyzed = false;

            if ((StatusCom.StatusFlags.Flags.ErrorCom == false) &&
                (StatusErrors.StatusFlags.Flags.AccessCmdError == false) &&
                (StatusErrors.StatusFlags.Flags.SubIndexError == false) &&
                (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                if (((Ps_ReadOut_OD_Index == Udc_Out) || (Ps_ReadOut_OD_Index == Idc_Out)) && ((PowerSupplyValues.StatusFlags.Flags.AnswerReadVoutput) && (PowerSupplyValues.StatusFlags.Flags.AnswerReadIoutput)))
                {
                    PowerSupplyValues.StatusFlags.Flags.AnswerReadVoutput =
                        false;
                    PowerSupplyValues.StatusFlags.Flags.AnswerReadIoutput =
                        false;
                    stateReadOutput = 2;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom &&
                     StatusErrors.StatusFlags.Flags.AccessCmdError &&
                     StatusErrors.StatusFlags.Flags.SubIndexError &&
                     StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                //TODO: State aSsigned for trigger "Error Machine"
                status = 0x00;
                stateReadOutput = 3;
            }
            else
            {
                stateReadOutput = 1;
            }

            break;

        case 2:
            PowerSupplyValues.PowerModuleStatus |= 0x10; //Bit /   Description /   Value
                                                         // 0      Ps On  / 0: OFF; 1: ON
                                                         // 1      Communication / 0: OK COM; 1: ERROR COM
                                                         // 2      Updated Value / 0: NO; 1: YES
                                                         // 3      Initiated Ps  / 0: NO; 1: YES
                                                         // 4      Updated ADC values / 0: NO; 1: YES
            stateReadOutput = 0;                         //For next time
            break;

        case 3: //TODO: State asigned for trigger "Error Machine"
            stateReadOutput = 0;
            status = 0x00;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            //TODO: State asigned for trigger "Error Machine"
            status = 0x00;
            stateReadOutput = 0;
            break;
        }
    }
    return (status);
}
/**
 * @brief 
 * 
 * @param EnablePs 
 * @return uint16_t 
 */
uint16_t PsEnableDisable(bool EnablePs)
{
    uint16_t status = 0;
    static uint_fast16_t stateEnablePs = 0;
    static enum Indice_Diccionario_TPO Ps_OnOff_OD_Index;

    if ((PowerSupplyValues.PowerModuleStatus & 0x01) != (EnablePs))
    {
        switch (stateEnablePs)
        {
        case 0:

            Ps_OnOff_OD_Index = Module_Enable;
            if (EnablePs == true)
            {
                PsEnable_ON();
                status = Set_CANOpenMsg_To_Tx(Ps_OnOff_OD_Index, &FIFO_CanTx,
                                              ENABLE_PS,
                                              RSDO + PS_NODE_ID,
                                              OD_WRITE_2BYTES);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
            }
            else if (EnablePs == false)
            {
                status = Set_CANOpenMsg_To_Tx(Ps_OnOff_OD_Index, &FIFO_CanTx,
                                              0x00,
                                              RSDO + PS_NODE_ID,
                                              OD_WRITE_2BYTES);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
            }
            else
            {
                //TODO: Set an error flag, stop everything and display it in user interface
                //TODO: State asigned for trigger "Error Machine"
                status = 0x00;
                stateEnablePs = 3;
            }
            stateEnablePs = 1;
            break;

        case 1:

            //if ((StatusCom.StatusFlags.Flags.DataAnalyzed) && (PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs))
            if ((StatusCom.StatusFlags.Flags.ErrorCom == false) &&
                (StatusErrors.StatusFlags.Flags.AccessCmdError == false) &&
                (StatusErrors.StatusFlags.Flags.SubIndexError == false) &&
                (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                if (PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs)
                {
                    //Confirmed reception of Ack for Enable Power Supply command.
                    //StatusCom.StatusFlags.Flags.DataAnalyzed = false;
                    PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs = false;
                    PowerSupplyValues.PowerModuleStatus |= 0x01; //Bit /   Description /   Value
                                                                 // 0      Ps ON  / 0: OFF; 1: ON
                    if (EnablePs == false)
                    {
                        //Switch off via GPIO signal
                        PsEnable_OFF();
                        PowerSupplyValues.PowerModuleStatus = 0x00; //Bit /   Description /   Value
                                                                    // 0      Ps ON  / 0: OFF; 1: ON
                    }
                    stateEnablePs = 0;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom &&
                     StatusErrors.StatusFlags.Flags.AccessCmdError &&
                     StatusErrors.StatusFlags.Flags.SubIndexError &&
                     StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                //TODO: State aSsigned for trigger "Error Machine"
                status = 0x00;
                stateEnablePs = 3;
            }
            break;

        case 3: //TODO: State asigned for trigger "Error Machine"
            stateEnablePs = 0;
            status = 0x00;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            //TODO: State asigned for trigger "Error Machine"
            stateEnablePs = 0;
            status = 0x00;
            break;
        }
    }
    return (status);
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
            StatusCom.StatusFlags.Flags.ErrorCom = true; //Show that a reception timeout has happened
            ulTimeOutCANRx = 0;
        }
    }
}

void InitPeripherals(void)
{
    Config_CANA(500);       //CAN Speed 500kbit/s
    Config_SCIC(115200);    //UART Speed 115200bps
    ContactorNegativoOFF(); //Sanity Check
    ContactorChargeOFF();   //Sanity Check
    ContactorPositiveOFF(); //Sanity Check
    RelayPreChargeOFF();    //Sanity Check
    RelayFanOFF();          //Sanity Check
}
/*-----------------------------------END MISCELANIOUS FUNCTIONS--------------------------------------*/

/*----------------------------------BEGIN CONTROL FUNCTIONS------------------------------------*/
/**
 * @brief Every time this function is triggered CAN messages are analyzed and we 
 * update flags, variables and we take decisions
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
            if ((AccessMode >= OD_READ) && (AccessMode <= OD_READ_1BYTES))
            {
                //Access Mode within all the range of reading
                StatusCom.StatusFlags.Flags.AccessModeRead = true;
                StatusCom.StatusFlags.Flags.AccessModeWrite = false;
            }
            else if ((AccessMode >= OD_WRITE) && (AccessMode <= OD_WRITE_1BYTES))
            {
                //Access Mode within all the range of writing
                StatusCom.StatusFlags.Flags.AccessModeRead = false;
                StatusCom.StatusFlags.Flags.AccessModeWrite = true;
            }
            else if ((AccessMode >= OD_WRITE + OD_READ) && (AccessMode <= OD_WRITE_1BYTES + OD_READ_1BYTES))
            {
                //Access Mode within all the range of reading/writing
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
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }

        else if (SubIndex == 0x01) //Iout from ADC
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
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
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
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x2100: //Enable Power Supply command

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x60)
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerEnablePs = true;
            }
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x2107: //DC Output Voltage

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x4B)
            {
                PowerSupplyValues.DCOutputVoltage = TmpValue;
                PowerSupplyValues.StatusFlags.Flags.AnswerReadVoutput = true;
            }
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x2108: //DC Output Current

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x4B)
            {
                PowerSupplyValues.DCOutputCurrrent = TmpValue;
                PowerSupplyValues.StatusFlags.Flags.AnswerReadIoutput = true;
            }
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x2109: //Output voltage set point

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x60)
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerVSet = true;
            }
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    case 0x210A: //Output current set point

        if (SubIndex == 0x00)
        {
            if (AccessMode == 0x60)
            {
                PowerSupplyValues.StatusFlags.Flags.AnswerISet = true;
            }
            else
            {
                //Requested Access Mode incorrect, send an error message to the transmitter
                StatusErrors.StatusFlags.Flags.AccessCmdError = true;
                Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                          RSDO + NodeIdRx);
            }
        }
        else
        {
            //Requested Sub-Index incorrect, send an error message to the transmitter
            StatusErrors.StatusFlags.Flags.SubIndexError = true;
            Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                      RSDO + NodeIdRx);
        }
        break;

    default:
        //Requested Object Index incorrect, send an error message to the transmitter
        StatusErrors.StatusFlags.Flags.ObjectIndexError = true;
        Set_CANOpenErrorMsg_To_Tx(DictionaryIndex, &FIFO_CanTx, 0x00,
                                  RSDO + NodeIdRx);
        break;
    }
    StatusCom.StatusFlags.Flags.DataAvailable = false; //Data received has been analyzed
    //by this function yet, it is not longer useful, then set flag to zero for the next time
    //StatusCom.StatusFlags.Flags.DataAnalyzed = true; //This flag is used by other functions that
    //are waiting some result from analyzed data received
}

/**
 * @brief 
 * 
 */
void Scheduler(void)
{
    uint16_t status = 0x01; //By default all ok
    PowerSupplyValues.RequiredOnOffProcess = true;

    Count1ms++;
    if (Count1ms == 10)
    {
        Count1ms = 0;
        Count20ms++;
        Count50ms++;
        Count100ms++;
        //Every 10ms
        TimeOutRxCanMsg();
    }
    if (Count20ms == 2)
    {
        Count20ms = 0;
        //Every 20ms
        if ((StatusCom.StatusFlags.Flags.ErrorCom != true) && (FIFO_CanRx.Estado_PILA != PILA_VACIA))
        {
            //No timeout in Rx and there are data on FIFO Rx to analyze
            AnalyzeCanMsg();
        }
        status = InitPowerSupply();
    }
    if (Count50ms == 5)
    {
        Count50ms = 0;
        Count250ms++;
        //Every 50ms
    }
    if (Count100ms == 10)
    {
        Count100ms = 0;
        Count200ms++;
        Count500ms++;
        Count1s++;
        //Every 100ms
        PsEnableDisable(PowerSupplyValues.RequiredOnOffProcess);
        PsKeepAlive();
    }
    if (Count200ms == 2)
    {
        Count200ms = 0;
        //Every 200ms
        FSM_FastCharge();
    }
    if (Count250ms == 5)
    {
        Count250ms = 0;
        //Every 250ms
        PsSetVoltageCurrent(PowerSupplyValues.RequiredVoltageValue,
                           PowerSupplyValues.RequiredCurrentValue);
        //PsSetVoltageCurrent(3000, 10);
    }
    if (Count500ms == 5)
    {
        Count500ms = 0;
        //Every 5000ms
        if ((PowerSupplyValues.PowerModuleStatus & 0x04) == 0x04) // 0      Power Supply Enable  / 0: OFF; 1: ON
                                                                  // 1      Communication / 0: OK COM; 1: ERROR COM
                                                                  // 2      Updated Value / 0: NO; 1: YES
                                                                  // 3      Initiated Power Supply  / 0: NO; 1: YES
        {
            PowerSupplyValues.PowerModuleStatus &= 0xFB; //Set to zero bit updated. We read ADC only once after
                                                         //finish updated process
            PsReadOutputVI();
        }
    }
    if (Count1s == 10)
    {
        Count1s = 0;
        Count2s++;
        Count4s++;
        //Every 1000ms
    }
    if (Count2s == 2)
    {
        Count2s = 0;
        //Every 20000ms
    }
    if (Count4s == 4)
    {
        Count4s = 0;
        //Every 40000ms
    }
}

/*  END CONTROL FUNCTIONS */

/* INIT STATE MACHINES */

/**
 * @brief State machine in charge of set power supply to Vout = Vbatt
 * to avoid sparks in contactors
 * 
 * @return uint16_t 
 */
/*
uint16_t FSM_Precharge(void)
{
    static uint16_t statePreCharge = 0;
    static uint16_t TimeOutContactorsPreChg = 0;
    uint16_t status = 0x01; //By default all ok
    static uint16_t TimeOutWaitingBus = 0;

    switch (statePreCharge)
    {
    case 0: //State for switch ON enable earth-contactor
        ContactorGndON();
        statePreCharge = 1;
        break;

    case 1: //Awaiting for steady state of earth-contactor. Minimum 130ms
        TimeOutContactorsPreChg++;
        if (TimeOutContactorsPreChg >= 2)
        {
            //>200ms elapsed
            //Contactor in steady state
            TimeOutContactorsPreChg = 0;
            statePreCharge = 2;
        }
        else
        {
            statePreCharge = 1;
        }

        break;

    case 2: //Switch On Power Supply and set voltage/current

        PowerSupplyValues.RequiredOnOffProcess = Ps_ON;
        PowerSupplyValues.RequiredVoltageValue = 4000; //Corresponds to 400V
        PowerSupplyValues.RequiredCurrentValue = 10;   //Corresponds to 1A
        statePreCharge = 3;
        break;

    case 3: //Awaiting to have final requiered voltage and current values. End ramp-up/down
        if ((PowerSupplyValues.PowerModuleStatus & 0x04) == 0x04) // 0      Power Supply Enable  / 0: OFF; 1: ON
                                                                  // 1      Communication / 0: OK COM; 1: ERROR COM
                                                                  // 2      Updated Value / 0: NO; 1: YES
                                                                  // 3      Initiated Power Supply  / 0: NO; 1: YES
        {
            //We get final values, then next state
            PowerSupplyValues.PowerModuleStatus &= 0xFB; //Clear Updated Value bit for next time
            statePreCharge = 4;
        }
        else
        {
            statePreCharge = 3;
        }
        break;

    case 4: //Cheking output power supply voltage. Compare with Vbatt
        //TODO: May it be necessary to set a maximum waiting time no more than 2 seconds. Two
        //seconds is the time defined to get ramp up/down
        if ((PowerSupplyValues.DCOutputVoltage
                < (PowerSupplyValues.RequiredVoltageValue + VOLTAGE_THRESHOLD))
                && (PowerSupplyValues.DCOutputVoltage
                        > (PowerSupplyValues.RequiredVoltageValue
                                - VOLTAGE_THRESHOLD))) //0.1V/bit then 4000/10=400V
        {
            //IPCLtoRFlagSet(IPC_FLAG6); //report to the other CPU that Bus is ready
            statePreCharge = 5;
        }
        /**else
         {
         TimeOutWaitingBus++;
         if (TimeOutWaitingBus == 50) // 10 seconds
         {
         TimeOutWaitingBus = 0;
         statePreCharge = 10;  //Go to Errors
         }
         else
         {
         statePreCharge = 4;
         }

         }

        break;

    case 5: //State to switch ON precharge_contactor, charge_contator

        ContactorPreChargeON();
        statePreCharge = 6;
        break;

    case 6: //Awaiting for steady state of precharge_contactor.
        //Minimum 130ms
        TimeOutContactorsPreChg++;
        if (TimeOutContactorsPreChg >= 2)
        {
            //>200ms elapsed
            //Contactor in steady state
            TimeOutContactorsPreChg = 0;
            statePreCharge = 7;
        }
        else
        {
            statePreCharge = 6;
        }

        break;

    case 7:
        ContactorPreChargeOFF();
        ContactorChargeON();
        statePreCharge = 8;
        break;

    case 8: //Awaiting for steady state of charge_contator.
        //Minimum 130ms
        TimeOutContactorsPreChg++;
        if (TimeOutContactorsPreChg >= 2)
        {
            //>200ms elapsed
            //Contactor in steady state
            TimeOutContactorsPreChg = 0;
            statePreCharge = 9;
        }
        else
        {
            statePreCharge = 8;
        }

        break;

    case 9: //Last state. Enable next FSM
        FsmStatus.nextFSM = FsmCharge;
        FsmStatus.previousFSM = FsmPreCharge;
        status = 0x01;
        break;

    case 10: //Error state. DC bus voltage has not been reached in less than 4 seconds
        GPIO_TogglePin(P9_10);
        PsEnable_OFF();
        PowerSupplyValues.RequiredOnOffProcess = false;
        DELAY_US(1000000);
        status = 0x00;
        break;

    default:
        status = 0x00; //Error
        break;
    }
}
*/
uint16_t FSM_FastCharge(void)
{
    static uint16_t stateFastCharge = 0;
    static uint16_t TimeOutContactorsFastCharge = 0;
    static uint16_t TimeOutWaitingBusFastChg = 0;
    static uint16_t TimeOutCharge = 0;
    uint16_t status = 0x01; //By default all ok

    if ((PowerSupplyValues.PowerModuleStatus & 0x09) == 0x09) //Bit /   Description /   Value
                                                              // 0      Ps On  / 0: OFF; 1: ON
                                                              // 1      Communication / 0: OK COM; 1: ERROR COM
                                                              // 2      Updated Value / 0: NO; 1: YES
                                                              // 3      Initialized Ps  / 0: NO; 1: YES
                                                              // 4      Updated ADC values / 0: NO; 1: YES
    {
        //Power Supply ON and Initialized
        switch (stateFastCharge)
        {
        case 0: //Setting required values of voltage and current
            //PowerSupplyValues.RequiredOnOffProcess = Ps_ON;
            PowerSupplyValues.RequiredVoltageValue = 4000; //Corresponds to 400V
            PowerSupplyValues.RequiredCurrentValue = 10;   //Minimum value. Corresponds to 1A
            stateFastCharge = 1;
            break;

        case 1: //Cheking output power supply voltage. Compare with Vbatt
            if ((PowerSupplyValues.PowerModuleStatus & 0x10) == 0x10)
            {
                //We can get values from ADC power supply and compare them with V_battery
                PowerSupplyValues.PowerModuleStatus &= 0xEF; //Reset bit for next time

                if ((PowerSupplyValues.DCOutputVoltage < (PowerSupplyValues.RequiredVoltageValue + VOLTAGE_THRESHOLD)) &&
                    (PowerSupplyValues.DCOutputVoltage > (PowerSupplyValues.RequiredVoltageValue - VOLTAGE_THRESHOLD)))
                {
                    //IPCLtoRFlagSet(IPC_FLAG6); //report to the other CPU that Bus is ready
                    ContactorNegativoON();
                    stateFastCharge = 2;
                }
            }
            else
            {
                TimeOutWaitingBusFastChg++;
                if (TimeOutWaitingBusFastChg >= 25) //5 seconds maximun waiting time reached
                {
                    TimeOutWaitingBusFastChg = 0;
                    stateFastCharge = 6; //Goto error machine
                }
                else
                {
                    //It is all ok
                    stateFastCharge = 1;
                }
            }
            break;

        case 2: //Awaiting for steady state of negative contactor
            //Minimum 130ms
            TimeOutContactorsFastCharge++;
            if (TimeOutContactorsFastCharge >= 2)
            {
                //>200ms elapsed
                //Contactor in steady state
                TimeOutContactorsFastCharge = 0;
                ContactorChargeON();
                RelayFanON();
                stateFastCharge = 3;
            }
            else
            {
                stateFastCharge = 2;
            }
            break;

        case 3: //Awaiting for steady state of diode contactor
            //Minimum 130ms
            TimeOutContactorsFastCharge++;
            if (TimeOutContactorsFastCharge >= 2)
            {
                //>200ms elapsed
                //Contactor in steady state
                TimeOutContactorsFastCharge = 0;
                stateFastCharge = 4;
            }
            else
            {
                stateFastCharge = 2;
            }
            break;

        case 4:                                          //Setting charge current value
            PowerSupplyValues.RequiredCurrentValue = 50; //Corresponds to 5A
            stateFastCharge = 5;
            break;

        case 5: //Check output current from power supply

            if ((PowerSupplyValues.PowerModuleStatus &= 0x10) == 0x10)
            {
                //We can get values from ADC power supply and compare them with I_request
                PowerSupplyValues.PowerModuleStatus &= 0xEF; //Reset bit for next time

                if ((PowerSupplyValues.DCOutputCurrrent < (PowerSupplyValues.RequiredCurrentValue + CURRENT_THRESHOLD)) &&
                    (PowerSupplyValues.DCOutputCurrrent > (PowerSupplyValues.RequiredCurrentValue - CURRENT_THRESHOLD)))
                {
                    TimeOutCharge++;
                    if (TimeOutCharge >= 1200) //4 minutes
                    {
                        TimeOutCharge = 0;
                    }
                    else
                    {
                        stateFastCharge = 5;
                    }
                }
            }
            else
            {
                TimeOutWaitingBusFastChg++;
                if (TimeOutWaitingBusFastChg >= 25) //5 seconds maximun waiting time reached
                {
                    TimeOutWaitingBusFastChg = 0;
                    stateFastCharge = 6; //Goto error machine
                }
                else
                {
                    stateFastCharge = 1;
                }
            }
            break;

        case 6: //Error state. Error machine
            GPIO_TogglePin(P9_10);
            PsEnable_OFF();
            PowerSupplyValues.RequiredOnOffProcess = false;
            DELAY_US(1000000);
            status = 0x00;
            stateFastCharge = 6;
            break;

        default:
            status = 0x00;
            break;
        }
    }
}
/**
 * @brief 
 * 
 * @return uint16_t 
 */
/*
uint16_t FSM_Charge(void)
{
    static uint16_t stateCharge = 0;
    static uint16_t TimeOutContactorsChg = 0;
    static uint16_t TimeOutCharge = 0;
    static uint16_t TimeOutWaitingBusChg = 0;
    static uint16_t TimeOutUpdatedValuesChg = 0;
    uint16_t status = 0x01; //By default all ok

    switch (stateCharge)
    {
    case 0: //Set current and voltage values
        PowerSupplyValues.RequiredOnOffProcess = Ps_ON;
        PowerSupplyValues.RequiredVoltageValue = 5000; //Corresponds to 500V
        PowerSupplyValues.RequiredCurrentValue = 0;    //Corresponds to 0
        stateCharge = 1;
        break;

    case 1: //Awaiting to have final requiered voltage and current values. End ramp-up/down
        //TODO: May it be necessary to set a maximum waiting time no more than 2 seconds. Two
        //seconds is the time defined to get ramp up/down
        if ((PowerSupplyValues.PowerModuleStatus & 0x04) == 0x04) // 0      Power Supply Enable  / 0: OFF; 1: ON
                                                                  // 1      Communication / 0: OK COM; 1: ERROR COM
                                                                  // 2      Updated Value / 0: NO; 1: YES
                                                                  // 3      Initiated Power Supply  / 0: NO; 1: YES
        {
            PowerSupplyValues.PowerModuleStatus &= 0xFB; //Clear Updated Value bit for next time
            stateCharge = 2;              //We get final values, then next state
        }
        else
        {
            //TimeOutUpdatedValuesChg++;
            // if(TimeOutUpdatedValuesChg >= 20) //4000ms
            // {
            //     TimeOutUpdatedValuesChg=0;
            //     stateCharge = 6; //Go to Errors
            // }
            // else
            //  {
            stateCharge = 1;
            //  }
        }

        break;

    case 2: //Check tolerance in voltage and current
        if ((PowerSupplyValues.DCOutputVoltage
                < (PowerSupplyValues.RequiredVoltageValue + VOLTAGE_THRESHOLD))
                && (PowerSupplyValues.DCOutputVoltage
                        > (PowerSupplyValues.RequiredVoltageValue
                                - VOLTAGE_THRESHOLD))
                && (PowerSupplyValues.DCOutputCurrrent
                        < (PowerSupplyValues.RequiredCurrentValue
                                + CURRENT_THRESHOLD))
                && (PowerSupplyValues.DCOutputCurrrent
                        > (PowerSupplyValues.RequiredCurrentValue
                                - CURRENT_THRESHOLD)))
        {
            PowerSupplyValues.RequiredCurrentValue = 50; //50/10=5A / 0.1A/bit
            stateCharge = 3;
        }
        else
        {
            TimeOutWaitingBusChg++;
            if (TimeOutWaitingBusChg >= 20) //4000ms
            {
                TimeOutWaitingBusChg = 0;
                stateCharge = 6; //Go to Error
            }
            else
            {
                stateCharge = 2;
            }
        }
        break;

    case 3:  //Awaiting to have final requiered current values. End ramp-up/down
        if ((PowerSupplyValues.PowerModuleStatus & 0x04) == 0x04) // 0      Power Supply Enable  / 0: OFF; 1: ON
                                                                  // 1      Communication / 0: OK COM; 1: ERROR COM
                                                                  // 2      Updated Value / 0: NO; 1: YES
                                                                  // 3      Initiated Power Supply  / 0: NO; 1: YES
        {
            PowerSupplyValues.PowerModuleStatus &= 0xFB; //Clear Updated Value bit for next time
            stateCharge = 4;              //We get final values, then next state
        }
        else
        {
            //TimeOutUpdatedValuesChg++;
            //if(TimeOutUpdatedValuesChg >= 20) //4000ms
            //{
            //    TimeOutUpdatedValuesChg=0;
            //    stateCharge = 6; //Go to Errors
            //}
            //else
            //{
            stateCharge = 3;
            //}
        }
        break;

    case 4: //Check tolerance in current
        if ((PowerSupplyValues.DCOutputCurrrent
                < (PowerSupplyValues.RequiredCurrentValue + CURRENT_THRESHOLD))
                && (PowerSupplyValues.DCOutputCurrrent
                        > (PowerSupplyValues.RequiredCurrentValue
                                - CURRENT_THRESHOLD)))
        {
            // if(IpcRegs.IPCFLG.bit.IPC2) //Flag related to "Stop/End process"
            // {

            // }
            TimeOutCharge++;
            if (TimeOutCharge >= 60) //2 minutes
            {
                TimeOutCharge = 0;
                PowerSupplyValues.RequiredOnOffProcess = Ps_OFF; //Send signal to switch off power
                stateCharge = 5;
            }
            else
            {
                stateCharge = 4;
            }
        }
        else
        {
            TimeOutWaitingBusChg++;
            if (TimeOutWaitingBusChg >= 20) //4000ms
            {
                TimeOutWaitingBusChg = 0;
                stateCharge = 6; //Go to Error
            }
            else
            {
                stateCharge = 4;
            }
        }
        break;

    case 5: //Check output current/voltage before open contators
        if ((PowerSupplyValues.DCOutputCurrrent < 5)
                && (PowerSupplyValues.DCOutputVoltage < 10))
        {
            //Voltage/current are less than 1V/0.5A
            ContactorChargeOFF();
            stateCharge = 0; //Initialize state machine for next time
            FsmStatus.nextFSM = NoneFSM;
        }
        else
        {
            TimeOutCharge++;
            if (TimeOutCharge >= 900) //3 minute
            {
                TimeOutCharge = 0;
                stateCharge = 6;
            }
            stateCharge = 5;
        }
        break;

    case 6: //Error state. Requested voltage/current values exceed tolerances
        //Error machine
        GPIO_TogglePin(P9_10);
        PsEnable_OFF();
        PowerSupplyValues.RequiredOnOffProcess = false;
        DELAY_US(1000000);
        status = 0x00;
        break;

    default:
        status = 0x00;
        break;
    }
}
*/
/* END STATE MACHIBES */
