/**
 * \file ADC.c
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
#include "ADC.h"

AdcValues_t AdcValuesSaved = { 0, 0, 0, 0, 0, 0, 0 };
/**
 * @brief Enable/Disable ADC PCB function. When EnableAdc param is
 * true we sent Enable commnad via CAN and we wait answers in the form of voltage 
 * and current values from ADC, in the other hand when EnableAdc param is false
 * we sent Disable command and we wait an ACK from ADC 
 * 
 * @param EnableAdc 
 * @return uint16_t 
 */
uint16_t AdcSetEnableDisable(bool EnableAdc)
{
    static uint16_t stateEnableDisableAdc = 0;
    uint16_t status = 0x01; //By default all ok
    static enum Indice_Diccionario_TPO Adc_EnableDisable_OD_Index;

    //TODO: When fixed problem with ADcChkCom uncomment this  
    //if(((AdcValuesSaved.AdcModuleStatus & 0x01) != (EnableAdc))&&
    //((AdcValuesSaved.AdcModuleStatus & 0x08) == 0x08))
    if ((AdcValuesSaved.AdcModuleStatus & 0x01) != (EnableAdc))
    {
        //We are here because the previous state of ADC has changed and  
        // Adc PCB has been initiated 
        switch (stateEnableDisableAdc)
        {
        case 0:

            Adc_EnableDisable_OD_Index = Config_ADC; //CAN command array Index. Commands present in Diccionario_CANOpen.c file     

            if (EnableAdc == true)
            {
                status = Set_CANOpenMsg_To_Tx((uint16_t)Adc_EnableDisable_OD_Index,
                                              &FIFO_CanTx,
                                              ENABLE_ADC,
                                              RSDO + ADC_NODE_ID, 0);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
                stateEnableDisableAdc = 1;
            }
            else if (EnableAdc == false)
            {
                status = Set_CANOpenMsg_To_Tx((uint16_t)Adc_EnableDisable_OD_Index,
                                              &FIFO_CanTx,
                                              DISABLE_ADC,
                                              RSDO + ADC_NODE_ID, 0);
                if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
                {
                    status = Desencolar_FIFO(&FIFO_CanTx);
                    Transmit_CANOPenMsg(FIFO_CanTx);
                }
                stateEnableDisableAdc = 1;
            }

            break;

        case 1: //Check if we have a TimeOut reception from ADC, if so,
            //set an error
            if ((StatusCom.StatusFlags.Flags.ErrorCom == false)
                    && (StatusErrors.StatusFlags.Flags.AccessCmdError == false)
                    && (StatusErrors.StatusFlags.Flags.SubIndexError == false)
                    && (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                if ((EnableAdc == true)
                        && ((AdcValuesSaved.StatusFlags.Flags.VoltageAnswerFromAdc)
                                && (AdcValuesSaved.StatusFlags.Flags.CurrentAnswerFromAdc)))
                {

                    AdcValuesSaved.StatusFlags.Flags.VoltageAnswerFromAdc =
                            false;
                    AdcValuesSaved.StatusFlags.Flags.CurrentAnswerFromAdc =
                            false;
                    AdcValuesSaved.AdcModuleStatus |= 0x01;
                    stateEnableDisableAdc = 0;
                    status = 0x00;
                }
                else if ((EnableAdc == false)
                        && (AdcValuesSaved.StatusFlags.Flags.DisableAnswerFromADC))
                {
                    AdcValuesSaved.AdcModuleStatus &= 0xFE;
                    stateEnableDisableAdc = 0;
                    status = 0x00;
                }
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom
                    || StatusErrors.StatusFlags.Flags.AccessCmdError
                    || StatusErrors.StatusFlags.Flags.SubIndexError
                    || StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index        
                stateEnableDisableAdc = 2;
            }
            break;

        case 2: //TODO: State assigned for trigger "Error Machine" 
            status = 0x00;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            status = 0x00;
            break;
        }
    }
    return (status);
}

/**
 * @brief Check communication with ADC
 * 
 * @return uint16_t : 0x01 all ok, all is working as it should
 *                    0x00 error, there is a problem with ADC PCB
 */
uint16_t AdcCheckCom(void)
{
    static uint16_t stateChkCom = 0;
    uint16_t status = 0x00; //By default, not ok
    static enum Indice_Diccionario_TPO Adc_Chk_OD_Index;

    if ((AdcValuesSaved.AdcModuleStatus & 0x08) != 0x08)
    //Bit /   Description /  Value
    // 0      Adc PCB Enable / 0: OFF; 1: ON
    // 1      Communication Adc PCB / 0: OK COM; 1: ERROR COM
    // 2      Updated Value Adc PCB / 0: NO; 1: YES
    // 3      Initiated Adc PCB / 0: NO; 1: YES
    {
        //execute only once
        switch (stateChkCom)
        {
        case 0:

            Adc_Chk_OD_Index = ChkCom_ADC;
            status = Set_CANOpenMsg_To_Tx((uint16_t)Adc_Chk_OD_Index, &FIFO_CanTx,
            CHKCOM_ADC,
                                          RSDO + ADC_NODE_ID, 0);
            if (FIFO_CanTx.Estado_PILA != PILA_VACIA)
            {
                status = Desencolar_FIFO(&FIFO_CanTx);
                Transmit_CANOPenMsg(FIFO_CanTx);
            }
            stateChkCom = 1;
            break;

        case 1:
            //Check if we have a TimeOut reception from ADC, if so,
            //set an error
            if ((StatusCom.StatusFlags.Flags.ErrorCom == false)
                    && (StatusErrors.StatusFlags.Flags.AccessCmdError == false)
                    && (StatusErrors.StatusFlags.Flags.SubIndexError == false)
                    && (StatusErrors.StatusFlags.Flags.ObjectIndexError == false))
            {
                //all ok
                if (AdcValuesSaved.StatusFlags.Flags.ChkComAnswerFromAdc == true)
                {
                    status = 0x01;
                    stateChkCom = 0;
                    AdcValuesSaved.AdcModuleStatus &= 0xFE; //Clear bit "Communication Adc PCB"
                                                            //beacuse there are not errors in communication
                    AdcValuesSaved.AdcModuleStatus |= 0x08; //and set bit "Initiated Adc PCB" to inform
                                                            //to the system to the status of Adc PCB
                }
                else
                    stateChkCom = 1;
            }
            else if (StatusCom.StatusFlags.Flags.ErrorCom
                    || StatusErrors.StatusFlags.Flags.AccessCmdError
                    || StatusErrors.StatusFlags.Flags.SubIndexError
                    || StatusErrors.StatusFlags.Flags.ObjectIndexError)
            {
                //If we are here we had a timeout in rx, incorrect command, access mode or
                //sub-index
                AdcValuesSaved.AdcModuleStatus &= 0x02; //Set bit "Communication Adc PCB"
                                                        //beacuse there are errors in communication
                stateChkCom = 2;
            }
            break;

        case 2: //TODO: State assigned for trigger "Error Machine"
            status = 0x00;
            break;

        default:
            //TODO: Set an error flag, stop everything and display it in user interface
            status = 0x00;
            break;
        }
    }
    return (status);
}

void AdcEnableDisable(bool EnableAdc)
{
    AdcValuesSaved.RequiredOnOffProcess = EnableAdc;
}

