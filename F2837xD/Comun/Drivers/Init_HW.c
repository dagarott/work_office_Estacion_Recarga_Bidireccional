/*
 * Init_HW.c
 *
 *  Created on: 18 abr. 2018
 *      Author: jeniher
 */


// Included Files
//
#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "DEF_Global.h"
#include "Inc_Drivers.h"


void Init_HW ()
{

//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xS_SysCtrl.c file.
//
    InitSysCtrl();

#ifdef CPU2
    // Sincronizaci�n de arranque CPU1
    // Esperamos a que se configure la cpu1
    //IPCLtoRFlagSet (IPC17);
    while (IpcRegs.IPCSTS.bit.IPC17 == 0) ;
    //
    // Inciar relojes
    //
    // CANB
    CpuSysRegs.PCLKCR10.bit.CAN_B = 1;

#endif   
    // Configurar Timer 0
    //
    // Configure CPU-Timer 0 to __interrupt every 1 milliseconds:
    // 200MHz CPU Freq, 1 millisecond Period (in uSeconds)
    //
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, (T_clk/1000000) , 10000);
    //
    // To ensure precise timing, use write-only instructions to write to the entire
    // register. Therefore, if any of the configuration bits are changed in
    // ConfigCpuTimer and InitCpuTimers (in F2837xD_cputimervars.h), the below
    // settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4001;

// Step 2. Inicializar GPIO:
// This example function is found in the F2837xS_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//s
#ifdef CPU1

    Config_GPIO ();
    EALLOW;
    // Toma el control of CAN_B to CPU2
    DevCfgRegs.CPUSEL8.bit.CAN_B = 1;
    EDIS;

// Sincronizaci�n de arranque CPU1
// Indicamos que se ha configurado la cpu1
    IPCLtoRFlagSet (IPC_FLAG17);
#endif
}


void Hablitar_ISR ()
{
//
//  Clear all __interrupts and initialize PIE vector table:
//  Disable CPU __interrupts
//
    DINT;

//
// Initialize PIE control registers to their default state.
// The default state is all PIE __interrupts disabled and flags
// are cleared.
// This function is found in the F2837xS_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU __interrupts and clear all CPU __interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the __interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xS_DefaultIsr.c.
// This function is found in F2837xS_PieVect.c.
//
    InitPieVectTable();

//
// Activamos las interrupciones del CPU1
//
//  Memoria Compartida
#ifdef CPU1

    IER |= M_INT1;                         // CPU1 Timer0 Interrupt INT1.y
    IER |= M_INT9;                         // CPU1 CANB_0 Interrupt INT9.y

    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;     // CPU1 Timer0 Interrupt INTx.7
    //PieCtrlRegs.PIEIER9.bit.INTx7 = 1;   // CPU1 CANB_0 Interrupt INTx.7
    PieCtrlRegs.PIEIER9.bit.INTx5 = 1;     // CPU1 CANA_0 Interrupt INTx.5
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;     // CPU1 SCI_RX Interrupt INTx.5

#endif

    //
    // Activamos las interrupciones del CPU2
    //
#ifdef CPU2

    //PieCtrlRegs.PIEIER1.bit.INTx13 = 1;   // CPU1 to CPU2 INT0
    //PieCtrlRegs.PIEIER1.bit.INTx14 = 1;   // CPU1 to CPU2 INT0


#endif

//
// Enable global Interrupts and higher priority real-time debug events:
//
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM}

}
