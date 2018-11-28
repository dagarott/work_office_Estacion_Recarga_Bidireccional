/**
 ***************************************************************************
 \defgroup       DRIVERS

 \brief          Modulo de inicializacion de los perifericos del
 TMS320F2877s

 Microcontroladores en los que se ha probado:
 TMS320F2877s

 **************************************************************************/
/**
 ***************************************************************************
 \file           CONFIG_CAN.c
 \brief          Modulo de configuracion de los CAN del microcontrolador
 TMS320F2877s.

 \author         Jesus Nieto Hervas
 \version        V0.0
 \date           16/05/2018
 **************************************************************************/

//
// Included
//
#include "F28x_Project.h"
#include "Inc_Drivers.h"
#include "DEF_Global.h"

#include <stdint.h>
#include <stdbool.h>

//
// Defines
//

//
// Estructuras
//

uint16_t BRR;

/**
 ***************************************************************************
 \fn         Transmitir_SCI
 \brief      Funcion para transmitir por el SCI.

 \param[in]  SCIx numero de SCI por el que transmitir
 \return     void

 **************************************************************************/
void Transmitir_SCI(tNumSCI SCIx, uint16_t Data)
{
    switch (SCIx)
    {
    case (SCI_A):
    {
        // Esperamos que se envie los datos
        while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
            ;

        SciaRegs.SCITXBUF.all = Data;

        // Esperamos a que este todo correcto antes de enviar otra trama
        //while (SciaRegs.SCICTL2.bit.TXRDY != 1) {}
        break;
    }
    case (SCI_B):
    {
        // Esperamos que se envie los datos
        while (ScibRegs.SCIFFTX.bit.TXFFST != 0)
            ;

        ScibRegs.SCITXBUF.all = Data;

        // Esperamos a que este todo correcto antes de enviar otra trama
        while (ScibRegs.SCICTL2.bit.TXRDY != 1)
        {
        }
        break;
    }
    case (SCI_C):
    {
        // Esperamos que se envie los datos
        while (ScicRegs.SCIFFTX.bit.TXFFST != 0)
            ;

        ScicRegs.SCITXBUF.all = Data;

        // Esperamos a que este todo correcto antes de enviar otra trama
        while (ScicRegs.SCICTL2.bit.TXRDY != 1)
        {
        }
        break;
    }
    }            //FIN switch
}

/**
 ***************************************************************************
 \fn         Config_SCIB
 \brief      Funcion principal de configuracion del SCIA.

 \param[in]  BaudRate in Bauds
 \return     void

 **************************************************************************/
void Config_SCIA(uint32_t BaudRate)
{
    //-------------------------------------------------------------------------
    //    Setup SCICCR
    //-------------------------------------------------------------------------
    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  STOPBITS            | Registro que selecciona  |  00h - 1 stop bit  |
     * |  (7)                 |el numero de bits para el |  01h - 2 stop bit  |
     * |                      |stop.                     |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.STOPBITS = 0x00; // 1 stop bit

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  PARITY              | Registro que selecciona  |  00h - Odd parity  |
     * |  (6)                 |si los datos transmitidos |  01h - Even parity |
     * |                      |son pares o impares.      |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.PARITY = 0x00; // No parity

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  PARITYENA           | Registro que selecciona  |  00h - Parity      |
     * |  (5)                 |si habilita el registro   |      disable       |
     * |                      |PARITY.                   |  01h - Parity      |
     * |                      |                          |      enable        |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.PARITYENA = 0x00; // No parity

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  LOOPBKENA           | Registro que habilita    |  00h - Loop Back   |
     * |  (4)                 |el modo de test.          |      disable       |
     * |                      |                          |  01h - Loop Back   |
     * |                      |                          |      enable        |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.LOOPBKENA = 0x00; // Loop Back Disable

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  ADDRIDDLE_MODE      | Registro de control de   |  00h - Idele-line  |
     * |  (3)                 |modo multiprocesador SCI. |      mode.         |
     * |                      |SCICTL1 bit2 y            |                    |
     * |                      |SCICTL2 bit3              |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0x00; // idle-line protocol

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  SCICHAR             | Registro que indica el   |  00h -             |
     * |  (2 - 0)             |tamaño del caracter       | SCIACHAR_LEGNTH_1  |
     * |                      |recibido.                 |  01h -             |
     * |                      |                          | SCIACHAR_LEGNTH_2  |
     * |                      |                          |  02h -             |
     * |                      |                          | SCIACHAR_LEGNTH_3  |
     * |                      |                          |  ......            |
     * |                      |                          |  07h -             |
     * |                      |                          | SCIACHAR_LEGNTH_8  |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICCR.bit.SCICHAR = 0x07; //SCIACHAR_LEGNTH_8

    //-------------------------------------------------------------------------
    //    Setup SCICTL1
    //-------------------------------------------------------------------------
    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXERRINTENA         | Registro habilita la     |  00h - INT disable |
     * |  (6)                 |interrupcion en caso de   |  01h - INT enable  |
     * |                      |error.                    |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL1.bit.RXERRINTENA = 0x00; // Disable interrupt error

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  SWRESET             | Registro habilita el     |  00h - Inicializa  |
     * |  (5)                 |reset por software.       |las maquinas de     |
     * |                      |                          |estado SCI.         |
     * |                      |                          |  01h - Despues de  |
     * |                      |                          |reiniciar el sistema|
     * |                      |                          |poner a 1 este bit. |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL1.bit.SWRESET = 0x00; // Reset by Software

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXWAKE              | Registro selecciona el   |  00h               |
     * |  (3)                 |metodo de activacion de   |  01h               |
     * |                      |la transmision SCI.       |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    /*
     0h (R / W) = La función de transmisión no está seleccionada. En modo inactivo: escriba un
     1 a TXWAKE, luego escriba los datos para registrar SCITXBUF para generar un
     período de inactividad de 11 bits de datos En modo de bit de dirección: escriba un 1 en
     TXWAKE, luego escriba los datos en SCITXBUF para establecer el bit de dirección para
     ese marco a 1
     1h (R / W) = La función de transmisión seleccionada depende del modo,
     línea inactiva o bit de dirección: TXWAKE no se borra mediante el SW RESET
     bit (SCICTL1, bit 5)
     se borra mediante un reinicio del sistema o la transferencia de TXWAKE al
     Bandera WUT*/

    SciaRegs.SCICTL1.bit.TXWAKE = 0x00;

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  SLEEP               | Registro que depende     | 00h - sleep disable|
     * |  (2)                 |del registro anterior.    | 01h - sleep enable |
     * |                      |                          |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL1.bit.SLEEP = 0x00; // Disable sleep mode

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXENA               | Registro habilita la     |  00h - TX  disable |
     * |  (1)                 |transmision de datios por |  01h - TX  enable  |
     * |                      |el SCI.                   |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL1.bit.TXENA = 0x01; // Enable TX

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXENA               | Registro habilita la     |  00h - RX  disable |
     * |  (0)                 |recepcion de datios por   |  01h - RX  enable  |
     * |                      |el SCI.                   |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL1.bit.RXENA = 0x01; // Enable RX

    //-------------------------------------------------------------------------
    //    Setup SCICTL2
    //-------------------------------------------------------------------------
    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXRDY               | Registro que indica que  |  00h - TX Full     |
     * |  (7)                 |el SCI esta listo para    |  01h - TX Ready    |
     * |                      |transmitir.               |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL2.bit.TXRDY = 0x00; // Resgistro de lectura para enviar la
                                       //siguiente trama de datos.

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXEMPTY             | Registro que indica que  |  00h - FIFO data   |
     * |  (6)                 |fifo de la SCI esta vacia |  01h - FIFO empty  |
     * |                      |o tiene datos recibidos.  |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL2.bit.TXEMPTY = 0x00; // Resgistro de lectura conocer estado fifo.

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXBKINTENA          | Registro que habilita la |  00h - INT disable |
     * |  (1)                 |interrupcion a la hora de |  01h - INT enable  |
     * |                      |recibir una trama.        |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0x01; // Enable INT RX

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXINTENA            | Registro que habilita la |  00h - INT disable |
     * |  (0)                 |interrupcion a la hora de |  01h - INT enable  |
     * |                      |transmitir una trama.     |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCICTL2.bit.TXINTENA = 0x00; // Disable INT TX

    //-------------------------------------------------------------------------
    //    Setup SCIFFTX
    //-------------------------------------------------------------------------
    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  SCIRST              | Registro que reset la    |  00h - Reset       |
     * |  (15)                |fifo.                     |  01h - SCI FIFO OK |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFTX.bit.SCIRST = 0x01; // Reset SCI

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  SCIFFENA            | Registro que habilita la |  00h - FIFO disable|
     * |  (14)                |fifo en la transmision de |  01h - FIFO enable |
     * |                      |datos.                    |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFTX.bit.SCIFFENA = 0x01; //Enable FIFO

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXFIFORESET         | Registro que reset la    |  00h - FIFO reset  |
     * |  (13)                |fifo en la transmision de |  01h - FIFO        |
     * |                      |datos.                    |        re-enable   |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 0x00; //reset FIFO

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXFFIENA            | Registro que habilita la |  00h -             |
     * |  (5)                 |interrupcion al transmitir|TX FIFO disable.    |
     * |                      |una trama,                |  01h -             |
     * |                      |                          |TX FIFO enable.     |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFTX.bit.TXFFIENA = 0x00; //Registro de lectura

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  TXFFIL              | Registro que indica el   |                    |
     * |  (4 - 0)             |nivle de bits en la INT de|                    |
     * |                      |la fifo.                  |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFTX.bit.TXFFIL = 0x00; //SIN INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFRX
    //-------------------------------------------------------------------------
    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXFIFORESET         | Registro que reset la    |  00h - FIFO reset  |
     * |  (13)                |fifo en la recepcion   de |  01h - FIFO        |
     * |                      |datos.                    |        re-enable   |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0x00; //reset FIFO

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXFFIENA            | Registro que habilita la |  00h -             |
     * |  (5)                 |interrupcion al recibir   |RX FIFO disable.    |
     * |                      |una trama,                |  01h -             |
     * |                      |                          |RX FIFO enable.     |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFRX.bit.RXFFIENA = 0x01; //RX FIFO Enable

    /* |----------------------|--------------------------|--------------------|
     * |    NAME BIT (BITS)   |       Descripcion        |       Valores      |
     * |----------------------|--------------------------|--------------------|
     * |  RXFFIL              | Registro que indica el   |                    |
     * |  (4 - 0)             |nivel de bits en la INT de|                    |
     * |                      |la fifo.                  |                    |
     * |----------------------|--------------------------|--------------------|
     *
     */
    SciaRegs.SCIFFRX.bit.RXFFIL = 0x02; //CON INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFCT
    //-------------------------------------------------------------------------
    /*
     * Configuracion del AutoBaud
     */
    SciaRegs.SCIFFCT.all = 0x00;

    //-------------------------------------------------------------------------
    //    Setup BAUD
    //-------------------------------------------------------------------------
    /*
     * Configuracion del Baud
     */
    BRR = ((T_clk / 4) / (BaudRate * 8)) - 1;

    SciaRegs.SCILBAUD.bit.BAUD = BRR & 0x00FF;

    SciaRegs.SCIHBAUD.bit.BAUD = (BRR >> 8) & 0x00FF;

    //Reset SCIA
    SciaRegs.SCICTL1.bit.SWRESET = 1;
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
} // FIN void Config_SCIA

/**
 ***************************************************************************
 \fn         Config_SCIB
 \brief      Funcion principal de configuracion del SCIB.

 \param[in]  BaudRate in Bauds
 \return     void

 **************************************************************************/
void Config_SCIB(uint32_t BaudRate)
{
    //-------------------------------------------------------------------------
    //    Setup SCICCR
    //-------------------------------------------------------------------------
    ScibRegs.SCICCR.bit.STOPBITS = 0x00; // 1 stop bit
    ScibRegs.SCICCR.bit.PARITY = 0x00; // No parity
    ScibRegs.SCICCR.bit.PARITYENA = 0x00; // No parity
    ScibRegs.SCICCR.bit.LOOPBKENA = 0x00; // Loop Back Disable
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0x00; // idle-line protocol
    ScibRegs.SCICCR.bit.SCICHAR = 0x07; //SCIACHAR_LEGNTH_8

    //-------------------------------------------------------------------------
    //    Setup SCICTL1
    //-------------------------------------------------------------------------
    ScibRegs.SCICTL1.bit.RXERRINTENA = 0x00; // Disable interrupt error
    ScibRegs.SCICTL1.bit.SWRESET = 0x00; // Reset by Software
    ScibRegs.SCICTL1.bit.TXWAKE = 0x00;
    ScibRegs.SCICTL1.bit.SLEEP = 0x00; // Disable sleep mode
    ScibRegs.SCICTL1.bit.TXENA = 0x01; // Enable TX
    ScibRegs.SCICTL1.bit.RXENA = 0x01; // Enable RX

    //-------------------------------------------------------------------------
    //    Setup SCICTL2
    //-------------------------------------------------------------------------
    ScibRegs.SCICTL2.bit.RXBKINTENA = 0x01; // Enable INT RX
    ScibRegs.SCICTL2.bit.TXINTENA = 0x00; // Disable INT TX

    //-------------------------------------------------------------------------
    //    Setup SCIFFTX
    //-------------------------------------------------------------------------
    ScibRegs.SCIFFTX.bit.SCIRST = 0x01; // Reset SCI
    ScibRegs.SCIFFTX.bit.SCIFFENA = 0x01; //Enable FIFO
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 0x00; //reset FIFO
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0x00; //Registro de lectura
    ScibRegs.SCIFFTX.bit.TXFFIL = 0x00; //SIN INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFRX
    //-------------------------------------------------------------------------
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 0x00; //reset FIFO
    ScibRegs.SCIFFRX.bit.RXFFIENA = 0x01; //RX FIFO Enable
    ScibRegs.SCIFFRX.bit.RXFFIL = 0x02; //CON INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFCT
    //-------------------------------------------------------------------------
    ScibRegs.SCIFFCT.all = 0x00;

    //-------------------------------------------------------------------------
    //    Setup BAUD
    //-------------------------------------------------------------------------
    /*
     * Configuracion del Baud
     */
    BRR = ((T_clk / 4) / (BaudRate * 8)) - 1;

    ScibRegs.SCILBAUD.bit.BAUD = BRR & 0x00FF;

    ScibRegs.SCIHBAUD.bit.BAUD = (BRR >> 8) & 0x00FF;
    //Reset SCIA
    ScibRegs.SCICTL1.bit.SWRESET = 1;
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
}
// FIN void Config_SCIB

/**
 ***************************************************************************
 \fn         Config_SCIC
 \brief      Funcion principal de configuracion del SCIA.

 \param[in]  BaudRate in Bauds
 \return     void

 **************************************************************************/
void Config_SCIC(uint32_t BaudRate)
{
    /*
     ScicRegs.SCIFFTX.all = 0xE040;
     ScicRegs.SCIFFRX.all = 0x2044;
     ScicRegs.SCIFFCT.all = 0x0;
     ScicRegs.SCICCR.all = 0x0007;   // 1 stop bit,  No loopback
     // No parity,8 char bits,
     // async mode, idle-line protocol
     ScicRegs.SCICTL1.all = 0x0003;  // enable TX, RX, internal SCICLK,
     // Disable RX ERR, SLEEP, TXWAKE
     ScicRegs.SCICTL2.all = 0x0003;
     ScicRegs.SCICTL2.bit.TXINTENA = 0;
     ScicRegs.SCICTL2.bit.RXBKINTENA = 1;

     */
    //-------------------------------------------------------------------------
    //    Setup SCICCR
    //-------------------------------------------------------------------------
    ScicRegs.SCICCR.bit.STOPBITS = 0x00; // 1 stop bit
    ScicRegs.SCICCR.bit.PARITY = 0x00; // No parity
    ScicRegs.SCICCR.bit.PARITYENA = 0x00; // No parity
    ScicRegs.SCICCR.bit.LOOPBKENA = 0x00; // Loop Back Disable
    ScicRegs.SCICCR.bit.ADDRIDLE_MODE = 0x00; // idle-line protocol
    ScicRegs.SCICCR.bit.SCICHAR = 0x07; //SCIACHAR_LEGNTH_8

    //-------------------------------------------------------------------------
    //    Setup SCICTL1
    //-------------------------------------------------------------------------
    ScicRegs.SCICTL1.bit.RXERRINTENA = 0x00; // Disable interrupt error
    ScicRegs.SCICTL1.bit.SWRESET = 0x00; // Reset by Software
    ScicRegs.SCICTL1.bit.TXWAKE = 0x00;
    ScicRegs.SCICTL1.bit.SLEEP = 0x00; // Disable sleep mode
    ScicRegs.SCICTL1.bit.TXENA = 0x01; // Enable TX
    ScicRegs.SCICTL1.bit.RXENA = 0x01; // Enable RX

    //-------------------------------------------------------------------------
    //    Setup SCICTL2
    //-------------------------------------------------------------------------
    ScicRegs.SCICTL2.bit.RXBKINTENA = 0x01; // Enable INT RX
    ScicRegs.SCICTL2.bit.TXINTENA = 0x00; // Disable INT TX

    //-------------------------------------------------------------------------
    //    Setup SCIFFTX
    //-------------------------------------------------------------------------
    ScicRegs.SCIFFTX.bit.SCIRST = 0x01; // Reset SCI
    ScicRegs.SCIFFTX.bit.SCIFFENA = 0x01; //Enable FIFO
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 0x00; //reset FIFO
    ScicRegs.SCIFFTX.bit.TXFFIENA = 0x00; //Registro de lectura
    ScicRegs.SCIFFTX.bit.TXFFIL = 0x00; //SIN INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFRX
    //-------------------------------------------------------------------------
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 0x00; //reset FIFO
    ScicRegs.SCIFFRX.bit.RXFFIENA = 0x01; //RX FIFO Enable
    ScicRegs.SCIFFRX.bit.RXFFIL = 0x02; //CON INT

    //-------------------------------------------------------------------------
    //    Setup SCIFFCT
    //-------------------------------------------------------------------------
    ScicRegs.SCIFFCT.all = 0x00;

    //-------------------------------------------------------------------------
    //    Setup BAUD
    //-------------------------------------------------------------------------

    //Configuracion del Baud

    BRR = ((T_clk / 4) / (BaudRate * 8)) - 1;

    ScicRegs.SCILBAUD.bit.BAUD = BRR & 0x00FF;

    ScicRegs.SCIHBAUD.bit.BAUD = (BRR >> 8) & 0x00FF;

    //Reset SCIA
    ScicRegs.SCICTL1.bit.SWRESET = 1;
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;

    ScicRegs.SCICTL1.all = 0x0023;  // Relinquish SCI from Reset
}
// FIN void Config_SCIC
//
// FIN DEL ARCHIVO
//
