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
 \file           CONFIG_GPIO.c
 \brief          Modulo de configuracion de los GPIO del microcontrolador
 TMS320F2877s.

 \author         Jesus Nieto Hervas
 \version        V0.0
 \date           15/05/2018
 **************************************************************************/

//
// Included
//
#include "F28x_Project.h"
#include "Inc_Drivers.h"

/**
 ********************************************************************************
 *  CPU01 :
 *  | GPIO | I/O|   Nombre    |  Descripcion
 *  ----------------------------------------------------------------------------
 *  | 02   | I  | E_Tierra2   | Entrada errores del check tierra Chademo
 *  ----------------------------------------------------------------------------
 *  | 03   | I  | E_Tierra1   | Entrada errores del check tierra Chademo
 *  ----------------------------------------------------------------------------
 *  | 17   | I  | VerifCL     | Entrada verifiaciones del Conector Lock
 *  ----------------------------------------------------------------------------
 *  | 20   | I  | CANB        | RX del CAN B para protcolo Cahdemo
 *  ----------------------------------------------------------------------------
 *  | 21   | O  | CANB        | TX del CAN B para protcolo Cahdemo
 *  ----------------------------------------------------------------------------
 *  | 41   | O  | CL          | Salida Conector Lock protocolo Chademo
 *  ----------------------------------------------------------------------------
 *  | 58   | O  | D2          | Salida D2 protocolo Chademo
 *  ----------------------------------------------------------------------------
 *  | 59   | O  | D1          | Salida D1 protocolo Chademo
 *  ----------------------------------------------------------------------------
 *  | 61   | 0  | Ventilador  | Salida Rele forzar refirgeracion por aire
 *  ----------------------------------------------------------------------------
 *  | 62   | 0  | S_Diodo     | Salida Rele para salida de potencia por diodo
 *  ----------------------------------------------------------------------------
 *  | 63   | 0  | S_V2G       | Salida Rele para entrada de potencia directa
 *  ----------------------------------------------------------------------------
 *  | 64   | 0  | S_Precarga  | Salida Rele para precargar del Bus
 *  ----------------------------------------------------------------------------
 *  | 65   | 0  | S_Masa      | Salida Rele para conectar masa del Bus
 *  ----------------------------------------------------------------------------
 *  | 89   | 0  | S_Tierra2   | Salida Rele activar check tierra Chademo
 *  ----------------------------------------------------------------------------
 *  | 91   | 0  | S_Tierra1   | Salida Rele activar check tierra Chademo
 *  ----------------------------------------------------------------------------
 *  | 99   | I  | j           | Entrada J protocolo Chademo
 *  ----------------------------------------------------------------------------
 */
//
/**
 ***************************************************************************
 \fn         INIT_GPIO
 \brief      Funcion principal de configuracion de los GPIOSs. Esta funcion
 es llamada desde el "main".
 Los GPIOS estan divididos en 6 Grupos (A,B,C,D,E,F) donde esta
 repartidos e la siguiente manera :
 - GRUPO A = GPIO00  a GPIO31
 - GRUPO B = GPIO32  a GPIO63
 - GRUPO C = GPIO64  a GPIO95
 - GRUPO D = GPIO96  a GPIO127
 - GRUPO E = GPIO128 a GPIO159
 - GRUPO F = GPIO160 a GPIO168

 NOTA IMPORTANTE : Se han implementado los modulos necesarios
 y los m�s importantes de configuracion. para m�s resigtros
 consular el PDF de TI "spruhx5e.pdf".

 \param[in]  void
 \return     void

 **************************************************************************/
void Config_GPIO()
{
    //
    //  pin = Num GPIO
    //  cpu = Num CPU
    //  peripheral = Cod funcion especial (tms320f28377s.pdf -> Table 4-3)
    // GPIO_SetupPinMux(pin, cpu, peripheral);
    //
    //  pin    = Num GPIO
    //  output = GPIO_OUTPUT
    //           GPIO_INPUT
    //  peripheral = GPIO_PUSHPULL
    //               GPIO_PULLUP
    //               GPIO_ASYNC
    //GPIO_SetupPinOptions(pin, output, flags)

    InitGpio();

    /*-------------INIT CONFIGURATION FOR CPU2-------------------*/
    //GPIOs OUTPUTs
    GPIO_SetupPinMux(P9_8, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_7, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_6, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_5, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_5, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_4, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_3, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_3, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_2, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_1, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P9_1, GPIO_OUTPUT, GPIO_PUSHPULL);

    //GPIOs INPUTs
    GPIO_SetupPinMux(P4_1, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P4_1, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(P4_2, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(P4_2, GPIO_INPUT, GPIO_ASYNC);

    //CANB_RX
    GPIO_SetupPinMux(P6_2_RX, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(P6_2_RX, GPIO_INPUT, GPIO_ASYNC);

    //CANB_TX
    GPIO_SetupPinMux(P6_2_TX, GPIO_MUX_CPU2, 3);
    GPIO_SetupPinOptions(P6_2_TX, GPIO_OUTPUT, GPIO_PUSHPULL);
    /*-------------END CONFIGURATION FOR CPU2-------------------*/

    /*-------------INIT CONFIGURATION FOR CPU1-------------------*/
    //GPIOs OUTPUTs
    GPIO_SetupPinMux(P9_9, GPIO_MUX_CPU1,0);
    GPIO_SetupPinOptions(P9_9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(P9_10, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(P9_10, GPIO_OUTPUT, GPIO_PUSHPULL);

    //GPIOs INPUTs
    GPIO_SetupPinMux(P4_4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(P4_4, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(P4_5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(P4_5, GPIO_INPUT, GPIO_ASYNC);
    GPIO_SetupPinMux(P4_3, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(P4_3, GPIO_INPUT, GPIO_ASYNC);

    // CANA_RX
    GPIO_SetupPinMux(P6_1_RX, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(P6_1_RX, GPIO_INPUT, GPIO_ASYNC);

    // CANA_TX
    GPIO_SetupPinMux(P6_1_TX, GPIO_MUX_CPU1, 3);
    GPIO_SetupPinOptions(P6_1_TX, GPIO_OUTPUT, GPIO_PUSHPULL);
    /*-------------END CONFIGURATION FOR CPU1-------------------*/
}

//
// FIN DEL ARCHIVO
//

