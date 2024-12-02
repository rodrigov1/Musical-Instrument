#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>

// Drivers
#include <string.h>
#include <stdio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_gpdma.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_timer.h>

// Definicion de Macros
#define DO 262
#define RE 294
#define MI 330
#define FA 349
#define SOL 392
#define LA 440
#define SI 494
#define DO2 523
#define RE2 587
#define MI2 659
#define FA2 699
#define SOL2 784
#define LA2 880
#define SI2 988
#define DO3 1047
#define RE3 1175
#define EDGE_RISING 0
#define De_mS_A_Cuentas_For(num) ((num*SystemCoreClock)/(1000*11))
#define ROWS 4
#define COLUMNS 4
#define NUM_SAMPLES 2  // Total number of samples for the full sine wave
#define DMA_SIZE 2
#define BIT(x) (1 << x)
#define SIZE 16
#define TIMER 600000
#define PORT(x) x
#define INPUT 0
#define OUTPUT 1
#define SALIDA_TECLADO (0XF << 4)
#define ENTRADA_TECLADO 0XF
#define PCLK 25
#define UART_BUFFER_SIZE 20

// Variables Globales
uint32_t fDAC = 1000000;			 // 1MHz
uint8_t i = 0;
uint8_t flag = 0;
//uint32_t octava[16] = {  DO,    RE,    MI,   FA,   SOL,    LA,    SI,    DO2,   RE2,   MI2,   FA2,   SOL2,  LA2,   SI2,   DO3,   RE3 };
uint32_t matches[16] = { 47710, 42517, 37878, 35816, 31887, 28409, 25303, 23900, 21294, 18968, 17882, 15943, 14204, 12651, 11938, 10638};
uint16_t dac_waveform[NUM_SAMPLES] = {0 ,(1023 << 6)};  // Buffer to store DAC waveform values
uint8_t divisor = 1;

// Buffer de salida para el mensaje
char uartBuffer[UART_BUFFER_SIZE];

// PROTOTIPOS DE FUNCIONES
void PIN_Config(void);
void ADC_Config(void);
void DAC_Config(void);
void DMA_Config(void);
void TIM0_Config(void);
void TIM1_Config(void);
void UART_Config(void);
uint8_t obtener_teclaMatricial(void);
void mapearNota(uint8_t indice);
void delay(uint32_t tiempo_mS);

// FUNCION PRINCIPAL
int main() {
	//SystemInit();
	PIN_Config();
	ADC_Config();
	DAC_Config();
	TIM0_Config();
	TIM1_Config();
	UART_Config();

	// Infinite loop, system operates using DMA
	while (TRUE)
	{
		//__WFI(); // Wait for interrupt, keeping CPU in low-power mode
	}

	return 0;
}

void PIN_Config(void) {
	PINSEL_CFG_Type PinCfg;
	// DAC AOUT
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 26;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Funcnum = PINSEL_FUNC_2;
	PINSEL_ConfigPin(&PinCfg);

    uint8_t i;
	PinCfg.Funcnum = PINSEL_FUNC_0;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLDOWN;
	PinCfg.Portnum = PINSEL_PORT_2;

	/* Entradas */
	for (i = 0; i< 4; i++) {
		PinCfg.Pinnum = i;
		PINSEL_ConfigPin(&PinCfg); //configura los pines p2.0 a p2.3 como gpio, con pull-down y modo normal
	}

	GPIO_SetDir(PINSEL_PORT_2, ENTRADA_TECLADO, INPUT); // configura los pines p2.0 a p2.3 como entradas

	/* Salidas */
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;

	for (i = 4; i< 8; i++) {
		PinCfg.Pinnum = i;
		PINSEL_ConfigPin(&PinCfg); //configura los pines p2.4 a p2.7 como gpio, con pull-up y modo normal
	}

	GPIO_SetDir(PINSEL_PORT_2, SALIDA_TECLADO, OUTPUT); // configura los pines p2.4 a p2.7 como salids
	FIO_ByteSetValue(2, 0, SALIDA_TECLADO);
	GPIO_IntCmd(PINSEL_PORT_2, ENTRADA_TECLADO, EDGE_RISING);
	GPIO_ClearInt(PINSEL_PORT_2, ENTRADA_TECLADO);
	NVIC_SetPriority(EINT3_IRQn, 2);
	NVIC_EnableIRQ(EINT3_IRQn);

	LPC_GPIO2->FIODIR |= (1<<13);
}

void ADC_Config(void){
	// AD0.0
	PINSEL_CFG_Type PinCfg;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PINSEL_ConfigPin(&PinCfg);

	ADC_Init(LPC_ADC, 200000);                  /* Initialize the ADC at 200 kHz */
	ADC_ChannelCmd(LPC_ADC, 0, ENABLE); /* Enable ADC channel 0 */
	ADC_BurstCmd(LPC_ADC, DISABLE);
	ADC_IntConfig(LPC_ADC, ADC_ADINTEN0, SET);
	NVIC_SetPriority(ADC_IRQn, 5);
	NVIC_EnableIRQ(ADC_IRQn);
}

void DAC_Config(void) {
	DAC_Init(LPC_DAC);
}

void DMA_Config(void) {
	GPDMA_Init(); // Inicializa el controlador de DMA

	// Configura el canal de DMA para transferir datos desde el buffer a UART
	GPDMA_Channel_CFG_Type uartcfg;
	uartcfg.ChannelNum = 0; // Canal 0 de DMA
	uartcfg.SrcMemAddr = (uint32_t)uartBuffer; // Dirección de inicio del mensaje
	uartcfg.DstMemAddr = 0; // Registro de transmisión de UART2
	uartcfg.TransferSize = sizeof(uartBuffer); // Tamaño del mensaje
	uartcfg.TransferWidth = 0; // Transfiere en bytes
	uartcfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Transferencia de Memoria a Periférico
	uartcfg.SrcConn = 0; // No se requiere conexión de fuente
	uartcfg.DstConn = GPDMA_CONN_UART2_Tx; // Conexión de destino UART2 Tx
	uartcfg.DMALLI = 0; // Sin enlace a otra transferencia

	// Configura
	GPDMA_Setup(&uartcfg);
	GPDMA_ChannelCmd(0, ENABLE);
}

void TIM0_Config(void) {
	LPC_TIM0->PR = 0; // Prescaler a 0 (PCLK = 25 MHz)

	// Configurar el valor de comparación (Match Register)
	LPC_TIM0->MR0 = matches[0]; // Match value para 1.908 ms (25 MHz)

	// Configurar Match Control Register (MCR)
	LPC_TIM0->MCR = (1 << 0) | (1 << 1); // Interrupción y reset al alcanzar MR0

	LPC_TIM0->IR |= 0x3F;

	// Configurar Timer0 para iniciar
	LPC_TIM0->TCR = 3; // Iniciar el Timer0
	LPC_TIM0->TCR &= ~2;
	NVIC_SetPriority(TIMER0_IRQn, 3);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

void TIM1_Config(void) {
	LPC_TIM1->PR = 24999; // Prescaler (PCLK = 25 MHz), T_res = 1mS

	// Configurar el valor de comparación (Match Register)
	LPC_TIM1->MR0 = 499; // Match value para 500ms (2Hz)

	// Configurar Match Control Register (MCR)
	LPC_TIM1->MCR = (1 << 0) | (1 << 1); // Interrupción y reset al alcanzar MR0

	LPC_TIM1->IR |= 0x3F;

	// Configurar Timer0 para iniciar
	LPC_TIM1->TCR = 3; // Iniciar el Timer1
	LPC_TIM1->TCR &= ~2;
	NVIC_SetPriority(TIMER1_IRQn, 4);
	NVIC_EnableIRQ(TIMER1_IRQn);
}

void UART_Config(void) {
	//Configuracion para pin - UART TXD2
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;

	PINSEL_ConfigPin(&PinCfg);

	UART_CFG_Type UARTConfigStruct;
	UART_ConfigStructInit(&UARTConfigStruct);
	UART_Init(LPC_UART2, &UARTConfigStruct);

	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE; // Habilita el modo DMA
	UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV0;
	UARTFIFOConfigStruct.FIFO_ResetRxBuf = ENABLE;
	UARTFIFOConfigStruct.FIFO_ResetTxBuf = ENABLE;
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	UART_FIFOConfig(LPC_UART2, &UARTFIFOConfigStruct);
	UART_TxCmd(LPC_UART2, ENABLE);  // Habilita transmisión
}

void TIMER0_IRQHandler(void) {
	i = (i+1) % 2;
	DAC_UpdateValue(LPC_DAC, dac_waveform[i] / divisor);
	LPC_TIM0->IR |= (1<<0);
}

void TIMER1_IRQHandler(void) {
	if (!ADC_ChannelGetStatus(LPC_ADC, 0, 1)){// Pregunto si ya termino la ultima conversion
		ADC_StartCmd(LPC_ADC, ADC_START_NOW); // Solo inicia si no esta ocupado
	}
	LPC_TIM1->IR |= (1<<0);
}

void ADC_IRQHandler(void) {
	uint16_t value = ADC_ChannelGetData(LPC_ADC, 0);
	static uint8_t flag = 0;

	if (flag % 2 == 0){
		LPC_GPIO2->FIOSET |= (1<<13);
	} else {
		LPC_GPIO2->FIOCLR |= (1<<13);
	}

	flag = (flag + 1) % 5;

    if (value < 4096 && value >= 2731){
    	divisor = 1;
    }
    if (value < 2731 && value >= 1365){
    	divisor = 2;
    }
    if (value < 1365 && value >= 0){
    	divisor = 3;
    }

    LPC_ADC->ADGDR &= ~(1 << 31); // Limpia la flag DONE

}

void EINT3_IRQHandler(void) {
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(EINT3_IRQn);
	uint8_t indice = obtener_teclaMatricial();
	delay(200);

	memset(uartBuffer, '\0', sizeof(uartBuffer));
	mapearNota(indice);
	DMA_Config();

	TIM_UpdateMatchValue(LPC_TIM0, 0, matches[indice]);
	TIM_ResetCounter(LPC_TIM0);


	LPC_GPIOINT->IO2IntClr |= 0xf;
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

uint8_t obtener_teclaMatricial(void) {
	uint8_t fila=0,	columna=0;

	/*
	 * Voy pin a pin verificando cual es la fila apretada
	 */
	while (fila != 4) {
		if (GPIO_ReadValue(2) & (1 << fila))
			break;
		fila++;
	}
	if (fila >= 4)
		return 4; //Si no encontro devuelve una tecla sin uso

	/*
	 * Para obtener la columna barro un 0 por las columnas, cuando la fila
	 * cambia de estado es porque encontre la columna apretada
	 */
	while (columna != 4) {
		LPC_GPIO2->FIOPIN0 = ~(1 << (4+columna));

		if (!((FIO_ByteReadValue(2,0)) & ENTRADA_TECLADO))
			break;

		columna++;
	}
	if (columna >= 4)
		return 4;

	FIO_ByteSetValue(2, 0,SALIDA_TECLADO);

	return (fila*4 + columna);
}

void mapearNota(uint8_t indice) {
    // Mensajes para cada caso
    const char *mensajes[] = {
        "DO",
        "RE",
        "MI",
        "FA",
        "SOL",
        "LA",
        "SI",
		"DO2",
		"RE2",
		"MI2",
		"FA2",
		"SOL2",
		"LA2",
		"SI2",
		"DO3",
		"RE3"
    };
    //uartBuffer[sizeof(uartBuffer)] = '\n';
    if (indice < 16) {
        // Copiar el mensaje al buffer global
        const char *mensaje = mensajes[indice];
        uint8_t i = 0;

        while (mensaje[i] != '\0') {
            uartBuffer[i] = mensaje[i];
            i++;
        }
        uartBuffer[i] = '\n'; // Terminar el string
    }
}

void delay(uint32_t tiempo_mS) {
	/*
	 * Cada ciclo for tarda 1.1*10 a la -7 segundos. Entonces hago la regla de 3
	 *
	 * 1.1/100M -> 	1
	 * tiempo 	->	x
	 *
	 * entonces:
	 *
	 * x=tiempo*100M / 1.1 que para evitar el punto flotante hago = tiempo*100M*10 / 11.
	 *
	 * Para simplificar calculos, el valor ingresado es directamente el valor en mS
	 */
	tiempo_mS = De_mS_A_Cuentas_For(tiempo_mS);
	for(uint32_t cont=0; cont<tiempo_mS;cont++);
}
