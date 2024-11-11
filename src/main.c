#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>

// Drivers
#include <lpc17xx_pinsel.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_gpdma.h>
#include <lpc17xx_dac.h>
#include <lpc17xx_uart.h>

// Definicion de Macros
#define DO 262	    // DO
#define RE 294	    // RE
#define MI 330	    // MI
#define FA 349	    // FA
#define SOL 392	    // SOL
#define LA 440	    // LA
#define SI 494	    // SI
#define NUM_SAMPLES 60  // Total number of samples for the full sine wave
#define DMA_SIZE 60

// Variables Globales
uint32_t PCLK = SystemCoreClock / 4; // CCLK/4
uint32_t fDAC = 1000000;			 // 1MHz
// Muestras de Seno desde 90° a 0°, cada 6 grados y multiplicadas por 10.000
uint16_t sin0_90_16_samples[16] = {0, 1045, 2079, 3090, 4067, 5000, 5877, 6691, 7431, 8090, 8660, 9135, 9510, 9781, 9945, 10000};
uint8_t octava[7] = {DO, RE, MI, FA, SOL, LA, SI};
uint16_t dac_waveform[NUM_SAMPLES];  // Buffer to store DAC waveform values

// Funciones de Configuracion
void PIN_Config(void) {
	PINSEL_CFG_Type PinCfg;
	// Configure pin P0.26 as DAC output
	PinCfg.Funcnum = PINSEL_FUNC_2;			  // DAC function
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL; // Disable open drain
	PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE; // No pull-up or pull-down
	PinCfg.Pinnum = PINSEL_PIN_26;			  // Pin number 26
	PinCfg.Portnum = PINSEL_PORT_0;			  // Port number 0
	PINSEL_ConfigPin(&PinCfg);

	// Configure pin P0.23 as ADC Channel 0
	PinCfg.Pinnum = PINSEL_PIN_23;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&PinCfg);

	// Configure pin P0.3 as RXD0
	PinCfg.Pinnum = PINSEL_PIN_3;
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PINSEL_ConfigPin(&PinCfg);
}

void DAC_Config() {
	DAC_CONVERTER_CFG_Type dac_cfg;
	dac_cfg.DBLBUF_ENA = 0;  // Disable Double Buffer
	dac_cfg.CNT_ENA = 1;     // Enable DAC Timeout Counter
	dac_cfg.DMA_ENA = 1;	 // Enable DMA Burst
	DAC_Init(LPC_DAC);

	// Apply the DAC configuration
	DAC_ConfigDAConverterControl(LPC_DAC, &dac_cfg);
}

void DAC_Timeout_Config(uint16_t f_signal) {
	uint32_t time_out = (PCLK * fDAC) / (NUM_SAMPLES * f_signal);
	DAC_SetDMATimeOut(LPC_DAC, time_out);
}

// Esto si es que uso un potenciometro para cambiar el volumen
void ADC_Config(void) {
	ADC_Init(LPC_ADC, 200000);  // 200 kHz
	ADC_ChannelCmd(LPC_ADC, 0);
}

void DMA_Config() {
	// Set up the DMA linked list for continuous waveform transfer
	GPDMA_LLI_Type linkedlist;
	linkedlist.SrcAddr = (uint32_t)dac_waveform;	 // Source: DAC waveform table
	linkedlist.DstAddr = (uint32_t)&(LPC_DAC->DACR); // Destination: DAC register
	linkedlist.NextLLI = (uint32_t)&linkedlist;	     // Point to itself for continuous transfer
	linkedlist.Control = DMA_SIZE
					   | (1 << 18)		 // Source width: 16-bit
					   | (2 << 21)		 // Destination width: 16-bit
					   | (1 << 26);	     // Increment source address

	// Initialize the DMA module
	GPDMA_Init();

	// Configure DMA channel for memory-to-peripheral transfer
	GPDMA_Channel_CFG_Type gpdma_channel;
	gpdma_channel.ChannelNum = 0;						 // Use channel 0
	gpdma_channel.SrcMemAddr = (uint32_t)table;			 // Source: DAC waveform table
	gpdma_channel.DstMemAddr = 0;						 // No memory destination (peripheral)
	gpdma_channel.TransferSize = DMA_SIZE;				 // Transfer size: 60 samples
	gpdma_channel.TransferWidth = 0;					 // Not used
	gpdma_channel.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memory-to-Peripheral transfer
	gpdma_channel.SrcConn = 0;							 // Source is memory
	gpdma_channel.DstConn = GPDMA_CONN_DAC;				 // Destination: DAC connection
	gpdma_channel.DMALLI = (uint32_t)&linkedlist;	     // Linked list for continuous transfer

	// Apply DMA configuration
	GPDMA_Setup(&gpdma_channel);
	GPDMA_ChannelCmd(0, ENABLE);
}

void UART_Config(void) {
	UART_CFG_Type uart_config;

	uart_config.Baud_rate = 9600;
	uart_config.Databits = UART_DATABIT_8;
	uart_config.Parity = UART_PARITY_NONE;
	uart_config.Stopbits = UART_STOPBIT_1;

	// es lo mismo que UART_ConfigStructInit(&uart_config);

	UART_Init(LPC_UART0,&uart_config);

	UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE);

	NVIC_EnableIRQ(UART0_IRQn);
}

// IRQ Handlers
void UART0_IRQHandler(void){
	uint8_t NOTA = UART_ReceiveByte(LPC_UART0);
	playNote(octava[NOTA]);

	// La flag se limpia al leer el valor
}

// Prototipos Funciones
uint16_t readVolume(void);
void create_waveform_table(uint32_t *table);
void playNote(uint16_t frecuency);

// FUNCION PRINCIPAL
int main() {
	PIN_Config();
	ADC_Config();
	DAC_Config();
	DMA_Config();
	UART_Config();

	create_waveform_table(dac_waveform);

	while (1) {
		//nope
	}
}

uint16_t readVolume() {
	ADC_StartCmd(LPC_ADC, ADC_START_NOW); // Inicio la conver

	return ADC_ChannelGetData(LPC_ADC, 0);
}

void create_waveform_table(uint32_t *table) {
	uint8_t i;

	// Use precomputed sine values for 0° to 90°, scale to DAC range (0 - 1023)
	// We mirror the values for 90°-180° and 270°-360°, and invert them for 180°-270°.
	for (i = 0; i < NUM_SAMPLES; i++) {
		if (i <= 15) // 0° to 90°
		{
			table[i] = 512 + (512 * sin0_90_16_samples[i]) / 10000; // Scale and shift for DAC
			if (i == 15)											// Ensure maximum value at 90°
				table[i] = 1023;
		} else if (i <= 30) // 90° to 180°
		{
			table[i] = 512 + (512 * sin0_90_16_samples[30 - i]) / 10000; // Mirrored values
		} else if (i <= 45)												 // 180° to 270°
		{
			table[i] = 512 - (512 * sin0_90_16_samples[i - 30]) / 10000; // Inverted values
		} else															 // 270° to 360°
		{
			table[i] = 512 - (512 * sin0_90_16_samples[60 - i]) / 10000; // Mirrored inverted values
		}
		table[i] <<= 6; // Shift left to align with DAC register format
	}
}

void playNote(uint16_t frequency) {
	DAC_Timeout_Config(frequency);	// Configura el DAC acorde a la frecuencia de la nota
}
