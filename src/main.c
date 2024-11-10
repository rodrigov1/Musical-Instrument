#define NOTE_A 262	   // Do
#define NOTE_B 294	   // Re
#define NOTE_C 330	   // Mi
#define NOTE_D 349	   // Fa
#define NOTE_E 392	   // Sol
#define NOTE_F 440	   // La
#define NOTE_G 494	   // Si
#define NUM_SAMPLES 60 // Total number of samples for the full sine wave
#define DMA_SIZE 60

uint32_t PCLK = SystemCoreClock / 4; // CCLK/4
uint32_t fADC = 1000000;			 // 1MHz
uint32_t sin0_90_16_samples[16] = {0, 1045, 2079, 3090, 4067, 5000, 5877, 6691, 7431, 8090, 8660, 9135, 9510, 9781, 9945, 10000};
uint32_t dac_waveform[NUM_SAMPLES]; // Buffer to store DAC waveform values

// Prototipos de funciones
/**
 * @brief Create a full sine wave table for the DAC using 60 samples.
 * The sine wave values are computed based on the 0-90 degree values from the sin0_90_16_samples table.
 * @param table Pointer to the DAC waveform table
 * @param num_samples Number of samples in the waveform
 */
void create_waveform_table(uint32_t *table, uint32_t num_samples);

/**
 * @brief Configure the pin for DAC output (P0.26).
 */
void cfgPIN(void);

/**
 * @brief Initialize and configure the DAC.
 */
void initDAC(void);

/**
 * @brief Configure the DMA to transfer the waveform to the DAC.
 * @param table Pointer to the waveform table
 */
void cfgDMA(uint32_t *table);

// DMA configuration structure
GPDMA_Channel_CFG_Type GPDMACfg;
GPDMA_LLI_Type DMA_LLI_Struct; // DMA linked list item for continuous transfer

void cfgPIN() {
	// Configure pin P0.26 as DAC output
	PinCfg.Funcnum = PINSEL_FUNC_2;			  // DAC function
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL; // Disable open drain
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;	  // No pull-up or pull-down
	PinCfg.Pinnum = PINSEL_PIN_26;			  // Pin number 26
	PinCfg.Portnum = PINSEL_PORT_0;			  // Port number 0
	PINSEL_ConfigPin(&PinCfg);
}

void initDAC(uint32_t f_signal) {
	DAC_CONVERTER_CFG_TYPE dac_cfg;
	dac_cfg.DBLBUF_ENA = 0;
	dac_cfg.CNT_ENA = 1;
	dac_cfg.DMA_ENA = 1;
	DAC_Init(LPC_DAC);

	// Calculate sample update interval for the desired waveform frequency
	uint32_t time_out = (PCLK * fDAC) / (SINE_TABLE_SIZE * f_signal);
	DAC_SetDMATimeOut(LPC_DAC, time_out); // Set the DAC timeout between samples
	// Apply the DAC configuration
	DAC_ConfigDAConverterControl(LPC_DAC, &dac_cfg);
}

void create_waveform_table(uint32_t *table, uint32_t num_samples) {
	uint32_t i;

	// Use precomputed sine values for 0° to 90°, scale to DAC range (0 - 1023)
	// We mirror the values for 90°-180° and 270°-360°, and invert them for 180°-270°.
	for (i = 0; i < num_samples; i++) {
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

void DMA_Init(uint32_t *table) {
	// Set up the DMA linked list for continuous waveform transfer
	DMA_LLI_Struct.SrcAddr = (uint32_t)table;			 // Source: DAC waveform table
	DMA_LLI_Struct.DstAddr = (uint32_t)&(LPC_DAC->DACR); // Destination: DAC register
	DMA_LLI_Struct.NextLLI = (uint32_t)&DMA_LLI_Struct;	 // Point to itself for continuous transfer
	DMA_LLI_Struct.Control = DMA_SIZE | (2 << 18)		 // Source width: 32-bit
							 | (2 << 21)				 // Destination width: 32-bit
							 | (1 << 26);				 // Increment source address

	// Initialize the DMA module
	GPDMA_Init();

	// Configure DMA channel for memory-to-peripheral transfer
	GPDMACfg.ChannelNum = 0;						// Use channel 0
	GPDMACfg.SrcMemAddr = (uint32_t)table;			// Source: DAC waveform table
	GPDMACfg.DstMemAddr = 0;						// No memory destination (peripheral)
	GPDMACfg.TransferSize = DMA_SIZE;				// Transfer size: 60 samples
	GPDMACfg.TransferWidth = 0;						// Not used
	GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_M2P; // Memory-to-Peripheral transfer
	GPDMACfg.SrcConn = 0;							// Source is memory
	GPDMACfg.DstConn = GPDMA_CONN_DAC;				// Destination: DAC connection
	GPDMACfg.DMALLI = (uint32_t)&DMA_LLI_Struct;	// Linked list for continuous transfer

	// Apply DMA configuration
	GPDMA_Setup(&GPDMACfg);
}

void playNote(uint16_t frequency) {
	initDAC(frequency);								  // Configura el DAC acorde a la frecuencia de la nota
	create_waveform_table(dac_waveform, NUM_SAMPLES); // Generate the full sine wave
	GPDMA_ChannelCmd(DMA_CHANNEL_ZERO, ENABLE);		  // Enable DMA channel 0 to start the waveform generation
}

// Se podria configurar un boton (EXTI) que al apretarlo detenga la reproduccion de la nota

void UART_Init(void) {
	// Configura los pines P0.2 y P0.3 para UART
	LPC_PINCON->PINSEL0 |= (1 << 4) | (1 << 6);

	// Configura el baud rate
	uint32_t baudrate = 9600;
	uint32_t Fdiv = (SystemCoreClock / 16) / baudrate;
	LPC_UART0->LCR = 0x83; // Habilita el divisor y modo 8 bits, sin paridad, 1 bit de parada
	LPC_UART0->DLM = Fdiv / 256;
	LPC_UART0->DLL = Fdiv % 256;
	LPC_UART0->LCR = 0x03; // Desactiva DLAB y mantiene 8N1
	LPC_UART0->FCR = 0x07; // Habilita y resetea FIFOs
}

char UART_ReceiveChar(void) {
	while (!(LPC_UART0->LSR & 0x01))
		;				   // Espera hasta que el dato esté disponible
	return LPC_UART0->RBR; // Lee el dato recibido
}

int main() {
	uint32_t dac_waveform[NUM_SAMPLES]; // Buffer to store DAC waveform values

	DMA_Init(dac_waveform);

	cfgPIN();
	UART_Init();
	DMA_Init();

	while (1) {
		char command = UART_ReceiveChar(); // Recibe el comando de nota
		switch (command) {
		case 'A':
			playNote(NOTE_A);
			break;
			// Otros casos de notas
		}

		// uint16_t volume = readVolume();
		// setDACVolume(volume); // Ajusta el volumen en función del valor del ADC
	}
}

// ------------------------------------------------------------------
// -------- Control del volumen con un potenciometro con ADC --------
// ------------------------------------------------------------------
/* Esto si es que uso un potenciometro para cambiar el volumen
void ADC_Init(void) {
	LPC_PINCON->PINSEL0 |= (1 << 5); // P0.2 como AD0.7 (channel 7 del ADC)
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC, 7, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);

uint16_t readVolume() {
	ADC_ChannelCmd(LPC_ADC, 7) DC_StartCmd(LPC_ADC, ADC_START_NOW) // Inicio la conver
}
*/
