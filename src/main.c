#define NOTE_A 262 // Do
#define NOTE_B 294 // Re
#define NOTE_C 330 // Mi
#define NOTE_D 349 // Fa
#define NOTE_E 392 // Sol
#define NOTE_F 440 // La
#define NOTE_G 494 // Si
#define SINE_TABLE_SIZE 256

uint32_t PCLK = SystemCoreClock/4; // CCLK/4
uint32_t fADC = 1000000; // 1MHz
uint16_t sineTable[SINE_TABLE_SIZE];
uint32_t f_signal = NOTA_A;
#
// Inicializa la tabla con valores senoidales
void initSineTable() {
	for (int i = 0; i < SINE_TABLE_SIZE; i++) {
		sineTable[i] = (uint16_t)((1 + sin(2 * M_PI * i / SINE_TABLE_SIZE)) * 512); // Escala a valores de DAC
	}
}

void playNote(uint16_t frequency) {
	uint32_t interval = (SystemCoreClock / (SINE_TABLE_SIZE * frequency)); // Calcula el intervalo entre muestras
	LPC_TIM0->MR0 = interval;											   // Ajusta el temporizador para DMA
	LPC_TIM0->TCR = 1;													   // Inicia el temporizador para activar la frecuencia
}

void DMAtoDAC() {
	DAC_CONVERTER_CFG_TYPE dac_cfg;
	dac_cfg.DBLBUF_ENA = 0;
	dac_cfg.CNT_ENA = 1;
	dac_cfg.DMA_ENA = 1;
	DAC_Init(LPC_DAC);
  DAC_ConfigDAConverterControl(LPC_ADC,&dac_cfg);
  uint32_t time_out = (PCLK * fDAC)/(SINE_TABLE_SIZE* f_signal);
}
// ------------------------------------------------------------------
// -------- Control del volumen con un potenciometro con ADC --------
// ------------------------------------------------------------------
}
void ADC_Init(void) {
	LPC_PINCON->PINSEL0 |= (1 << 5); // P0.2 como AD0.7 (channel 7 del ADC)
	ADC_Init(LPC_ADC, 200000);
	ADC_IntConfig(LPC_ADC, 7, ENABLE);
	NVIC_EnableIRQ(ADC_IRQn);
}

uint16_t readVolume() {
	ADC_ChannelCmd(LPC_ADC, 7)
	DC_StartCmd(LPC_ADC, ADC_START_NOW)	   // Inicio la conver
	

