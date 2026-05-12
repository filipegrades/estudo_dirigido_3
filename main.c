// Inclusão das bibliotecas
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "stdlib.h"

// Declaração dos defines
#define BUFFER_SIZE 80      // Tamanho do buffer de armazenamento das entradas e saídaa
#define FILTER_BUFFER_SIZE 16 // Tamanho do buffer do filtro média móvel
#define PI 3.14     // Valor adotado para pi
#define ADC_MAX_VALUE           4095U   // Valor máximo para ADC de 12 bits
#define ADC_REFERENCE_VOLTAGE   3.3F    // Tensão de referência do ADC
#define ADC_MEAN_VOLTAGE        1.65F   // Valor médio da tensão para ser alternada

// --- Enumeração para o Estado do Canal ADC ---
typedef enum {
    ADC_CHANNEL_STATE_DISABLED,
    ADC_CHANNEL_STATE_NORMAL,
    ADC_CHANNEL_STATE_SECURITY_ALERT
} AdcChannelState_t;

// Struct para o Canal ADC
typedef struct {
    float g_inputSignal;                            // Sinal de entrada 
    float g_inputSignalBuffer[BUFFER_SIZE];         // Buffer do sinal de entrada
    float g_outputSignal;                           // Sinal de saída
    float g_outputSignalBuffer[BUFFER_SIZE];        // Buffer do sinal de saída
    unsigned int g_iS;                       // Contador do buffer do sinal
    float g_filterBuffer[FILTER_BUFFER_SIZE];       // Buffer do filtro 
    unsigned int g_iF;                       // Contador do buffer do filtro
    float g_outputSignalVoltage;                           // Sinal de saída em volts
    float g_outputSignalVoltageBuffer[BUFFER_SIZE];        // Buffer do sinal de saída em volts
    AdcChannelState_t state;                      // Estado atual do canal
    unsigned int securityLimit;          // Valor do limite de segurança do canal
} AdcChannel_t;



// Delaração das variáveis globais
unsigned int g_sineMean = 2048;
unsigned int g_sinePeak = 1000;
int g_signalNoiseMax = 50;
int g_signalNoiseMin = -50;
AdcChannel_t g_adcChannel;



// Protótipo das funções
void initAdcChannel(void);
void processAdcChannel(AdcChannel_t *pChannel);
float movingAverage(AdcChannel_t *pCh);
float convertADCToVoltage(float adcValue);

// Main
void main(void)
{
    // Device Initialization
    Device_init();

    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

	Board_init();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    initAdcChannel();

    while(1)
    {
    }	
	
}

// Função de inicialização do canal
void initAdcChannel(void)
{
    AdcChannel_t *pCh = &g_adcChannel;
    unsigned int i;
    // Limpa o buffer do filtro
    for (i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        pCh->g_filterBuffer[i] = 0U;
    }
    // Limpa o buffer de entrada e saida
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        pCh->g_inputSignalBuffer[i] = 0U;
        pCh->g_outputSignalBuffer[i] = 0U;
        pCh->g_outputSignalVoltageBuffer[i] = 0U;
    }
    pCh->g_inputSignal = 0.0f;
    pCh->g_outputSignal = 0.0f;
    pCh->g_iS = 0U;
    pCh->g_iF = 0U;
    pCh->g_outputSignalVoltage = 0.0F;
    pCh->state = ADC_CHANNEL_STATE_NORMAL;
    pCh->securityLimit = 4095U;
}

// Função de interrupção do Timer de geração da senoide
__interrupt void INT_sineTimer_ISR(void)
{
    processAdcChannel(&g_adcChannel);
}

void processAdcChannel(AdcChannel_t *pChannel)
{
    pChannel->g_inputSignal = g_sineMean + g_sinePeak * sin(2*PI*pChannel->g_iS/BUFFER_SIZE) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    pChannel->g_inputSignalBuffer[pChannel->g_iS] = pChannel->g_inputSignal;
    pChannel->g_outputSignal = movingAverage(&g_adcChannel);
    pChannel->g_outputSignalBuffer[pChannel->g_iS] = pChannel->g_outputSignal;
    pChannel->g_outputSignalVoltage = convertADCToVoltage(pChannel->g_outputSignal);
    pChannel->g_outputSignalVoltageBuffer[pChannel->g_iS] = pChannel->g_outputSignalVoltage;
    pChannel->g_iS = (pChannel->g_iS+1)%BUFFER_SIZE;
}

// Função de calculo da média movel
float movingAverage(AdcChannel_t *pCh)
{
    pCh->g_filterBuffer[pCh->g_iF] = pCh->g_inputSignal;
    float sum = 0.0f;
    unsigned int i;
    for (i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        sum += pCh->g_filterBuffer[i]; 
    }
    pCh->g_iF = (pCh->g_iF+1)%FILTER_BUFFER_SIZE;
    return sum / (float)FILTER_BUFFER_SIZE;
}

// Converte um valor ADC (0 a 4095) para tensão (0 a 3.3 V).
float convertADCToVoltage(float adcValue)
{
    return ((float)adcValue / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE - ADC_MEAN_VOLTAGE;
}


//
// End of File
//
