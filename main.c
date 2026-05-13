// Inclusão das bibliotecas
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "stdlib.h"

// Declaração dos defines
#define BUFFER_SIZE 80      // Tamanho do buffer de armazenamento das entradas e saídas
#define FILTER_BUFFER_SIZE 16 // Tamanho do buffer do filtro média móvel
#define PI 3.14     // Valor adotado para pi
#define ADC_MAX_VALUE           4095U   // Valor máximo para ADC de 12 bits
#define ADC_REFERENCE_VOLTAGE   3.3F    // Tensão de referência do ADC
#define ADC_MEAN_VOLTAGE        1.65F   // Valor médio da tensão para ser alternada
#define PWM_COMPARE_MASK    0x03FFU // Máscara para bits 0-9 (valor de comparação)
#define PWM_ENABLE_BIT      (1U << 10) // Bit 10: habilita PWM
#define PWM_PERIOD_US       1000U   // Período total do PWM em microssegundos

// --- Enumeração para o Estado do Canal ADC ---
typedef enum {
    ADC_CHANNEL_STATE_DISABLED,
    ADC_CHANNEL_STATE_NORMAL,
    ADC_CHANNEL_STATE_SECURITY_ALERT
} AdcChannelState_t;

// Struct para o Canal ADC
typedef struct {
    float inputSignal;                            // Sinal de entrada 
    float inputSignalBuffer[BUFFER_SIZE];         // Buffer do sinal de entrada
    float outputSignal;                           // Sinal de saída
    float outputSignalBuffer[BUFFER_SIZE];        // Buffer do sinal de saída
    unsigned int iS;                       // Contador do buffer do sinal
    float filterBuffer[FILTER_BUFFER_SIZE];       // Buffer do filtro 
    unsigned int iF;                       // Contador do buffer do filtro
    float outputSignalVoltage;                           // Sinal de saída em volts
    float outputSignalVoltageBuffer[BUFFER_SIZE];        // Buffer do sinal de saída em volts
    AdcChannelState_t state;                      // Estado atual do canal
    unsigned int securityLimit;          // Valor do limite de segurança do canal
} AdcChannel_t;

// Enumeração para a FSM
typedef enum {
    SIGNAL_STATE_IDLE,
    SIGNAL_STATE_POSITIVE,
    SIGNAL_STATE_NEGATIVE
} SignalState_t;

// Enumeração para determinação do estado do LED
typedef enum {
    LED_LIGADO,
    LED_DESLIGADO
} LedState_t;

// Struct do PWM
typedef struct {
    unsigned int g_pwmControlReg; // Registrador de controle PWM simulado
    float g_dutyCyclePercent;       // Ciclo de trabalho desejado (0.0 a 100.0)
    unsigned int g_timeOn_us;               // Tempo LIGADO (LED ON)
    unsigned int g_timeOff_us;              // Tempo DESLIGADO (LED OFF)
} PWMChannel_t;


// Delaração das variáveis globais
unsigned int g_sineMean = 2048;
unsigned int g_sinePeak = 1000;
int g_signalNoiseMax = 50;
int g_signalNoiseMin = -50;
AdcChannel_t g_adcChannel;
bool g_enableModulation = false;
SignalState_t g_signalState1;
PWMChannel_t g_pwmChannel1;


// Protótipo das funções
void initAdcChannel(void);
void processAdcChannel(AdcChannel_t *pChannel);
float movingAverage(AdcChannel_t *pCh);
float convertADCToVoltage(float adcValue);
void signal_state_handler(AdcChannel_t *pCh);

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
        pCh->filterBuffer[i] = 0U;
    }
    // Limpa o buffer de entrada e saida
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        pCh->inputSignalBuffer[i] = 0U;
        pCh->outputSignalBuffer[i] = 0U;
        pCh->outputSignalVoltageBuffer[i] = 0U;
    }
    pCh->inputSignal = 0.0f;
    pCh->outputSignal = 0.0f;
    pCh->iS = 0U;
    pCh->iF = 0U;
    pCh->outputSignalVoltage = 0.0F;
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
    pChannel->inputSignal = g_sineMean + g_sinePeak * sin(2*PI*pChannel->iS/BUFFER_SIZE) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    pChannel->inputSignalBuffer[pChannel->iS] = pChannel->inputSignal;
    pChannel->outputSignal = movingAverage(pChannel);
    pChannel->outputSignalBuffer[pChannel->iS] = pChannel->outputSignal;
    pChannel->outputSignalVoltage = convertADCToVoltage(pChannel->outputSignal);
    pChannel->outputSignalVoltageBuffer[pChannel->iS] = pChannel->outputSignalVoltage;
    pChannel->iS = (pChannel->iS+1)%BUFFER_SIZE;
    if(g_enableModulation)
    {
        signal_state_handler(pChannel);
    }
    else {
        g_signalState1 = SIGNAL_STATE_IDLE;
    }
    
}

// Função de calculo da média movel
float movingAverage(AdcChannel_t *pCh)
{
    pCh->filterBuffer[pCh->iF] = pCh->inputSignal;
    float sum = 0.0f;
    unsigned int i;
    for (i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        sum += pCh->filterBuffer[i]; 
    }
    pCh->iF = (pCh->iF+1)%FILTER_BUFFER_SIZE;
    return sum / (float)FILTER_BUFFER_SIZE;
}

// Converte um valor ADC (0 a 4095) para tensão (0 a 3.3 V).
float convertADCToVoltage(float adcValue)
{
    return ((float)adcValue / (float)ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE - ADC_MEAN_VOLTAGE;
}

// Implementação da função do FSM
void signal_state_handler(AdcChannel_t *pCh)
{
    if(pCh->outputSignalVoltage > 0)
    {
        g_signalState1 = SIGNAL_STATE_POSITIVE;
    }
    else if (pCh->outputSignalVoltage <= 0) 
    {
        g_signalState1 = SIGNAL_STATE_NEGATIVE;
    }
}

//
// End of File
//
