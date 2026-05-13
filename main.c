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
    float outputAlternatingSignal;                           // Sinal de saída em volts
    float outputAlternatingSignalBuffer[BUFFER_SIZE];        // Buffer do sinal de saída em volts
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
    unsigned int pwmControlReg; // Registrador de controle PWM simulado
    float dutyCyclePercent;       // Ciclo de trabalho desejado (0.0 a 100.0)
    unsigned int timeOn_us;               // Tempo LIGADO (LED ON)
    unsigned int timeOff_us;              // Tempo DESLIGADO (LED OFF)
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
float g_dutyCycleAlternatingSignal;

// Protótipo das funções
void initAdcChannel(void);
void processAdcChannel(AdcChannel_t *pChannel);
float movingAverage(AdcChannel_t *pCh);
float convertADCToAlternating(float adcValue);
void signal_state_handler(AdcChannel_t *pCh);
void enablePWM(void);
void disablePWM(void);
void calculatePWMOnOffTimes(unsigned int compareValue);
unsigned int calculateCompareValueFromDutyCycle(float dutyCycle);
void setPWMDutyCycleAndRegister(float dutyCycle);
void generateSoftwarePWM(void);
void initPWM(void);

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

    // Inicialização do ADC
    initAdcChannel();
    // Inicialização do PWM
    initPWM();

    while(1)
    {
        // Geração do PWM por Software
        generateSoftwarePWM();
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
        pCh->outputAlternatingSignalBuffer[i] = 0U;
    }
    pCh->inputSignal = 0.0f;
    pCh->outputSignal = 0.0f;
    pCh->iS = 0U;
    pCh->iF = 0U;
    pCh->outputAlternatingSignal = 0.0F;
    pCh->state = ADC_CHANNEL_STATE_NORMAL;
    pCh->securityLimit = 4095U;
}

// Função de interrupção do Timer de geração da senoide
__interrupt void INT_sineTimer_ISR(void)
{
    processAdcChannel(&g_adcChannel);
}

// Função de processamento do ADC a cada interrupção o timer
void processAdcChannel(AdcChannel_t *pChannel)
{
    pChannel->inputSignal = g_sineMean + g_sinePeak * sin(2*PI*pChannel->iS/BUFFER_SIZE) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    pChannel->inputSignalBuffer[pChannel->iS] = pChannel->inputSignal;
    pChannel->outputSignal = movingAverage(pChannel);
    pChannel->outputSignalBuffer[pChannel->iS] = pChannel->outputSignal;
    pChannel->outputAlternatingSignal = convertADCToAlternating(pChannel->outputSignal);
    pChannel->outputAlternatingSignalBuffer[pChannel->iS] = pChannel->outputAlternatingSignal;
    pChannel->iS = (pChannel->iS+1)%BUFFER_SIZE;
    g_dutyCycleAlternatingSignal = abs(pChannel->outputAlternatingSignal)/(float)g_sinePeak*100.0;
    setPWMDutyCycleAndRegister(g_dutyCycleAlternatingSignal);
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
float convertADCToAlternating(float adcValue)
{
    return ((float)adcValue  - g_sineMean);
}

// Implementação da função do FSM
void signal_state_handler(AdcChannel_t *pCh)
{
    if(pCh->outputAlternatingSignal > 0)
    {
        g_signalState1 = SIGNAL_STATE_POSITIVE;
    }
    else if (pCh->outputAlternatingSignal <= 0) 
    {
        g_signalState1 = SIGNAL_STATE_NEGATIVE;
    }
}

// Função de habilitação do PWM
void enablePWM(void)
{
    g_pwmChannel1.pwmControlReg = g_pwmChannel1.pwmControlReg | PWM_ENABLE_BIT;
}

// Função de desabilitação do PWM
void disablePWM(void)
{
    g_pwmChannel1.pwmControlReg = g_pwmChannel1.pwmControlReg & (~PWM_ENABLE_BIT);
}

// Calcula tempos ON/OFF a partir do valor de comparação do registrador.
void calculatePWMOnOffTimes(unsigned int compareValue)
{
    g_pwmChannel1.timeOn_us = compareValue;
    g_pwmChannel1.timeOff_us = PWM_PERIOD_US - g_pwmChannel1.timeOn_us;
}

// Converte ciclo de trabalho (%) para valor de comparação (0 a PWM_PERIOD_US).
unsigned int calculateCompareValueFromDutyCycle(float dutyCycle)
{
    if (dutyCycle < 0.0F) dutyCycle = 0.0F;
    else if (dutyCycle > 100.0F) dutyCycle = 100.0F;
    return (unsigned int)((dutyCycle / 100.0F) * PWM_PERIOD_US);
}

// Configura ciclo de trabalho e atualiza registrador simulado e tempos ON/OFF.
void setPWMDutyCycleAndRegister(float dutyCycle)
{
    g_pwmChannel1.dutyCyclePercent = dutyCycle;

    unsigned int compareVal = calculateCompareValueFromDutyCycle(dutyCycle);

    // Preserva o bit de enable, limpa os bits de comparação e escreve o novo valor
    unsigned int currentConfigBits = g_pwmChannel1.pwmControlReg & ~PWM_COMPARE_MASK;
    g_pwmChannel1.pwmControlReg = currentConfigBits | (compareVal & PWM_COMPARE_MASK);

    calculatePWMOnOffTimes(compareVal);
}

// Gera um ciclo da onda PWM por software no pino do LED.
// Apenas lógica normal (ativo baixo: 0 = LED ON, 1 = LED OFF).
void generateSoftwarePWM(void)
{
    if (g_signalState1 == SIGNAL_STATE_POSITIVE) 
    {
        if ((g_pwmChannel1.pwmControlReg & PWM_ENABLE_BIT) != 0U) // Se PWM habilitado
    {
        GPIO_writePin(LED_vermelho_GPIO, LED_DESLIGADO);
        // Período ON: pino LOW -> LED aceso
        GPIO_writePin(LED_azul_GPIO, LED_LIGADO);
        DEVICE_DELAY_US(g_pwmChannel1.timeOn_us);

        // Período OFF: pino HIGH -> LED apagado
        GPIO_writePin(LED_azul_GPIO, LED_DESLIGADO);
        DEVICE_DELAY_US(g_pwmChannel1.timeOff_us);
    }
    else // PWM desabilitado
    {
        GPIO_writePin(LED_vermelho_GPIO, LED_DESLIGADO);
        GPIO_writePin(LED_azul_GPIO, LED_DESLIGADO); // LED OFF
        DEVICE_DELAY_US(PWM_PERIOD_US); // Aguarda período completo
    }
    }
    else if (g_signalState1 == SIGNAL_STATE_NEGATIVE) 
    {
        if ((g_pwmChannel1.pwmControlReg & PWM_ENABLE_BIT) != 0U) // Se PWM habilitado
    {
        GPIO_writePin(LED_azul_GPIO, LED_DESLIGADO);
        // Período ON: pino LOW -> LED aceso
        GPIO_writePin(LED_vermelho_GPIO, LED_LIGADO);
        DEVICE_DELAY_US(g_pwmChannel1.timeOn_us);

        // Período OFF: pino HIGH -> LED apagado
        GPIO_writePin(LED_vermelho_GPIO, LED_DESLIGADO);
        DEVICE_DELAY_US(g_pwmChannel1.timeOff_us);
    }
    else // PWM desabilitado
    {
        GPIO_writePin(LED_azul_GPIO, LED_DESLIGADO);
        GPIO_writePin(LED_vermelho_GPIO, LED_DESLIGADO); // LED OFF
        DEVICE_DELAY_US(PWM_PERIOD_US); // Aguarda período completo
    }
    }
    else {
        GPIO_writePin(LED_azul_GPIO, LED_DESLIGADO);
        GPIO_writePin(LED_vermelho_GPIO, LED_DESLIGADO); // LED OFF
        DEVICE_DELAY_US(PWM_PERIOD_US); // Aguarda período completo
    }
    
}

// Função de inicialização do PWM
void initPWM(void)
{
    g_pwmChannel1.pwmControlReg = 0x0000U;
    g_pwmChannel1.dutyCyclePercent = 0.0F;
    enablePWM();
}

//
// End of File
//
