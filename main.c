// Inclusão das bibliotecas
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "stdlib.h"

// Declaração dos defines
#define BUFFER_SIZE 160      // Tamanho do buffer de armazenamento das entradas e saídaa
#define FILTER_BUFFER_SIZE 16 // Tamanho do buffer do filtro média móvel
#define PI 3.14     // Valor adotado para pi

// Delaração das variáveis globais
unsigned int g_sineMean = 2048;
unsigned int g_sinePeak = 1000;
int g_signalNoiseMax = 50;
int g_signalNoiseMin = -50;
static unsigned int g_iS = 0;
float g_inputSignal;
float g_inputSignalBuffer[BUFFER_SIZE];
float g_noise;
float g_outputSignal;
float g_outputSignalBuffer[BUFFER_SIZE];
float g_filterBuffer[FILTER_BUFFER_SIZE];
static unsigned int g_iF = 0;


// Protótipo das funções
float movingAverage(float value);

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

    while(1)
    {
    }	
	
}


// Função de interrupção do Timer de geração da senoide
__interrupt void INT_sineTimer_ISR(void)
{
    g_inputSignal = g_sineMean + g_sinePeak * sin(2*PI*g_iS/BUFFER_SIZE) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    g_inputSignalBuffer[g_iS] = g_inputSignal;
    g_outputSignal = movingAverage(g_inputSignalBuffer[g_iS]);
    g_outputSignalBuffer[g_iS] = g_outputSignal;
    g_iS = (g_iS+1)%BUFFER_SIZE;
}

// Função de calculo da média movel
float movingAverage(float value)
{
    g_filterBuffer[g_iF] = value;
    float sum = 0.0f;
    unsigned int i;
    for (i = 0U; i < FILTER_BUFFER_SIZE; i++)
    {
        sum += g_filterBuffer[i]; 
    }
    g_iF = (g_iF+1)%FILTER_BUFFER_SIZE;
    return sum / (float)FILTER_BUFFER_SIZE;
}


//
// End of File
//
