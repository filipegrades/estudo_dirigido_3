// Inclusão das bibliotecas
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "stdlib.h"

// Declaração dos defines
#define BUFFER_SIZE 16      // Tamanho do buffer
#define PI 3.14     // Valor adotado para pi

// Delaração das variáveis globais
unsigned int g_sineMean = 2048;
unsigned int g_sinePeak = 1000;
int g_signalNoiseMax = 50;
int g_signalNoiseMin = -50;
static unsigned int g_iS = 0;
float g_inputSignalBuffer[BUFFER_SIZE];
float g_noise;
float g_outputSignalBuffer[BUFFER_SIZE];


// Protótipo das funções
float movingAverage(float *buffer);

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
    g_inputSignalBuffer[g_iS] = g_sineMean + g_sinePeak * sin(2*PI*g_iS/BUFFER_SIZE) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    
    g_outputSignalBuffer[g_iS] = movingAverage(g_inputSignalBuffer);
    g_iS = (g_iS+1)%BUFFER_SIZE;
}

// Função de calculo da média movel
float movingAverage(float *buffer)
{
    float sum = 0.0f;
    unsigned int i;
    for (i = 0U; i < BUFFER_SIZE; i++)
    {
        sum += buffer[i]; 
    }
    return sum / (float)BUFFER_SIZE;
}


//
// End of File
//
