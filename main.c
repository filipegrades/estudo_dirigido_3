// Inclusão das bibliotecas
#include "driverlib.h"
#include "device.h"
#include "board.h"
#include "math.h"
#include "stdlib.h"

// Declaração dos defines
#define N 16      // Tamanho do buffer
#define PI 3.14     // Valor adotado para pi

// Delaração das variáveis globais
unsigned int g_sineMean = 2048;
unsigned int g_sinePeak = 1000;
int g_signalNoiseMax = 50;
int g_signalNoiseMin = -50;
static unsigned int g_iS = 0;
float g_inputSignal;
float g_inputSignalBuffer[N];
float g_noise;

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
    g_inputSignal = g_sineMean + g_sinePeak * sin(2*PI*g_iS/N) + (rand() % (g_signalNoiseMax - g_signalNoiseMin + 1)) + g_signalNoiseMin;
    //g_inputSignal = g_sineMean + g_sinePeak * sin(2*PI*g_iS/N);
    g_inputSignalBuffer[g_iS] = g_inputSignal;
    g_iS = (g_iS+1)%N;
}


//
// End of File
//
