//
//  mlp.c
//  

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Auxiliar.h"

#define N 16

FILE *fs;  //For writing output

const int NI = 8; // Input neurons
const int NH = 4; // Hidden neurons
const int NO = 4; // Output neurons

struct Neurons{
    float input,output;
    int id;
};

float conectivityMat [N][N];
struct Neurons network[N];


void initializeWithWeights(float weights1[5][8], float weights2[4][4]){
    int i,j;
     //Initialize the network
    for (i = 0; i<N; i++) {
        struct Neurons neuron;
        neuron.id = i;
        neuron.input = 0;
        neuron.output = 0;
        network[i] = neuron;
    }
    // i = 0;
    for (i=0; i<N; i++) {
        for (j=0; j<N; j++) {
            conectivityMat[i][j] = 0;
        }
    }
    
    // Input layer only
    for (j = 0; j < 8; j++)
    {
        conectivityMat[j][j] = weights1[0][j];
    }                   
    
    // Input layer to hidden layer matrix
    for (i = 1; i < 5; i++)
    {
        for (j = 0; j < 8; j++)
        {
            conectivityMat[i+7][j] = weights1[i][j];
        }
    }
    
    // Hidden layer to output layer matrix
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            conectivityMat[i+12][j+8] = weights2[i][j];
        }
    }
    
}

void initializeWithWeights2(){
    int i,j;
     //Initialize the network
    for (i = 0; i<N; i++) {
        struct Neurons neuron;
        neuron.id = i;
        neuron.input = 0;
        neuron.output = 0;
        network[i] = neuron;
    }

    for (i=0; i<N; i++) {
        for (j=0; j<N; j++) {
            conectivityMat[i][j] = 0;
        }
    }
    
   
    
    // Input layer only
    conectivityMat[0][0] = -4.007387;
    conectivityMat[1][1] =  5.457971;
    conectivityMat[2][2] = -10.00000;
    conectivityMat[3][3] =  9.648352;
    conectivityMat[4][4] = -1.237472;
    conectivityMat[5][5] =  4.415173;
    conectivityMat[6][6] =  8.942057;
    conectivityMat[7][7] =  5.608282;
                  
    // Input layer to hidden layer matrix
    conectivityMat[8][0] = 4.510410;
    conectivityMat[8][1] = 3.262457;
    conectivityMat[8][2] = 1.663498;
    conectivityMat[8][3] = -10.0000;
    conectivityMat[8][4] = -0.834150;
    conectivityMat[8][5] = 10.000000;
    conectivityMat[8][6] = 10.000000;
    conectivityMat[8][7] = 1.818801;
    
    conectivityMat[9][0] = 0.029110;
    conectivityMat[9][1] = -10.000000;
    conectivityMat[9][2] = -10.000000;
    conectivityMat[9][3] = 10.000000;
    conectivityMat[9][4] = -10.000000;
    conectivityMat[9][5] = -7.417870;
    conectivityMat[9][6] = -3.512601;
    conectivityMat[9][7] = 2.050406;    
    
    conectivityMat[10][0] = 6.838716;
    conectivityMat[10][1] = 6.167096;
    conectivityMat[10][2] = -10.000000;
    conectivityMat[10][3] = -10.000000;
    conectivityMat[10][4] = -1.194183;
    conectivityMat[10][5] = -8.232101;
    conectivityMat[10][6] = 10.000000;
    conectivityMat[10][7] = 10.000000;
    
    conectivityMat[11][0] = -0.866501;
    conectivityMat[11][1] = -1.953355;
    conectivityMat[11][2] = -10.000000;
    conectivityMat[11][3] = 10.000000;
    conectivityMat[11][4] = -10.000000;
    conectivityMat[11][5] = -0.458687;
    conectivityMat[11][6] = 6.262162;
    conectivityMat[11][7] = 10.000000;
    
    // Hidden layer to output layer matrix
    conectivityMat[12][0] = 0.645230;
    conectivityMat[12][1] = -10.000000;
    conectivityMat[12][2] = 10.000000;
    conectivityMat[12][3] = -10.000000;

    conectivityMat[13][0] = -10.000000;
    conectivityMat[13][1] = 10.000000;
    conectivityMat[13][2] = 10.000000;
    conectivityMat[13][3] = 8.052084;
    
    conectivityMat[14][0] = 10.000000;
    conectivityMat[14][1] = -10.000000;
    conectivityMat[14][2] = -2.136456;
    conectivityMat[14][3] = -10.000000;
    
    conectivityMat[15][0] = -10.000000;
    conectivityMat[15][1] = 10.000000;
    conectivityMat[15][2] =  3.628583;
    conectivityMat[15][3] =  10.000000; 

}

float activationFunction(float x){ 
    // Sigmoid or Logistic
    //float af = (1/(1+exp(-x)));
    
    // Hyperbolic Tangent
    float af = ((1-exp(-2*x))/(1+exp(-2*x)))*10;
    
    //printf("x = %f, exp = %f \n", x, af);
    //fflush(stdout);
    return af;
}

float* runInput(float matrix1[5][8], float matrix2[4][4], float inputs[8]){
    initializeWithWeights(matrix1, matrix2);
    int i,j;
    static float outFirings[2];
    outFirings[0] = 0;
    outFirings[1] = 0;
    float	I[N];
    
    //Initialisation of the input array
    for (i = 0; i<N; i++) { 
        I[i] = 0;
    }

    // Transfer Sensor Data to Input Neurons
    for (i = 0; i<8; i++) {
        I[i] = (inputs[i]*20)-10; ///////////////////////////////////////////////////////////////////////
    }
    
    ////////////////////
    //  Feed-forward  //
    ////////////////////
            
    //Input layer:
    for (i=0; i<8; i++) {
        network[i].input = conectivityMat[i][i] * I[i]; // [-100 .. 100]
        // Normalize to [-2..2]
        network[i].input += 100; // [0 .. 200]
        network[i].input /= 33;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]
        network[i].output = activationFunction(network[i].input);
    }
    
    //Hidden layer:
    for (i=8; i<12; i++) {
        for (j=0; j<8; j++) {            
            network[i].input += (conectivityMat[i][j] * network[j].output); // [-100 .. 100] * 8
        }
        // Normalize to [-2..2]
        network[i].input += 800; // [0 .. 1600]
        network[i].input /= 266;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]                
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout);
    }
    
    //Output layer
    for (i = 12; i<16; i++) {
        for (j=8; j<12; j++) {
            network[i].input += conectivityMat[i][j] * network[j].output; // [-100 .. 100] * 4
        }
        // Normalize to [-2..2]
        network[i].input += 400; // [0 .. 800]
        network[i].input /= 133;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]    
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout); 
    }
    
    float positiveLeftWheel  = ((network[12].output+10)*0.5);       // network[12].output; 
    float negativeRightWheel = ((network[13].output+10)*0.5)*(-1);  // network[13].output;  
    float positiveRightWheel = ((network[14].output+10)*0.5);       // network[14].output;
    float negativeleftWheel  = ((network[15].output+10)*0.5)*(-1);  // network[15].output;
    
    outFirings[0] = positiveLeftWheel  + negativeleftWheel;
    outFirings[1] = positiveRightWheel + negativeRightWheel;
    
    //printf("Velocities: %f, %f \n\n", outFirings[0], outFirings[1]);
    //fflush(stdout); 
    return outFirings;
}

float* runInput2(float inputs[8]){
    initializeWithWeights2();
    int i,j;
    static float outFirings[2];
    outFirings[0] = 0;
    outFirings[1] = 0;
    float	I[N];
    
    //Initialisation of the input array
    for (i = 0; i<N; i++) { 
        I[i] = 0;
    }

    // Transfer Sensor Data to Input Neurons
    for (i = 0; i<8; i++) {
        I[i] = (inputs[i]*20)-10; ///////////////////////////////////////////////////////////////////////
    }
    
    ////////////////////
    //  Feed-forward  //
    ////////////////////
            
    //Input layer:
    for (i=0; i<8; i++) {
        network[i].input = conectivityMat[i][i] * I[i]; // [-100 .. 100]
        // Normalize to [-2..2]
        network[i].input += 100; // [0 .. 200]
        network[i].input /= 33;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]
        network[i].output = activationFunction(network[i].input);
    }
    
    //Hidden layer:
    for (i=8; i<12; i++) {
        for (j=0; j<8; j++) {            
            network[i].input += (conectivityMat[i][j] * network[j].output); // [-100 .. 100] * 8
        }
        // Normalize to [-2..2]
        network[i].input += 800; // [0 .. 1600]
        network[i].input /= 266;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]                
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout);
    }
    
    //Output layer
    for (i = 12; i<16; i++) {
        for (j=8; j<12; j++) {
            network[i].input += conectivityMat[i][j] * network[j].output; // [-100 .. 100] * 4
        }
        // Normalize to [-2..2]
        network[i].input += 400; // [0 .. 800]
        network[i].input /= 133;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]    
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout); 
    }
    
    float positiveLeftWheel  = ((network[12].output+10)*0.5);       // network[12].output; 
    float negativeRightWheel = ((network[13].output+10)*0.5)*(-1);  // network[13].output;  
    float positiveRightWheel = ((network[14].output+10)*0.5);       // network[14].output;
    float negativeleftWheel  = ((network[15].output+10)*0.5)*(-1);  // network[15].output;
    
    outFirings[0] = positiveLeftWheel  + negativeleftWheel;
    outFirings[1] = positiveRightWheel + negativeRightWheel;
    
    //printf("Velocities: %f, %f \n\n", outFirings[0], outFirings[1]);
    //fflush(stdout); 
    return outFirings;
}

