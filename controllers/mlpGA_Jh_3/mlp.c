//
//  mlp.c
//  

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Auxiliar.h"

#define N 16

#define MAX_VEL 3.0

FILE *fs;  //For writing output

const int NI = 8; // Input neurons
const int NH = 4; // Hidden neurons
const int NO = 4; // Output neurons

struct Neurons{
    double input,output;
    int id;
};

double conectivityMat [N][N];
struct Neurons network[N];

void initializeWithWeights(double vectorWeights[72]){
    int i,j;
     //Initialize the network
    for (i = 0; i<N; i++) {
        struct Neurons neuron;
        neuron.id = i;
        neuron.input = 0;
        neuron.output = 0;
        network[i] = neuron;
    }
    i = 0;
    for (i=0; i<N; i++) {
        for (j=0; j<N; j++) {
            conectivityMat[i][j] = 0;
        }
    }
    
    // Input layer only
    for (int j = 0; j < 8; j++)
    {
        conectivityMat[j][j] = vectorWeights[j];
    }                   
    
    // Input layer to hidden layer matrix
    for (int i = 1; i < 5; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            conectivityMat[i+7][j] = vectorWeights[8*i+j];
        }
    }
    
    // Hidden layer to output layer matrix
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            conectivityMat[i+12][j+8] = vectorWeights[39+4*i+j];
        }
    }
    
    // Recurrent - hidden
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            conectivityMat[i+8][j+8] = vectorWeights[55+4*i+j];
        }
    }
    
    // Recurrent - output
    //for (int i = 0; i < 4; i++)
    //{
    //    for (int j = 0; j < 4; j++)
    //    {
    //        conectivityMat[i+12][j+12] = vectorWeights[71+4*i+j];
    //    }
    //}
    
}

float activationFunction(float x){ 
    // Sigmoid or Logistic
    //float af = (1/(1+exp(-x)));
    
    // Hyperbolic Tangent
    float af = ((exp(2*x)-1)/(exp(2*x)+1))*10;
    
    //printf("x = %f, exp = %f \n", x, af);
    //fflush(stdout);
    return af;
}

double* runInput(double vectorWeights[72], double inputs[8], int new_genotype){

    if(new_genotype == 1) 
        initializeWithWeights(vectorWeights);

    int i,j;
    static double outFirings[2];
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
    
    
    //for (i = 0; i<N; i++) { 
    //    printf(":::::::::: Neuron = %lf \n", network[i].output);
    //    fflush(stdout);
    //}
    
    ////////////////////
    //  Feed-forward  //
    ////////////////////
            
    //Input layer:
    for (i=0; i<8; i++) {
        //printf(":::::::::: Neuron = %d \n", i);
        //fflush(stdout); 
        //printf("InputW = %f, InputV = %f \n", conectivityMat[i][i], I[i]);
        //fflush(stdout);
        network[i].input = conectivityMat[i][i] * I[i]; // [-100 .. 100]
        // Normalize to [-3..3]
        network[i].input += 100; // [0 .. 200]
        network[i].input /= 33;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout);
    }
    
    //Hidden layer:
    for (i=8; i<12; i++) {
        //printf(":::::::::: Neuron = %d \n", i);
        //fflush(stdout); 
        for (j=0; j<8; j++) {   
            //printf("HiddenW = %f, HiddenV = %f \n", conectivityMat[i][j], network[j].output);
            //fflush(stdout);         
            network[i].input += (conectivityMat[i][j] * network[j].output); // [-100 .. 100] * 8
        }
        //for (j=8; j<12; j++) { 
        //    //printf("HiddenW = %f, HiddenV = %f \n", conectivityMat[i][j], network[j].output);
        //    //fflush(stdout);            
        //    network[i].input += (conectivityMat[i][j] * network[j].output); // [-100 .. 100] * 4
        //}
        // Normalize to [-3..3]
        network[i].input += 1200; // [0 .. 2400]
        network[i].input /= 400;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]                
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT: %f \n\n", network[i].output);
        //fflush(stdout);
    }
    
    //Output layer
    for (i = 12; i<16; i++) {
        //printf(":::::::::: Neuron = %d \n", i);
        //fflush(stdout); 
        for (j=8; j<12; j++) {
            //printf("OutputW = %f, OutputV = %f \n", conectivityMat[i][j], network[j].output);
            //fflush(stdout); 
            network[i].input += conectivityMat[i][j] * network[j].output; // [-100 .. 100] * 4
        }
        //for (j=12; j<16; j++) {
        //    //printf("OutputW = %f, OutputV = %f \n", conectivityMat[i][j], network[j].output);
        //    //fflush(stdout); 
        //    network[i].input += conectivityMat[i][j] * network[j].output; // [-100 .. 100] * 4
        //}
        // Normalize to [-3..3]
        network[i].input += 800; // [0 .. 1600]
        network[i].input /= 266;  // [0 .. 6]
        network[i].input -= 3;   // [-3 .. 3]    
        network[i].output = activationFunction(network[i].input);
        //printf("OUTPUT %d : %f \n\n",i, network[i].output);
        //fflush(stdout); 
    }
    
    double positiveLeftWheel  = ((network[12].output+10)*0.15);       // network[12].output; 
    double negativeRightWheel = ((network[13].output+10)*0.15)*(-1);  // network[13].output;  
    double positiveRightWheel = ((network[14].output+10)*0.15);       // network[14].output;
    double negativeleftWheel  = ((network[15].output+10)*0.15)*(-1);  // network[15].output;
    
    outFirings[0] = positiveLeftWheel  + negativeleftWheel;
    outFirings[1] = positiveRightWheel + negativeRightWheel;
    
    if(outFirings[0] > MAX_VEL) outFirings[0] = MAX_VEL;
    if(outFirings[1] > MAX_VEL) outFirings[1] = MAX_VEL;
    if(outFirings[0] < -MAX_VEL) outFirings[0] = -MAX_VEL;
    if(outFirings[1] < -MAX_VEL) outFirings[1] = -MAX_VEL;
    
    //printf("Velocities: %f, %f \n\n", outFirings[0], outFirings[1]);
    //fflush(stdout); 
    return outFirings;
}

