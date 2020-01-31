#include <stdio.h>
#include <stdlib.h>
#include "Auxiliar.h"

#define N 16

#define MAX_VEL 3.0

FILE *fs;  //For writing output

const int NI = 8; // 8 Input: Number of inhibitory neurons (TC)
const int NH = 4; // 4 Hidden: Number of excitatory neurons (IB)
const int NO = 4; // 2 Output: Number of excitatory neurons (RS)
int firecounter;  //Counts the number of fired neurons

int countRecurrence[4] = {0,0,0,0};
int flagRecurrence[4] = {0,0,0,0};
    
struct Neurons{
    double v, u, a, b, c, d;
    int id;
};

double conectivityMat [N][N];
struct Neurons network[N];

void initializeWithWeights(double vectorWeights[72]){
    int i,j;
     //Initialize the network
    for (i = 0; i<N; i++) {
        struct Neurons neuron;
        if (i<NI){
            neuron.a = 0.02;
            neuron.b = 0.25;
            neuron.c = -65;
            neuron.d = 0.05;
        }else if (i<(NI+NH)){
            neuron.a = 0.02;
            neuron.b = 0.2;
            neuron.c = -55;
            neuron.d = 4;
        }else{
            neuron.a = 0.02;
            neuron.b = 0.2;
            neuron.c = -65;
            neuron.d = 8;  
        }
        neuron.v = -65;
        neuron.u = neuron.b * neuron.v;
        neuron.id = i;
        network[i] = neuron;
    }

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
}


double* runInput(double vectorWeights[72], double inputs[8], int new_genotype){
    //printf("New %d \n", new_genotype);
    //fflush(stdout); 
    if(new_genotype == 1) 
        initializeWithWeights(vectorWeights);
          
    int i,j,t;
    double outFirings[4];
    outFirings[0] = 0;
    outFirings[1] = 0;
    outFirings[2] = 0;
    outFirings[3] = 0;
    double count[4];
    count[0] = 0;
    count[1] = 0;
    count[2] = 0;
    count[3] = 0;
    static double velocity[2];
    velocity[0] = 0;
    velocity[1] = 0;
    firecounter = 0;
    double	I[N];
    
    bool fired[N];
    
    // Transfer Sensor Data to Input Neurons
    for (i = 0; i<8; i++) {    
        //printf(":: %d >> %lf \n", i, inputs[i]);
        //fflush(stdout);
        I[i] = inputs[i]*100;
    }
    
    int fireNeurons[N] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    // Run 256ms
    // For each milisecond
    for (t = 0; t<256; t++) {
                
        // Initialisation of the input array
        for (i = 8; i<N; i++) { 
            I[i] = 0;
        }
          
        // Check if membrane potential reached 30mV
        for (i = 0; i<N; i++) {
            if(network[i].v>=30) {
                fireNeurons[i] += 1;
                // Update values if fired 
                fired[i] = true;    
                firecounter++;
                network[i].v = network[i].c;
                network[i].u = network[i].u + network[i].d;
                // Output neurons
                if (i == 12) {
                    count[0] += 1;
                }else if (i == 13) {
                    count[1] += 1;
                }else if (i == 14) {
                    count[2] += 1;
                }else if (i == 15) {
                    count[3] += 1;
                }
            }else{
                fired[i] = false;
            }
        }

        ////////////////////////////////////////
        //  Update values for each iteration  //
        ////////////////////////////////////////
                
        //Input layer:
        for (i=0; i<8; i++) {
            //double sum = (conectivityMat[i][i]*1+10);
            //I[i] = I[i] + sum;    
                            
            network[i].v = network[i].v+0.5*(0.04*network[i].v*network[i].v+5*network[i].v+140-network[i].u+I[i]);
            network[i].v = network[i].v+0.5*(0.04*network[i].v*network[i].v+5*network[i].v+140-network[i].u+I[i]);
            network[i].u = network[i].u+network[i].a*(network[i].b*network[i].v-network[i].u);
        }

        //printf(":::::::::: ConectivityMat %lf \n", conectivityMat[0][0]);
        //fflush(stdout);
        //printf(":::::::::: Neuron %d (time %d)= %lf  ------- Input: %lf\n", 0, t, network[0].v, I[0]);
        //fflush(stdout);
        
        //printf(":::::::::: Neuron %d V = %lf  U = %lf \n", 8, network[8].v, network[8].u);
        //fflush(stdout);
        
        //Hidden layer:
        for (i=8; i<12; i++) {          
            for (j=0; j<8; j++) {
                if (fired[j]){
                    double sum = (conectivityMat[i][j]*5+50);
                    I[i] = I[i] + sum;                    
                }              
            }
            
            /*
            countRecurrence[i-8] += 1;
            //printf("\n >> Neuron %d Fired: %d countRecurrence: %d flagRecurrence: %d", i, fired[i],countRecurrence[i-8],flagRecurrence[i-8]);
            //fflush(stdout);
            if (fired[i]){
                flagRecurrence[i-8] = 1;
                countRecurrence[i-8] = 0;
                double sum = (conectivityMat[i][i]*1+10);
                I[i] = I[i] + sum;                    
            }else if(countRecurrence[i-8] < 5 && flagRecurrence[i-8]){
                double sum = (conectivityMat[i][i]*1+10);
                I[i] = I[i] + sum;
            }else{
                flagRecurrence[i-8] = 0;  
                countRecurrence[i-8] = 0;
            } 
            */          
            
            network[i].v = network[i].v+0.5*(0.04*network[i].v*network[i].v+5*network[i].v+140-network[i].u+I[i]);
            network[i].v = network[i].v+0.5*(0.04*network[i].v*network[i].v+5*network[i].v+140-network[i].u+I[i]);
            network[i].u = network[i].u+network[i].a*(network[i].b*network[i].v-network[i].u);
        }
        
        //printf(":::::::::: Neuron %d (time %d)= %lf  ------- Input: %lf\n", 8, t, network[8].v, I[8]);
        //fflush(stdout);
        
        //Output layer
        for (j = 12; j<16; j++) {            
            for (i=8; i<12; i++) {
                if (fired[i]){
                    double sum = (conectivityMat[j][i]*5+50);
                    I[j] = I[j] + sum;
                }
            }
            network[j].v = network[j].v+0.5*(0.04*network[j].v*network[j].v+5*network[j].v+140-network[j].u+I[j]);
            network[j].v = network[j].v+0.5*(0.04*network[j].v*network[j].v+5*network[j].v+140-network[j].u+I[j]);
            network[j].u = network[j].u+network[j].a*(network[j].b*network[j].v-network[j].u);            
        }
        
        //printf(":::::::::: Neuron %d (time %d)= %lf  ------- Input: %lf\n", 12, t, network[12].v, I[12]);
        //fflush(stdout);
    }
    
    /*
    for (i = 0; i<N; i++) {
        printf(":::::::::: %d >> %d \n", i, fireNeurons[i]);
        fflush(stdout);
    }
    */
    
    // Max 
    double max = 0; 
    double min = 50;
    
    outFirings[0] = MAX_VEL*((count[0]-min)/(max-min));
    outFirings[1] = -MAX_VEL*((count[1]-min)/(max-min));
    outFirings[2] = MAX_VEL*((count[2]-min)/(max-min));
    outFirings[3] = -MAX_VEL*((count[3]-min)/(max-min)); 
    
    
    /*
    printf("Count 0: %lf \n", count[0]);
    fflush(stdout);     
    printf("Count 1: %lf \n", count[1]);
    fflush(stdout);   
    printf("Count 2: %lf \n", count[2]);
    fflush(stdout);   
    printf("Count 3: %lf \n", count[3]);
    fflush(stdout);  
    */
    
    /*
    printf("outFirings 0: %lf \n", outFirings[0]);
    fflush(stdout);     
    printf("outFirings 1: %lf \n", outFirings[1]);
    fflush(stdout);   
    printf("outFirings 2: %lf \n", outFirings[2]);
    fflush(stdout);   
    printf("outFirings 3: %lf \n", outFirings[3]);
    fflush(stdout);  
    */
           
    // Mapping the sequence of spikings to velocity
    velocity[0] = (outFirings[0] + outFirings[1])/2;
    velocity[1] = (outFirings[2] + outFirings[3])/2;    
    
    if(velocity[0] > MAX_VEL) velocity[0] = MAX_VEL;
    if(velocity[1] > MAX_VEL) velocity[1] = MAX_VEL;
    if(velocity[0] < -MAX_VEL) velocity[0] = -MAX_VEL;
    if(velocity[1] < -MAX_VEL) velocity[1] = -MAX_VEL;
    
    //printf("V1: %lf \n", velocity[0]);
    //fflush(stdout);   
    //printf("V2: %lf \n", velocity[1]);
    //fflush(stdout); 

    return velocity;
}
