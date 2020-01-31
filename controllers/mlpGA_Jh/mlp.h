//
//  mlp.h
//

#ifndef mlp_h
#define mlp_h

#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Auxiliar.h"

typedef struct _Neurons_ Neurons;

//Methods:
void initializeWithWeights(double vectorWeights[72]);
float activationFunction(float x);
double* runInput(double vectorWeights[72], double inputs[8], int new_genotype);

#endif /* mlp_h */
