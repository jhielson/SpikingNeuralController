//
//  snn.h
//  MySNN
//
//  Created by Oscar Miranda Bravo on 12/5/16.
//  Copyright © 2016 Oscar Miranda Bravo. All rights reserved.
//

#ifndef snn_h
#define snn_h

#include <stdio.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Auxiliar.h"

typedef struct _Neurons_ Neurons;

//Methods:
void initializeWithWeights(double vectorWeights[72]);
double* runInput(double vectorWeights[72], double inputs[8], int new_genotype);


#endif /* snn_h */
