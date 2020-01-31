//
//  Auxiliar.c
//  MySNN
//
//  Created by Oscar Miranda Bravo on 12/5/16.
//  Copyright © 2016 Oscar Miranda Bravo. All rights reserved.
//

#include "Auxiliar.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int random_get_integer(int max) {
    return rand() % max;
}

double random_get_uniform() {
    return (double)rand() / RAND_MAX;
}

// polar form of the Box-Muller transformation
double random_get_gaussian() {
    double x1, x2, w;
    do {
        x1 = 2.0 * random_get_uniform() - 1.0;
        x2 = 2.0 * random_get_uniform() - 1.0;
        w = x1 * x1 + x2 * x2;
    }
    while (w >= 1.0);
    
    return x1 * sqrt(-2.0 * log(w) / w);
}