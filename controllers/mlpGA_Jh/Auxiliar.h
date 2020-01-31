//
//  Auxiliar.h
//  MySNN
//
//  Created by Oscar Miranda Bravo on 12/5/16.
//  Copyright © 2016 Oscar Miranda Bravo. All rights reserved.
//

#ifndef Auxiliar_h
#define Auxiliar_h

#include <stdio.h>

//   Description:   Random number functions

// returns a random integer number between [0;max-1] from a uniform distribution
int random_get_integer(int max);

// returns a random number between [0;1] from a uniform distribution
double random_get_uniform();

// returns a random number from a Gaussian distribution with mean 0 and standard deviation 1
double random_get_gaussian();





#endif /* Auxiliar_h */
