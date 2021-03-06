#include "random.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

int random_get_integer(int max) {
  return rand() % max;
}

double random_get_uniform() {
  srand(time(NULL));
  return ((double)rand()) / (RAND_MAX);
}
//double random_get_uniform() {
//    int randomN = rand();
//    if (randomN%2==0){
//        return (double)rand()/RAND_MAX;
//    }else{
//        return -(double)rand()/RAND_MAX;
//    }
//}

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
