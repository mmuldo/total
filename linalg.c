#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "linalg.h"

double dot(double vector1[], double vector2[], int dimension) {
    double result = 0;

    for (int i = 0; i < dimension; i++) {
        result += vector1[i]*vector2[i];
    }

    return result;
}

double norm(double vector[], int dimension) {
    return sqrt(dot(vector, vector, dimension));
}