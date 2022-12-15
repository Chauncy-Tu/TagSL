#include <stdlib.h>
#include <math.h>
#include <stdio.h>

typedef struct Matrix matrix;

struct Matrix{
	double** A;
	int m;
	int n;
	double det;
};

matrix* Minit(double** a, int m,int n);

