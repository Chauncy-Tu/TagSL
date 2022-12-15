#include <Mycmatrix.h>
#include <stdlib.h>

matrix* Minit(double** a, int m,int n)
{
  matrix* Mat;
  Mat = (matrix*)malloc(sizeof(matrix));
  printf("\r\nm=%d",m);
  printf("\r\nn=%d",n);

  Mat->A=(double**)malloc(m * sizeof(double));
  for(int i=0;i<m;i++)
  {
    Mat->A[i]=(double*)malloc(n * sizeof(double));
  }

  //double **A=(double**)malloc(4 * sizeof(double));
  //for(int i=0;i<4;i++)
  //{
  //  A[i]=(double*)malloc(3 * sizeof(double));
  //}


  for(int i=0;i<m;i++)
  {
    for(int j=0;j<n;j++)
    {
      Mat->A[i][j]=a[i][j];
    }
  }
  Mat->m=m;
  Mat->n=n;
  return Mat;
  
}
