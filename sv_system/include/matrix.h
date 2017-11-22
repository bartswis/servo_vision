
#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

class Matrix {

public:

  double   **element;
  int   m,n;

  Matrix ();
  Matrix (const int m,const int n);
  Matrix (const int m,const int n,const double* element_);
  Matrix (const Matrix &M);
  ~Matrix ();

  Matrix& operator= (const Matrix &M);
  
  Matrix  operator- ();
  Matrix  operator~ ();
  
  Matrix  operator+ (const Matrix &M);
  Matrix  operator- (const Matrix &M);
  Matrix  operator* (const Matrix &M);
  Matrix  operator* (const double &s);
  Matrix  operator/ (const Matrix &M);
  Matrix  operator/ (const double &s);
  
  static Matrix eye (const int m);
  
private:

  void memoryAlloc (const int m_,const int n_);
  void memoryFree ();

};

#endif // MATRIX_H
