
#include "matrix.h"
#include <math.h>

void Matrix::memoryAlloc (const int m_,const int n_)
{
    m = abs(m_); n = abs(n_);
    
    if (m==0 || n==0)
    {
        element = 0;
        return;
    }
    
    element = (double**)malloc(m*sizeof(double*));
    element[0] = (double*)calloc(m*n,sizeof(double));
    
    for(int i=1; i<m; i++)
    {
        element[i] = element[i-1]+n;
    }
}

void Matrix::memoryFree ()
{
    if (element!=0)
    {
        free(element[0]);
        free(element);
    }
}

Matrix::Matrix ()
{
    m = 0;
    n = 0;
    element = 0;
}

Matrix::Matrix (const int m_,const int n_)
{
    memoryAlloc(m_,n_);
}

Matrix::Matrix (const int m_,const int n_,const double* element_)
{
    memoryAlloc(m_,n_);
    int k=0;
    
    for (int i=0; i<m_; i++)
    {
        for (int j=0; j<n_; j++)
        {
            element[i][j] = element_[k++];
        }
    }
}

Matrix::Matrix (const Matrix &M)
{
    memoryAlloc(M.m,M.n);
    
    for (int i=0; i<M.m; i++)
    {
        memcpy(element[i],M.element[i],M.n*sizeof(double));
    }
}

Matrix::~Matrix () 
{
    memoryFree();
}

Matrix& Matrix::operator= (const Matrix &M)
{
    if (this!=&M)
    {
        if (M.m!=m || M.n!=n)
        {
            memoryFree();
            memoryAlloc(M.m,M.n);
        }
        if (M.n>0)
        {
            for (int i=0; i<M.m; i++)
            {
                memcpy(element[i],M.element[i],M.n*sizeof(double));
            }
        }
    }
    
    return *this;
}

Matrix Matrix::operator- ()
{
    Matrix C(m,n);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[i][j] = -element[i][j];
        }
    }
    
    return C;
}

Matrix Matrix::operator~ ()
{
    Matrix C(n,m);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[j][i] = element[i][j];
        }
    }
    
    return C;
}

Matrix Matrix::operator+ (const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    
    Matrix C(A.m,A.n);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[i][j] = A.element[i][j]+B.element[i][j];
        }
    }
    
    return C;
}

Matrix Matrix::operator- (const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    
    Matrix C(A.m,A.n);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[i][j] = A.element[i][j]-B.element[i][j];
        }
    }
    
    return C;
}

Matrix Matrix::operator* (const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    
    Matrix C(A.m,B.n);
    
    for (int i=0; i<A.m; i++)
    {
        for (int j=0; j<B.n; j++)
        {
            for (int k=0; k<A.n; k++)
            {
                C.element[i][j] += A.element[i][k]*B.element[k][j];
            }
        }
    }
    
    return C;
}

Matrix Matrix::operator* (const double &s)
{
    Matrix C(m,n);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[i][j] = element[i][j]*s;
        }
    }
    
    return C;
}

Matrix Matrix::operator/ (const Matrix &M)
{
    const Matrix &A = *this;
    const Matrix &B = M;
    
    Matrix C(A.m,A.n);
    
    if (A.m==B.m && A.n==B.n)
    {
        for (int i=0; i<A.m; i++)
        {
            for (int j=0; j<A.n; j++)
            {
                if (B.element[i][j]!=0)
                {
                    C.element[i][j] = A.element[i][j]/B.element[i][j];
                }
            }
        }
    } 
    else if (A.m==B.m && B.n==1)
    {
        for (int i=0; i<A.m; i++)
        {
            for (int j=0; j<A.n; j++)
            {
                if (B.element[i][0]!=0)
                {
                    C.element[i][j] = A.element[i][j]/B.element[i][0];
                }
            }
        }
    } 
    else if (A.n==B.n && B.m==1)
    {
        for (int i=0; i<A.m; i++)
        {
            for (int j=0; j<A.n; j++)
            {
                if (B.element[0][j]!=0)
                {
                    C.element[i][j] = A.element[i][j]/B.element[0][j];
                }
            }
        }
    }
    
    return C;
}

Matrix Matrix::operator/ (const double &s)
{
    Matrix C(m,n);
    
    for (int i=0; i<m; i++)
    {
        for (int j=0; j<n; j++)
        {
            C.element[i][j] = element[i][j]/s;
        }
    }
    
    return C;
}

Matrix Matrix::eye (const int m)
{
    Matrix M(m,m);
    
    for (int i=0; i<m; i++)
    {
        M.element[i][i] = 1;
    }
    
    return M;
}
