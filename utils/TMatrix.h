#ifndef TMATRIX_H
#define TMATRIX_H

/**
 * @file TMatrix.h
 *
 * @brief A dimension-templatized class for matrices of values.
 */
#include <math.h>
#include <cstdlib>
#include <iostream>
#include "our_int_types.h"

#ifndef USE_LAPACK
void svdcmp(double **a, int m, int n, double *w, double **v, int *failed_p = (int*)NULL);
#else
extern "C" {
  // Lapack SVD
  void dgesvd_(char *jobu, char *jobvt,
               int *m, int *n, double *a,
               int *lda, double *s, double *u, int *ldu,
               double *vt, int *ldvt, double *work, int *lwork,
               int *info);
}
#endif


template <uint32_t, uint32_t, typename> class BasicMatrix;

/**
 * @brief Class that layers on operator[] functionality for
 * typical matrices.
 */
template <uint32_t Rows, uint32_t Cols, typename value_type>
  class BasicIndexMatrix : public BasicMatrix<Rows,Cols,value_type> {
    typedef BasicMatrix<Rows,Cols,value_type> BaseType;
 protected:
 BasicIndexMatrix(bool dummy) : BaseType(dummy) { }
 public:
    BasicIndexMatrix() { }
 BasicIndexMatrix(const value_type* data) : BaseType(data) {}

    const value_type* operator[](uint32_t r) const {
#ifndef IGNORE_BOUNDS_CHECKING
      if (r < Rows) {
        return &BaseType::mData[r*Cols];
      }
      std::clog << "Invalid row index " << r << std::endl;
      return &BaseType::mData[0];
#else
    return &BaseType::mData[r*Cols];
#endif
    }

    value_type* operator[](uint32_t r) {
#ifndef IGNORE_BOUNDS_CHECKING
      if (r < Rows) {
        return &BaseType::mData[r*Cols];
      }
      std::clog << "Invalid row index " << r << std::endl;
      return &BaseType::mData[0];
#else
      return &BaseType::mData[r*Cols];
#endif
    }
};

/**
 * @brief Specialization of BasicIndexMatrix that provides
 * single-indexing operator for column vectors.
 */
template <uint32_t Rows, typename value_type>
  class BasicIndexMatrix<Rows,1,value_type> :
  public BasicMatrix<Rows,1,value_type> {
    typedef BasicMatrix<Rows,1,value_type> BaseType;
  protected:
  BasicIndexMatrix(bool dummy) : BaseType(dummy) {}
  public:
    BasicIndexMatrix() { }
  BasicIndexMatrix(const value_type* data) : BaseType(data) {}

    value_type operator[](uint32_t r) const {
#ifndef IGNORE_BOUNDS_CHECKING
      if (r < Rows) {
        return BaseType::mData[r];
      }
      std::clog << "Invalid vector index " << r << std::endl;
      return BaseType::mData[0];
#else
        return BaseType::mData[r];
#endif
    }
    value_type& operator[](uint32_t r) {
#ifndef IGNORE_BOUNDS_CHECKING
      if (r < Rows) {
        return BaseType::mData[r];
      }
      std::clog << "Invalid vector index " << r << std::endl;
      return BaseType::mData[0];
#else
        return BaseType::mData[r];
#endif
    }

    value_type norm() const {
      /*double normSum = 0;
        for (uint32_t i = 0; i < Rows; i++) {
        normSum += BaseType::mData[i]*BaseType::mData[i];
        }*/
      return sqrt(dot(*this));
    }

    value_type dot(const BasicIndexMatrix<Rows,1,value_type>& rhs) const {
      double sum = 0;
      for (uint32_t i = 0; i < Rows; i++) {
        sum += BaseType::mData[i]*rhs.mData[i];
      }
      return sum;
    }
  };

/**
 * @brief A dimension-templatized class for matrices of values.
 *
 * This template class generically supports any constant-sized
 * matrix of values.  The @p Rows and @p Cols template parameters
 * define the size of the matrix at @e compile-time.  Hence, the
 * size of the matrix cannot be chosen at runtime.  However, the
 * dimensions are appropriately type-checked at compile-time where
 * possible.
 *
 * By default, the matrix contains values of type @p double.  The @p
 * value_type template parameter may be selected to allow matrices
 * of integers or floats.
 *
 * @note At present, type cohersion between matrices with different
 * @p value_type parameters is not implemented.  It is recommended
 * that matrices with value type @p double be used for all numerical
 * computation.
 *
 * Note that the special cases of row and column vectors are
 * subsumed by this class.
 */
template <uint32_t Rows, uint32_t Cols, typename value_type = double>
  class TMatrix : public BasicIndexMatrix<Rows, Cols, value_type> {
  typedef BasicIndexMatrix<Rows, Cols, value_type>  BaseType;
 public:
 TMatrix() : BaseType() { }
 TMatrix(const value_type* data) : BaseType(data) {}
};

/**
 * @brief Template specialization of TMatrix for Vector4.
 */
template <typename value_type>
class TMatrix<4,1, value_type> : public BasicIndexMatrix<4,1, value_type> {
  typedef BasicIndexMatrix<4, 1, value_type>  BaseType;
 public:
  TMatrix() { }
 TMatrix(const value_type* data) : BaseType(data) {}
 TMatrix(value_type a0, value_type a1, value_type a2, value_type a3) : BaseType(false) {
    BaseType::mData[0] = a0;
    BaseType::mData[1] = a1;
    BaseType::mData[2] = a2;
    BaseType::mData[3] = a3;
  }
};

/**
 * @brief Template specialization of TMatrix for Vector3.
 */
template <typename value_type>
class TMatrix<3,1, value_type> : public BasicIndexMatrix<3,1, value_type> {
  typedef BasicIndexMatrix<3, 1, value_type>  BaseType;
 public:
  TMatrix() { }
 TMatrix(const value_type* data) : BaseType(data) {}
 TMatrix(value_type a0, value_type a1, value_type a2) : BaseType(false) {
    BaseType::mData[0] = a0;
    BaseType::mData[1] = a1;
    BaseType::mData[2] = a2;
  }

  TMatrix<3,1,value_type> cross(const TMatrix<3,1, value_type>& v) const {
    const TMatrix<3,1,value_type>& u = *this;
    return TMatrix<3,1,value_type>(u[1]*v[2]-u[2]*v[1],
                                   u[2]*v[0]-u[0]*v[2],
                                   u[0]*v[1]-u[1]*v[0]);
  }
};

/**
 * @brief Template specialization of TMatrix for Vector2.
 */
template <typename value_type>
class TMatrix<2,1, value_type> : public BasicIndexMatrix<2,1, value_type> {
  typedef BasicIndexMatrix<2, 1, value_type>  BaseType;
 public:
  TMatrix() { }
 TMatrix(const value_type* data) : BaseType(data) {}
 TMatrix(value_type a0, value_type a1) : BaseType(false) {
    BaseType::mData[0] = a0;
    BaseType::mData[1] = a1;
  }
};

/**
 * @brief Template specialization of TMatrix for a 1x1 vector.
 */
template <typename value_type>
class TMatrix<1,1, value_type> : public BasicIndexMatrix<1,1, value_type> {
  typedef BasicIndexMatrix<1, 1, value_type>  BaseType;
 public:
  TMatrix() { }
 TMatrix(const value_type* data) : BaseType(data) {}

  // explicit conversion from value_type
  explicit TMatrix(value_type a0) : BaseType(false) { // don't initialize
    BaseType::mData[0] = a0;
  }

  // implicit conversion to value_type
  operator value_type() const {
    return BaseType::mData[0];
  }

  const TMatrix<1,1, value_type>&
    operator=(const TMatrix<1,1, value_type>& m) {
    BaseType::operator=(m);
    return *this;
  }

  double operator=(double a0) {
    BaseType::mData[0] = a0;
    return BaseType::mData[0];
  }
};


/**
 * @brief Base class implementing standard matrix functionality.
 */
template <uint32_t Rows, uint32_t Cols, typename value_type = double>
  class BasicMatrix {
 protected:
 value_type mData[Rows*Cols];

 // Constructs uninitialized matrix.
 BasicMatrix(bool dummy) {}
 public:
 BasicMatrix() { // constructs zero matrix
   for (uint32_t i = 0; i < Rows*Cols; ++i) {
     mData[i] = 0;
   }
 }
 BasicMatrix(const value_type* data) { // constructs from array
   if (!data) {
     std::cerr << "Constructing matrix from 0 array." << std::endl;
     for (uint32_t i= 0; i < Rows*Cols; ++i) mData[i] = 0;
   } else {
     for (uint32_t i = 0; i < Rows*Cols; ++i) {
       mData[i] = data[i];
     }
   }
 }

 uint32_t rows() const { return Rows; }
 uint32_t columns() const { return Cols; }

 value_type element(uint32_t row, uint32_t col) const {
#ifndef IGNORE_BOUNDS_CHECKING
   if (row >= rows() || col >= columns()) {
     std::cerr << "Illegal read access: " << row << ", " << col
     << " in " << Rows << "x" << Cols << " matrix." << std::endl;
     return mData[0];
   }
#endif
   return mData[row*Cols+col];
 }

 value_type& element(uint32_t row, uint32_t col) {
#ifndef IGNORE_BOUNDS_CHECKING
   if (row >= rows() || col >= columns()) {
     std::cerr << "Illegal read access: " << row << ", " << col
     << " in " << Rows << "x" << Cols << " matrix." << std::endl;
     return mData[0];
   }
#endif
   return mData[row*Cols+col];
 }

 void setElement(uint32_t row, uint32_t col, value_type value) {
#ifndef IGNORE_BOUNDS_CHECKING
   if (row >= rows() || col >= columns()) {
     std::cerr << "Illegal write access: " << row << ", " << col
     << " in " << Rows << "x" << Cols << " matrix." << std::endl;
     return ;
   }
#endif
   mData[row*Cols+col] = value;
 }

 template <uint32_t RowRangeSize, uint32_t ColRangeSize>
 TMatrix<RowRangeSize, ColRangeSize, value_type>
 subMatrix(const TMatrix<1, RowRangeSize, uint32_t>& rowRange,
           const TMatrix<1, ColRangeSize, uint32_t>& colRange) const {
   TMatrix<RowRangeSize, ColRangeSize, value_type> result;
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       result.setElement(i,j, element(rowRange.element(0, i), colRange.element(0,j)));
     }
   }
   return result;
 }


 template <uint32_t RowRangeSize, uint32_t ColRangeSize>
 void setSubMatrix(const TMatrix<1, RowRangeSize, uint32_t>& rowRange,
                   const TMatrix<1, ColRangeSize, uint32_t>& colRange,
                   const TMatrix<RowRangeSize, ColRangeSize>& m) const {
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       setElement(rowRange.element(0, i), colRange.element(0,j), m.element(i,j));
     }
   }
 }


 template <uint32_t R, uint32_t C, uint32_t RowRangeSize, uint32_t ColRangeSize>
 TMatrix<RowRangeSize, ColRangeSize, value_type> subMatrix(void) const {
   TMatrix<RowRangeSize, ColRangeSize, value_type> result;
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       result.setElement(i,j, element(i+R, j+C));
     }
   }
   return result;
 }

 template <uint32_t RowRangeSize, uint32_t ColRangeSize>
 TMatrix<RowRangeSize, ColRangeSize, value_type> subMatrix(uint32_t R, uint32_t C) const {
   TMatrix<RowRangeSize, ColRangeSize, value_type> result;
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       result.setElement(i,j, element(i+R, j+C));
     }
   }
   return result;
 }

 template <uint32_t R, uint32_t C, uint32_t RowRangeSize, uint32_t ColRangeSize>
 void setSubMatrix(const TMatrix<RowRangeSize, ColRangeSize, value_type>& m) {
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       setElement(i+R,j+C, m.element(i, j));
     }
   }
 }

 template <uint32_t RowRangeSize, uint32_t ColRangeSize>
 void setSubMatrix(const TMatrix<RowRangeSize, ColRangeSize, value_type>& m, uint32_t R, uint32_t C) {
   for (uint32_t i = 0; i < RowRangeSize; i++) {
     for (uint32_t j = 0; j < ColRangeSize; j++) {
       setElement(i+R,j+C, m.element(i, j));
     }
   }
 }


 /**
  * @brief Matrix multiplication operator.
  * @return A matrix where result is the matrix product
  * (*this) * rhs.
  */
 template <uint32_t RhsCols>
 TMatrix<Rows, RhsCols, value_type> operator*(const TMatrix<Cols, RhsCols, value_type>& rhs) const {
   TMatrix<Rows, RhsCols, value_type> result;
   for (uint32_t i = 0; i < Rows; i++) {
     value_type* resultRow = result.row(i);
     for (uint32_t j = 0; j < RhsCols; j++) {
       resultRow[j] = 0;
       for (uint32_t k = 0; k < Cols; k++) {
         resultRow[j] += element(i,k) * rhs.element(k,j);
       }
     }
   }
   return result;
 }

 /**
  * @brief Element-wise addition operator.
  * @return A matrix where result(i,j) = (*this)(i,j) + rhs(i,j).
  */
 TMatrix<Rows, Cols, value_type> operator+(const TMatrix<Rows, Cols, value_type>& rhs) const {
   TMatrix<Rows, Cols, value_type> result;
   for (uint32_t i = 0;  i < Rows*Cols;  i++) {
     result.mData[i] = mData[i] + rhs.mData[i];
   }
   return result;
 }

 /**
  * @brief Element-wise subtraction operator.
  * @return A matrix where result(i,j) = (*this)(i,j) - rhs(i,j).
  */
 TMatrix<Rows, Cols, value_type> operator-(const TMatrix<Rows, Cols, value_type>& rhs) const {
   TMatrix<Rows, Cols, value_type> result;
   for (uint32_t i = 0;  i < Rows*Cols;  i++) {
     result.mData[i] = mData[i] - rhs.mData[i];
   }
   return result;
 }

 /**
  * @brief Scalar multiplication operator.
  * @return A matrix where result(i,j) = (*this)(i,j) * s.
  */
 TMatrix<Rows, Cols, value_type> operator*(value_type s) const {
   TMatrix<Rows, Cols, value_type> result;
   for (uint32_t i = 0;  i < Rows*Cols;  i++) {
     result.mData[i] = mData[i] * s;
   }
   return result;
 }

 /**
  * @brief Scalar division operator.
  * @return A matrix where result(i,j) = (*this)(i,j) / s.
  */
 TMatrix<Rows, Cols, value_type> operator/(value_type s) const {
   TMatrix<Rows, Cols, value_type> result;
   for (uint32_t i = 0;  i < Rows*Cols;  i++) {
     result.mData[i] = mData[i] / s;
   }
   return result;
 }

 /**
  * @brief Unary negation operator.
  * @return A matrix where result(i,j) = -(*this)(i,j).
  */
 TMatrix<Rows, Cols, value_type> operator-(void) const {
   TMatrix<Rows, Cols, value_type> result;
   for (uint32_t i = 0;  i < Rows*Cols;  i++) {
     result.mData[i] = -mData[i];
   }
   return result;
 }

 /**
  * @brief Returns the inverse of this matrix. Takes U, diag, and Vt as arguments rather than recomputing the SVD.
  */
 TMatrix<Cols, Rows, double> inverse(           const TMatrix<Rows, Rows, double>&              U,
                                                const TMatrix<(Rows<Cols)?Rows:Cols,1,double>&  diag,
                                                const TMatrix<Cols, Cols, double>&              Vt)
 {

   TMatrix<Rows,Cols, double> S;
   for (uint32_t i = 0; i < Rows && i < Cols; i++) {
     S[i][i] = (fabs(diag[i]) > 1e-20) ? 1.0/diag[i] : 0;
   }

   return  (Vt.transpose() * S.transpose()) * U.transpose();

 }

 /**
  * @brief Returns the inverse of this matrix.
  */
 TMatrix<Cols, Rows, double> inverse(void) const {
   TMatrix<Rows, Rows, double> U;
   TMatrix<Cols, Cols, double> Vt;
   TMatrix<(Rows<Cols)?Rows:Cols, 1, double> diag;
   TMatrix<Rows,Cols, double> S;

   svd(&U, &diag, &Vt);

   for (uint32_t i = 0; i < Rows && i < Cols; i++) {
     S[i][i] = (fabs(diag[i]) > 1e-20) ? 1.0/diag[i] : 0;
   }

   /*std::cout << "U: ";
     U.print(std::cout); std::cout << std::endl;
     std::cout << "S: ";
     S.print(std::cout); std::cout  << std::endl;
     std::cout << "Vt: ";
     Vt.print(std::cout); std::cout << std::endl;*/

   return  (Vt.transpose() * S.transpose()) * U.transpose();

   //svd(&U, &diag, &Vt);
   //
   /*gsl_matrix thisMat, resultMat;
     double dataCopy[Rows*Cols];

     memcpy(dataCopy, mData, Rows*Cols*sizeof(value_type));
     HeliMath::gsl_matrix_from_ptr(&thisMat, Rows, Cols, dataCopy);
     HeliMath::gsl_matrix_from_ptr(&resultMat, Rows, Cols, result.mData);

     HeliMath::gsl_svd_invert(&thisMat, &resultMat);
   */
 }

 /* SVD */
 void svd(TMatrix<Rows, Rows, double>* U,
          TMatrix<(Rows<Cols)?Rows:Cols, 1, double>* diag,
          TMatrix<Cols, Cols, double>* Vt) const {
#ifndef USE_LAPACK
   if(Rows < Cols){
     std::cerr << "svd unsupported for this case on Windows (rows < cols)" << std::endl;
     exit(-1);
   }
   double** a;
   a = new double*[Rows];
   for (unsigned int i=0; i<Rows; ++i){
     a[i] = new double[Rows];
   }
   double w[Rows];
   double** v = new double*[Cols];
   for (unsigned int i=0; i<Cols; ++i)
     v[i] = new double[Cols];

   for (unsigned int i=0; i<Rows; ++i){
     for (unsigned int j=0; j<Cols; ++j){
       a[i][j] = mData[i*Cols + j];
     }
   }

   svdcmp(a, Rows, Cols, w, v);

   for (unsigned int i=0; i<Rows; ++i){
     for (unsigned int j=0; j<Rows; ++j){
       (*U)[i][j] = a[i][j];
     }
   }
   for (unsigned int i=0; i<Cols; ++i){
     (*diag)[i] = w[i];
   }
   for (unsigned int i=0; i<Cols; ++i){
     for (unsigned int j=0; j<Cols; ++j){
       (*Vt)[i][j] = v[j][i];
     }
   }

   for (unsigned int i=0; i<Rows; ++i)
     delete[] a[i];
   delete[] a;
   for (unsigned int i=0; i<Cols; ++i)
     delete[] v[i];
   delete[] v;
#else
   int dex=0;
   double fortran_a[Rows*Cols];

   for (unsigned int j=0; j < Cols; j++)
     for (unsigned int i=0; i < Rows; i++)
       fortran_a[dex++] = element(i,j);

   char jobu[] = "A";
   char jobvt[] = "A";
   int thisM = Rows;
   int thisN = Cols;
   int lda = Rows;
   double fortran_u[Rows*Rows];
   int ldu = Rows;
   double fortran_vt[Cols*Cols];
   int ldvt = Cols;
   //const int MIN_WORKSIZE = std::max(3*std::min(Rows,Cols)+
   //	std::max(Rows,Cols), 5*std::min(Rows,Cols));
   int lwork = 40*(Rows+Cols); //5*MIN_WORKSIZE;
   double work[40*(Rows+Cols)]; //work[lwork];
   int info;
   dgesvd_(jobu, jobvt, &thisM, &thisN, fortran_a, &lda, diag->row(0),
           fortran_u, &ldu, fortran_vt, &ldvt, work, &lwork, &info);

   dex=0; for (unsigned int j=0; j < Rows; j++) for (unsigned int i=0; i < Rows; i++) (*U)[i][j] = fortran_u[dex++];
   dex=0; for (unsigned int j=0; j < Cols; j++) for (unsigned int i=0; i < Cols; i++) (*Vt)[i][j] = fortran_vt[dex++];

   if (info > 0) {
     std::cout << "SVD failed." << std::endl;
   } else if (info < 0) {
     std::cout << "SVD internal error." << std::endl;
   }
#endif
 }


 /**
  * @brief Returns the matrix transpose of this matrix.
  *
  * @return A TMatrix of dimension @p Cols by @p Rows where
  * result(i,j) = (*this)(j,i) for each element.
  */
 TMatrix<Cols, Rows, value_type> transpose(void) const {
   TMatrix<Cols, Rows, value_type> result;
   for (uint32_t i = 0;  i < Rows;  i++) {
     for (uint32_t j = 0;  j < Cols;  j++) {
       result.setElement(j,i, element(i,j));
     }
   }
   return result;
 }

 /**
  * @brief Makes all of the off-diagonal elements of this matrix 0
  *
  */
 void makeDiagonal(void) {
   for (uint32_t i = 0;  i < Rows;  i++) {
     for (uint32_t j = 0;  j < Cols;  j++) {
         if(i !=j){
            setElement(i,j,0);
         }
     }
   }
 }

 /**
  * @brief Returns an "identity" matrix with dimensions given by the
  * class's template parameters.
  *
  * In the case that @p Rows != @p Cols, this matrix is simply the
  * one where the Aii elements for i < min(Rows, Cols) are 1, and all
  * other elements are 0.
  *
  * @return A TMatrix<Rows, Cols, value_type> with off-diagonal
  * elements set to 0, and diagonal elements set to 1.
  */
 static TMatrix<Rows, Cols, value_type> identity() {
   TMatrix<Rows, Cols, value_type> id;
   for (uint32_t i = 0; i < Rows && i < Cols; i++) {
     id.setElement(i,i,1);
   }
   return id;
 }

 /**
  * @brief Returns a ones matrix with dimensions given by the
  * class's template parameters.
  *
  * @return A TMatrix<Rows, Cols, value_type> with all elements set
  * to 1.
  */
 static TMatrix<Rows, Cols, value_type> one() {
   TMatrix<Rows, Cols, value_type> ones;
   for (uint32_t i = 0; i < Rows; i++) {
     for (uint32_t j = 0; j < Cols; j++) {
       ones.setElement(i,j, 1);
     }
   }
   return ones;
 }

 /**
  * @brief Returns a zero matrix with dimensions given by the
  * class's template parameters.
  *
  * @return A TMatrix<Rows, Cols, value_type> containing all 0.
  */
 static TMatrix<Rows, Cols, value_type> zero() {
   return TMatrix<Rows, Cols, value_type>();
 }


 value_type* row(uint32_t i) { return &mData[i*Cols]; }
 const value_type* row(uint32_t i) const { return &mData[i*Cols]; }

 /**
  * @brief Checks to see if any of this matrix's elements are NaN.
  */
 bool hasNaN(void) const {
   for (uint32_t i = 0; i < Rows*Cols; i++) {
     if (isnan(mData[i])) {
       return true;
     }
   }
   return false;
 }


 void print(std::ostream& os) const {
   for (uint32_t i = 0; i < Rows; i++) {
     for (uint32_t j = 0; j < Cols; j++) {
       os.width(6);
       os.precision(3);
       os << element(i,j) << " ";
     }
     os << "\n";
   }
   os.flush();
 }

 void log(std::ostream& os) const {
   for (uint32_t i = 0; i < Rows; i++) {
     for (uint32_t j = 0; j < Cols; j++) {
       os.precision(6);
       os << element(i,j) << " ";
     }
   }
 }

 void input(std::istream& is) {
   for (uint32_t i = 0; i < Rows; i++) {
     for (uint32_t j = 0; j < Cols; j++) {
       if (is.good()) {
         is >> element(i,j);
       } else {
         std::cerr << "Unable to read matrix from input stream." << std::endl;
       }
     }
   }
 }

 private:


 template <uint32_t Rows2, uint32_t Cols2, typename value_type2>
 friend TMatrix<Rows2,Cols2,value_type2> operator*(double s, const TMatrix<Rows2, Cols2, value_type2>& m);

 template <uint32_t Rows2, uint32_t Cols2, typename value_type2>
 friend std::ostream& operator<<(std::ostream& os, const TMatrix<Rows2, Cols2, value_type2>& rhs);

 template <uint32_t Rows2, uint32_t Cols2, typename value_type2>
 friend std::istream& operator>>(std::istream& is, TMatrix<Rows2, Cols2, value_type2>& rhs);
};
typedef TMatrix<1,1>  TMatrix1;
typedef TMatrix<2,2>  TMatrix2;
typedef TMatrix<3,3>  TMatrix3;
typedef TMatrix<4,4>  TMatrix4;
typedef TMatrix<1,1>  TVector1;
typedef TMatrix<2,1>  TVector2;
typedef TMatrix<3,1>  TVector3;
typedef TMatrix<4,1>  TVector4;

// left-side scalar multiply
template <uint32_t Rows, uint32_t Cols, typename value_type>
  TMatrix<Rows,Cols,value_type> operator*(double s, const TMatrix<Rows, Cols, value_type>& m) {
  return m * s;
}

// input / output operators
template <uint32_t Rows, uint32_t Cols, typename value_type>
  std::ostream& operator<<(std::ostream& os, const TMatrix<Rows, Cols, value_type>& rhs) {
  rhs.print(os);
  return os;
}

template <uint32_t Rows, uint32_t Cols, typename value_type>
  std::istream& operator>>(std::istream& is, TMatrix<Rows, Cols, value_type>& rhs) {
  rhs.input(is);
  return is;
}



#endif /* TMATRIX_H */
