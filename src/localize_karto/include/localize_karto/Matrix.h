#ifndef _MATRIX_H
#define _MATRIX_H

#include <string>
#include <fstream>
#include <limits>
#include <algorithm>
#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <math.h>
#include <float.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "Parameter.h"
#include "data_structure.h"
#include "exception.h"



namespace karto
{
/**
   * Defines a Matrix 3 x 3 class.
   */
  class Matrix3
  {
  public:
    /**
     * Default constructor
     */
    Matrix3()
    {
      Clear();
    }

    /**
     * Copy constructor
     */
    inline Matrix3(const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
    }

  public:
    /**
     * Sets this matrix to identity matrix
     */
    void SetToIdentity()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));

      for (kt_int32s i = 0; i < 3; i++)
      {
        m_Matrix[i][i] = 1.0;
      }
    }

    /**
     * Sets this matrix to zero matrix
     */
    void Clear()
    {
      memset(m_Matrix, 0, 9*sizeof(kt_double));
    }

    /**
     * Sets this matrix to be the rotation matrix of rotation around given axis
     * @param x x-coordinate of axis
     * @param y y-coordinate of axis
     * @param z z-coordinate of axis
     * @param radians amount of rotation
     */
    void FromAxisAngle(kt_double x, kt_double y, kt_double z, const kt_double radians)
    {
      kt_double cosRadians = cos(radians);//
      kt_double sinRadians = sin(radians);
      kt_double oneMinusCos = 1.0 - cosRadians;

      kt_double xx = x * x;
      kt_double yy = y * y;
      kt_double zz = z * z;

      kt_double xyMCos = x * y * oneMinusCos;
      kt_double xzMCos = x * z * oneMinusCos;
      kt_double yzMCos = y * z * oneMinusCos;

      kt_double xSin = x * sinRadians;
      kt_double ySin = y * sinRadians;
      kt_double zSin = z * sinRadians;

      m_Matrix[0][0] = xx * oneMinusCos + cosRadians;//cos(theta)
      m_Matrix[0][1] = xyMCos - zSin;//-sin(theta)
      m_Matrix[0][2] = xzMCos + ySin;//0

      m_Matrix[1][0] = xyMCos + zSin;//sin(theta)
      m_Matrix[1][1] = yy * oneMinusCos + cosRadians;//cos(theta)
      m_Matrix[1][2] = yzMCos - xSin;//0

      m_Matrix[2][0] = xzMCos - ySin;//0
      m_Matrix[2][1] = yzMCos + xSin;//0
      m_Matrix[2][2] = zz * oneMinusCos + cosRadians;//1
    }

    /**
     * Returns transposed version of this matrix
     * @return transposed matrix
     */
    Matrix3 Transpose() const
    {
      Matrix3 transpose;

      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          transpose.m_Matrix[row][col] = m_Matrix[col][row];
        }
      }

      return transpose;
    }

    /**
     * Returns the inverse of the matrix
     */
    Matrix3 Inverse() const
    {
      Matrix3 kInverse = *this;
      kt_bool haveInverse = InverseFast(kInverse, 1e-14);
      if (haveInverse == false)
      {
        assert(false);
      }
      return kInverse;
    }

    /**
     * Internal helper method for inverse matrix calculation
     * This code is lifted from the OgreMatrix3 class!!
     */
    kt_bool InverseFast(Matrix3& rkInverse, kt_double fTolerance = KT_TOLERANCE) const
    {
      // Invert a 3x3 using cofactors.  This is about 8 times faster than
      // the Numerical Recipes code which uses Gaussian elimination.
      rkInverse.m_Matrix[0][0] = m_Matrix[1][1]*m_Matrix[2][2] - m_Matrix[1][2]*m_Matrix[2][1];
      rkInverse.m_Matrix[0][1] = m_Matrix[0][2]*m_Matrix[2][1] - m_Matrix[0][1]*m_Matrix[2][2];
      rkInverse.m_Matrix[0][2] = m_Matrix[0][1]*m_Matrix[1][2] - m_Matrix[0][2]*m_Matrix[1][1];
      rkInverse.m_Matrix[1][0] = m_Matrix[1][2]*m_Matrix[2][0] - m_Matrix[1][0]*m_Matrix[2][2];
      rkInverse.m_Matrix[1][1] = m_Matrix[0][0]*m_Matrix[2][2] - m_Matrix[0][2]*m_Matrix[2][0];
      rkInverse.m_Matrix[1][2] = m_Matrix[0][2]*m_Matrix[1][0] - m_Matrix[0][0]*m_Matrix[1][2];
      rkInverse.m_Matrix[2][0] = m_Matrix[1][0]*m_Matrix[2][1] - m_Matrix[1][1]*m_Matrix[2][0];
      rkInverse.m_Matrix[2][1] = m_Matrix[0][1]*m_Matrix[2][0] - m_Matrix[0][0]*m_Matrix[2][1];
      rkInverse.m_Matrix[2][2] = m_Matrix[0][0]*m_Matrix[1][1] - m_Matrix[0][1]*m_Matrix[1][0];

      kt_double fDet = m_Matrix[0][0]*rkInverse.m_Matrix[0][0] +
                       m_Matrix[0][1]*rkInverse.m_Matrix[1][0] +
                       m_Matrix[0][2]*rkInverse.m_Matrix[2][0];

      if (fabs(fDet) <= fTolerance)
      {
        return false;
      }

      kt_double fInvDet = 1.0/fDet;
      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          rkInverse.m_Matrix[row][col] *= fInvDet;
        }
      }

      return true;
    }

    /**
     * Returns a string representation of this matrix
     * @return string representation of this matrix
     */
    inline std::string ToString() const
    {
      std::stringstream converter;
      converter.precision(std::numeric_limits<double>::digits10);

      for (int row = 0; row < 3; row++)
      {
        for (int col = 0; col < 3; col++)
        {
          converter << m_Matrix[row][col] << " ";
        }
      }

      return converter.str();
    }

  public:
    /**
     * Assignment operator
     */
    inline Matrix3& operator = (const Matrix3& rOther)
    {
      memcpy(m_Matrix, rOther.m_Matrix, 9*sizeof(kt_double));
      return *this;
    }

    /**
     * Matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return reference to mat(r,c)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      return m_Matrix[row][column];
    }

    /**
     * Read-only matrix element access, allows use of construct mat(r, c)
     * @param row
     * @param column
     * @return mat(r,c)
     */
    inline kt_double operator()(kt_int32u row, kt_int32u column) const
    {
      return m_Matrix[row][column];
    }

    /**
     * Binary Matrix3 multiplication.
     * @param rOther
     * @return Matrix3 product
     */
    Matrix3 operator * (const Matrix3& rOther) const
    {
      Matrix3 product;

      for (size_t row = 0; row < 3; row++)
      {
        for (size_t col = 0; col < 3; col++)
        {
          product.m_Matrix[row][col] = m_Matrix[row][0]*rOther.m_Matrix[0][col] +
                                       m_Matrix[row][1]*rOther.m_Matrix[1][col] +
                                       m_Matrix[row][2]*rOther.m_Matrix[2][col];
        }
      }

      return product;
    }

    /**
     * Matrix3 and Pose2 multiplication - matrix * pose [3x3 * 3x1 = 3x1]
     * @param rPose2
     * @return Pose2 product
     */
    inline Pose2 operator * (const Pose2& rPose2) const
    {
      Pose2 pose2;

      pose2.SetX(m_Matrix[0][0] * rPose2.GetX() + m_Matrix[0][1] *
                 rPose2.GetY() + m_Matrix[0][2] * rPose2.GetHeading());
      pose2.SetY(m_Matrix[1][0] * rPose2.GetX() + m_Matrix[1][1] *
                 rPose2.GetY() + m_Matrix[1][2] * rPose2.GetHeading());
      pose2.SetHeading(m_Matrix[2][0] * rPose2.GetX() + m_Matrix[2][1] *
                       rPose2.GetY() + m_Matrix[2][2] * rPose2.GetHeading());

      return pose2;
    }

    /**
     * In place Matrix3 add.
     * @param rkMatrix
     */
    inline void operator += (const Matrix3& rkMatrix)
    {
      for (kt_int32u row = 0; row < 3; row++)
      {
        for (kt_int32u col = 0; col < 3; col++)
        {
          m_Matrix[row][col] += rkMatrix.m_Matrix[row][col];
        }
      }
    }

    /**
     * Write Matrix3 onto output stream
     * @param rStream output stream
     * @param rMatrix to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Matrix3& rMatrix)
    {
      rStream << rMatrix.ToString();
      return rStream;
    }

  private:
    kt_double m_Matrix[3][3];
  };  // Matrix3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines a general Matrix class.
   */
  class Matrix
  {
  public:
    /**
     * Constructs a matrix of size rows x columns
     */
    Matrix(kt_int32u rows, kt_int32u columns)
      : m_Rows(rows)
      , m_Columns(columns)
      , m_pData(NULL)
    {
      Allocate();

      Clear();
    }

    /**
     * Destructor
     */
    virtual ~Matrix()
    {
      delete [] m_pData;
    }

  public:
    /**
     * Set all entries to 0
     */
    void Clear()
    {
      if (m_pData != NULL)
      {
        memset(m_pData, 0, sizeof(kt_double) * m_Rows * m_Columns);
      }
    }

    /**
     * Gets the number of rows of the matrix
     * @return nubmer of rows
     */
    inline kt_int32u GetRows() const
    {
      return m_Rows;
    }

    /**
     * Gets the number of columns of the matrix
     * @return nubmer of columns
     */
    inline kt_int32u GetColumns() const
    {
      return m_Columns;
    }

    /**
     * Returns a reference to the entry at (row,column)
     * @param row
     * @param column
     * @return reference to entry at (row,column)
     */
    inline kt_double& operator()(kt_int32u row, kt_int32u column)
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

    /**
     * Returns a const reference to the entry at (row,column)
     * @param row
     * @param column
     * @return const reference to entry at (row,column)
     */
    inline const kt_double& operator()(kt_int32u row, kt_int32u column) const
    {
      RangeCheck(row, column);

      return m_pData[row + column * m_Rows];
    }

  private:
    /**
     * Allocate space for the matrix
     */
    void Allocate()
    {
      try
      {
        if (m_pData != NULL)
        {
          delete[] m_pData;
        }

        m_pData = new kt_double[m_Rows * m_Columns];
      }
      catch (const std::bad_alloc& ex)
      {
        throw Exception("Matrix allocation error");
      }

      if (m_pData == NULL)
      {
        throw Exception("Matrix allocation error");
      }
    }

    /**
     * Checks if (row,column) is a valid entry into the matrix
     * @param row
     * @param column
     */
    inline void RangeCheck(kt_int32u row, kt_int32u column) const
    {
      if (math::IsUpTo(row, m_Rows) == false)
      {
        throw Exception("Matrix - RangeCheck ERROR!!!!");
      }

      if (math::IsUpTo(column, m_Columns) == false)
      {
        throw Exception("Matrix - RangeCheck ERROR!!!!");
      }
    }

  private:
    kt_int32u m_Rows;
    kt_int32u m_Columns;

    kt_double* m_pData;
  };  // Matrix
}//namespace karto

#endif