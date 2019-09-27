#ifndef _DATA_STRUCTURE_H
#define _DATA_STRUCTURE_H


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

#include "Math.h"


namespace karto
{
	/**
   * Represents a size (width, height) in 2-dimensional real space.
   */
  template<typename T>
  class Size2
  {
  public:
    /**
     * Default constructor
     */
    Size2()
      : m_Width(0)
      , m_Height(0)
    {
    }

    /**
     * Constructor initializing point location
     * @param width
     * @param height
     */
    Size2(T width, T height)
      : m_Width(width)
      , m_Height(height)
    {
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Size2(const Size2& rOther)
      : m_Width(rOther.m_Width)
      , m_Height(rOther.m_Height)
    {
    }

  public:
    /**
     * Gets the width
     * @return the width
     */
    inline const T GetWidth() const
    {
      return m_Width;
    }

    /**
     * Sets the width
     * @param width
     */
    inline void SetWidth(T width)
    {
      m_Width = width;
    }

    /**
     * Gets the height
     * @return the height
     */
    inline const T GetHeight() const
    {
      return m_Height;
    }

    /**
     * Sets the height
     * @param height
     */
    inline void SetHeight(T height)
    {
      m_Height = height;
    }

    /**
     * Assignment operator
     */
    inline Size2& operator = (const Size2& rOther)
    {
      m_Width = rOther.m_Width;
      m_Height = rOther.m_Height;

      return(*this);
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Size2& rOther) const
    {
      return (m_Width == rOther.m_Width && m_Height == rOther.m_Height);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Size2& rOther) const
    {
      return (m_Width != rOther.m_Width || m_Height != rOther.m_Height);
    }

    /**
     * Write Size2 onto output stream
     * @param rStream output stream
     * @param rSize to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Size2& rSize)
    {
      rStream << "(" << rSize.m_Width << ", " << rSize.m_Height << ")";
      return rStream;
    }

  private:
    T m_Width;
    T m_Height;
  };  // Size2<T>

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a vector (x, y) in 2-dimensional real space.
   */
  template<typename T>
  class Vector2
  {
  public:
    /**
     * Default constructor
     */
    Vector2()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
    }

    /**
     * Constructor initializing vector location
     * @param x
     * @param y
     */
    Vector2(T x, T y)
    {
      m_Values[0] = x;
      m_Values[1] = y;
    }

  public:
    /**
     * Gets the x-coordinate of this vector2
     * @return the x-coordinate of the vector2
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the x-coordinate of this vector2
     * @param x the x-coordinate of the vector2
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }

    /**
     * Gets the y-coordinate of this vector2
     * @return the y-coordinate of the vector2
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the y-coordinate of this vector2
     * @param y the y-coordinate of the vector2
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }

    /**
     * Floor point operator
     * @param rOther
     */
    inline void MakeFloor(const Vector2& rOther)
    {
      if ( rOther.m_Values[0] < m_Values[0] ) m_Values[0] = rOther.m_Values[0];
      if ( rOther.m_Values[1] < m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
     * Ceiling point operator
     * @param rOther
     */
    inline void MakeCeil(const Vector2& rOther)
    {
      if ( rOther.m_Values[0] > m_Values[0] ) m_Values[0] = rOther.m_Values[0];
      if ( rOther.m_Values[1] > m_Values[1] ) m_Values[1] = rOther.m_Values[1];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]);
    }

    /**
     * Returns the length of the vector (x and y).
     * @return length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns the square distance to the given vector
     * @returns square distance to the given vector
     */
    inline kt_double SquaredDistance(const Vector2& rOther) const
    {
      return (*this - rOther).SquaredLength();
    }

    /**
     * Gets the distance to the other vector2
     * @param rOther
     * @return distance to other vector2
     */
    inline kt_double Distance(const Vector2& rOther) const
    {
      return sqrt(SquaredDistance(rOther));
    }

  public:
    /**
     * In place Vector2 addition.
     */
    inline void operator += (const Vector2& rOther)
    {
      m_Values[0] += rOther.m_Values[0];
      m_Values[1] += rOther.m_Values[1];
    }

    /**
     * In place Vector2 subtraction.
     */
    inline void operator -= (const Vector2& rOther)
    {
      m_Values[0] -= rOther.m_Values[0];
      m_Values[1] -= rOther.m_Values[1];
    }

    /**
     * Addition operator
     * @param rOther
     * @return vector resulting from adding this vector with the given vector
     */
    inline const Vector2 operator + (const Vector2& rOther) const
    {
      return Vector2(m_Values[0] + rOther.m_Values[0], m_Values[1] + rOther.m_Values[1]);
    }

    /**
     * Subtraction operator
     * @param rOther
     * @return vector resulting from subtracting this vector from the given vector
     */
    inline const Vector2 operator - (const Vector2& rOther) const
    {
      return Vector2(m_Values[0] - rOther.m_Values[0], m_Values[1] - rOther.m_Values[1]);
    }

    /**
     * In place scalar division operator
     * @param scalar
     */
    inline void operator /= (T scalar)
    {
      m_Values[0] /= scalar;
      m_Values[1] /= scalar;
    }

    /**
     * Divides a Vector2
     * @param scalar
     * @return scalar product
     */
    inline const Vector2 operator / (T scalar) const
    {
      return Vector2(m_Values[0] / scalar, m_Values[1] / scalar);
    }

    /**
     * Computes the dot product between the two vectors
     * @param rOther
     * @return dot product
     */
    inline kt_double operator * (const Vector2& rOther) const
    {
      return m_Values[0] * rOther.m_Values[0] + m_Values[1] * rOther.m_Values[1];
    }

    /**
     * Scales the vector by the given scalar
     * @param scalar
     */
    inline const Vector2 operator * (T scalar) const
    {
      return Vector2(m_Values[0] * scalar, m_Values[1] * scalar);
    }

    /**
     * Subtract the vector by the given scalar
     * @param scalar
     */
    inline const Vector2 operator - (T scalar) const
    {
      return Vector2(m_Values[0] - scalar, m_Values[1] - scalar);
    }

    /**
     * In place scalar multiplication operator
     * @param scalar
     */
    inline void operator *= (T scalar)
    {
      m_Values[0] *= scalar;
      m_Values[1] *= scalar;
    }

    /**
     * Equality operator returns true if the corresponding x, y values of each Vector2 are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Vector2& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] && m_Values[1] == rOther.m_Values[1]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y values of each Vector2 not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Vector2& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] || m_Values[1] != rOther.m_Values[1]);
    }

    /**
     * Less than operator
     * @param rOther
     * @return true if left vector is less than right vector
     */
    inline kt_bool operator < (const Vector2& rOther) const
    {
      if (m_Values[0] < rOther.m_Values[0])
        return true;
      else if (m_Values[0] > rOther.m_Values[0])
        return false;
      else
        return (m_Values[1] < rOther.m_Values[1]);
    }

    /**
     * Write Vector2 onto output stream
     * @param rStream output stream
     * @param rVector to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector2& rVector)
    {
      rStream << rVector.GetX() << " " << rVector.GetY();
      return rStream;
    }

    /**
     * Read Vector2 from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Vector2& /*rVector*/)
    {
      // Implement me!!  TODO(lucbettaieb): What the what?  Do I need to implement this?
      return rStream;
    }

  private:
    T m_Values[2];
  };  // Vector2<T>

  /**
   * Type declaration of Vector2<kt_double> vector
   */
  typedef std::vector< Vector2<kt_double> > PointVectorDouble;

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Represents a vector (x, y, z) in 3-dimensional real space.
   */
  template<typename T>
  class Vector3
  {
  public:
    /**
     * Default constructor
     */
    Vector3()
    {
      m_Values[0] = 0;
      m_Values[1] = 0;
      m_Values[2] = 0;
    }

    /**
     * Constructor initializing point location
     * @param x
     * @param y
     * @param z
     */
    Vector3(T x, T y, T z)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
    }

    /**
     * Copy constructor
     * @param rOther
     */
    Vector3(const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];
    }

  public:
    /**
     * Gets the x-component of this vector
     * @return x-component
     */
    inline const T& GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the x-component of this vector
     * @param x
     */
    inline void SetX(const T& x)
    {
      m_Values[0] = x;
    }

    /**
     * Gets the y-component of this vector
     * @return y-component
     */
    inline const T& GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the y-component of this vector
     * @param y
     */
    inline void SetY(const T& y)
    {
      m_Values[1] = y;
    }

    /**
     * Gets the z-component of this vector
     * @return z-component
     */
    inline const T& GetZ() const
    {
      return m_Values[2];
    }

    /**
     * Sets the z-component of this vector
     * @param z
     */
    inline void SetZ(const T& z)
    {
      m_Values[2] = z;
    }

    /**
     * Floor vector operator
     * @param rOther Vector3d
     */
    inline void MakeFloor(const Vector3& rOther)
    {
      if (rOther.m_Values[0] < m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] < m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] < m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Ceiling vector operator
     * @param rOther Vector3d
     */
    inline void MakeCeil(const Vector3& rOther)
    {
      if (rOther.m_Values[0] > m_Values[0]) m_Values[0] = rOther.m_Values[0];
      if (rOther.m_Values[1] > m_Values[1]) m_Values[1] = rOther.m_Values[1];
      if (rOther.m_Values[2] > m_Values[2]) m_Values[2] = rOther.m_Values[2];
    }

    /**
     * Returns the square of the length of the vector
     * @return square of the length of the vector
     */
    inline kt_double SquaredLength() const
    {
      return math::Square(m_Values[0]) + math::Square(m_Values[1]) + math::Square(m_Values[2]);
    }

    /**
     * Returns the length of the vector.
     * @return Length of the vector
     */
    inline kt_double Length() const
    {
      return sqrt(SquaredLength());
    }

    /**
     * Returns a string representation of this vector
     * @return string representation of this vector
     */
    inline std::string ToString() const
    {
      std::stringstream converter;
      converter.precision(std::numeric_limits<double>::digits10);

      converter << GetX() << " " << GetY() << " " << GetZ();

      return converter.str();
    }

  public:
    /**
     * Assignment operator
     */
    inline Vector3& operator = (const Vector3& rOther)
    {
      m_Values[0] = rOther.m_Values[0];
      m_Values[1] = rOther.m_Values[1];
      m_Values[2] = rOther.m_Values[2];

      return *this;
    }

    /**
     * Binary vector add.
     * @param rOther
     * @return vector sum
     */
    inline const Vector3 operator + (const Vector3& rOther) const
    {
      return Vector3(m_Values[0] + rOther.m_Values[0],
                     m_Values[1] + rOther.m_Values[1],
                     m_Values[2] + rOther.m_Values[2]);
    }

    /**
     * Binary vector add.
     * @param scalar
     * @return sum
     */
    inline const Vector3 operator + (kt_double scalar) const
    {
      return Vector3(m_Values[0] + scalar,
                     m_Values[1] + scalar,
                     m_Values[2] + scalar);
    }

    /**
     * Binary vector subtract.
     * @param rOther
     * @return vector difference
     */
    inline const Vector3 operator - (const Vector3& rOther) const
    {
      return Vector3(m_Values[0] - rOther.m_Values[0],
                     m_Values[1] - rOther.m_Values[1],
                     m_Values[2] - rOther.m_Values[2]);
    }

    /**
     * Binary vector subtract.
     * @param scalar
     * @return difference
     */
    inline const Vector3 operator - (kt_double scalar) const
    {
      return Vector3(m_Values[0] - scalar, m_Values[1] - scalar, m_Values[2] - scalar);
    }

    /**
     * Scales the vector by the given scalar
     * @param scalar
     */
    inline const Vector3 operator * (T scalar) const
    {
      return Vector3(m_Values[0] * scalar, m_Values[1] * scalar, m_Values[2] * scalar);
    }

    /**
     * Equality operator returns true if the corresponding x, y, z values of each Vector3 are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Vector3& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] &&
              m_Values[1] == rOther.m_Values[1] &&
              m_Values[2] == rOther.m_Values[2]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y, z values of each Vector3 not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Vector3& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] ||
              m_Values[1] != rOther.m_Values[1] ||
              m_Values[2] != rOther.m_Values[2]);
    }

    /**
     * Write Vector3 onto output stream
     * @param rStream output stream
     * @param rVector to write
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Vector3& rVector)
    {
      rStream << rVector.ToString();
      return rStream;
    }

    /**
     * Read Vector3 from input stream
     * @param rStream input stream
     */
    friend inline std::istream& operator >> (std::istream& rStream, const Vector3& /*rVector*/)
    {
      // Implement me!!
      return rStream;
    }

  private:
    T m_Values[3];
  };  // Vector3

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Defines an orientation as a quaternion rotation using the positive Z axis as the zero reference.
   * <BR>
   * Q = w + ix + jy + kz <BR>
   * w = c_1 * c_2 * c_3 - s_1 * s_2 * s_3 <BR>
   * x = s_1 * s_2 * c_3 + c_1 * c_2 * s_3 <BR>
   * y = s_1 * c_2 * c_3 + c_1 * s_2 * s_3 <BR>
   * z = c_1 * s_2 * c_3 - s_1 * c_2 * s_3 <BR>
   * where <BR>
   * c_1 = cos(theta/2) <BR>
   * c_2 = cos(phi/2) <BR>
   * c_3 = cos(psi/2) <BR>
   * s_1 = sin(theta/2) <BR>
   * s_2 = sin(phi/2) <BR>
   * s_3 = sin(psi/2) <BR>
   * and <BR>
   * theta is the angle of rotation about the Y axis measured from the Z axis. <BR>
   * phi is the angle of rotation about the Z axis measured from the X axis. <BR>
   * psi is the angle of rotation about the X axis measured from the Y axis. <BR>
   * (All angles are right-handed.)
   */
  class Quaternion
  {
  public:
    /**
     * Create a quaternion with default (x=0, y=0, z=0, w=1) values
     */
    inline Quaternion()
    {
      m_Values[0] = 0.0;
      m_Values[1] = 0.0;
      m_Values[2] = 0.0;
      m_Values[3] = 1.0;
    }

    /**
     * Create a quaternion using x, y, z, w values.
     * @param x
     * @param y
     * @param z
     * @param w
     */
    inline Quaternion(kt_double x, kt_double y, kt_double z, kt_double w)
    {
      m_Values[0] = x;
      m_Values[1] = y;
      m_Values[2] = z;
      m_Values[3] = w;
    }

    /**
     * Copy constructor
     */
    inline Quaternion(const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];
    }

  public:
    /**
     * Returns the X-value
     * @return Return the X-value of the quaternion
     */
    inline kt_double GetX() const
    {
      return m_Values[0];
    }

    /**
     * Sets the X-value
     * @param x X-value of the quaternion
     */
    inline void SetX(kt_double x)
    {
      m_Values[0] = x;
    }

    /**
     * Returns the Y-value
     * @return Return the Y-value of the quaternion
     */
    inline kt_double GetY() const
    {
      return m_Values[1];
    }

    /**
     * Sets the Y-value
     * @param y Y-value of the quaternion
     */
    inline void SetY(kt_double y)
    {
      m_Values[1] = y;
    }

    /**
     * Returns the Z-value
     * @return Return the Z-value of the quaternion
     */
    inline kt_double GetZ() const
    {
      return m_Values[2];
    }

    /**
     * Sets the Z-value
     * @param z Z-value of the quaternion
     */
    inline void SetZ(kt_double z)
    {
      m_Values[2] = z;
    }

    /**
     * Returns the W-value
     * @return Return the W-value of the quaternion
     */
    inline kt_double GetW() const
    {
      return m_Values[3];
    }

    /**
     * Sets the W-value
     * @param w W-value of the quaternion
     */
    inline void SetW(kt_double w)
    {
      m_Values[3] = w;
    }

    /**
     * Converts this quaternion into Euler angles
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
     * @param rYaw
     * @param rPitch
     * @param rRoll
     */
    void ToEulerAngles(kt_double& rYaw, kt_double& rPitch, kt_double& rRoll) const
    {
      kt_double test = m_Values[0] * m_Values[1] + m_Values[2] * m_Values[3];

      if (test > 0.499)
      {
        // singularity at north pole
        rYaw = 2 * atan2(m_Values[0], m_Values[3]);;
        rPitch = KT_PI_2;
        rRoll = 0;
      }
      else if (test < -0.499)
      {
        // singularity at south pole
        rYaw = -2 * atan2(m_Values[0], m_Values[3]);
        rPitch = -KT_PI_2;
        rRoll = 0;
      }
      else
      {
        kt_double sqx = m_Values[0] * m_Values[0];
        kt_double sqy = m_Values[1] * m_Values[1];
        kt_double sqz = m_Values[2] * m_Values[2];

        rYaw = atan2(2 * m_Values[1] * m_Values[3] - 2 * m_Values[0] * m_Values[2], 1 - 2 * sqy - 2 * sqz);
        rPitch = asin(2 * test);
        rRoll = atan2(2 * m_Values[0] * m_Values[3] - 2 * m_Values[1] * m_Values[2], 1 - 2 * sqx - 2 * sqz);
      }
    }

    /**
     * Set x,y,z,w values of the quaternion based on Euler angles.
     * Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
     * @param yaw
     * @param pitch
     * @param roll
     */
    void FromEulerAngles(kt_double yaw, kt_double pitch, kt_double roll)
    {
      kt_double angle;

      angle = yaw * 0.5;
      kt_double cYaw = cos(angle);
      kt_double sYaw = sin(angle);

      angle = pitch * 0.5;
      kt_double cPitch = cos(angle);
      kt_double sPitch = sin(angle);

      angle = roll * 0.5;
      kt_double cRoll = cos(angle);
      kt_double sRoll = sin(angle);

      m_Values[0] = sYaw * sPitch * cRoll + cYaw * cPitch * sRoll;
      m_Values[1] = sYaw * cPitch * cRoll + cYaw * sPitch * sRoll;
      m_Values[2] = cYaw * sPitch * cRoll - sYaw * cPitch * sRoll;
      m_Values[3] = cYaw * cPitch * cRoll - sYaw * sPitch * sRoll;
    }

    /**
     * Assignment operator
     * @param rQuaternion
     */
    inline Quaternion& operator = (const Quaternion& rQuaternion)
    {
      m_Values[0] = rQuaternion.m_Values[0];
      m_Values[1] = rQuaternion.m_Values[1];
      m_Values[2] = rQuaternion.m_Values[2];
      m_Values[3] = rQuaternion.m_Values[3];

      return(*this);
    }

    /**
     * Equality operator returns true if the corresponding x, y, z, w values of each quaternion are the same values.
     * @param rOther
     */
    inline kt_bool operator == (const Quaternion& rOther) const
    {
      return (m_Values[0] == rOther.m_Values[0] &&
              m_Values[1] == rOther.m_Values[1] &&
              m_Values[2] == rOther.m_Values[2] &&
              m_Values[3] == rOther.m_Values[3]);
    }

    /**
     * Inequality operator returns true if any of the corresponding x, y, z, w values of each quaternion not the same.
     * @param rOther
     */
    inline kt_bool operator != (const Quaternion& rOther) const
    {
      return (m_Values[0] != rOther.m_Values[0] ||
              m_Values[1] != rOther.m_Values[1] ||
              m_Values[2] != rOther.m_Values[2] ||
              m_Values[3] != rOther.m_Values[3]);
    }

    /**
     * Write this quaternion onto output stream
     * @param rStream output stream
     * @param rQuaternion
     */
    friend inline std::ostream& operator << (std::ostream& rStream, const Quaternion& rQuaternion)
    {
      rStream << rQuaternion.m_Values[0] << " "
              << rQuaternion.m_Values[1] << " "
              << rQuaternion.m_Values[2] << " "
              << rQuaternion.m_Values[3];
      return rStream;
    }

  private:
    kt_double m_Values[4];
  };//quaternion

  /**
   * Defines a bounding box in 2-dimensional real space.
   */
  class BoundingBox2
  {
  public:
    /*
     * Default constructor
     */
    BoundingBox2()
      : m_Minimum(999999999999999999.99999, 999999999999999999.99999)
      , m_Maximum(-999999999999999999.99999, -999999999999999999.99999)
    {
    }

  public:
    /**
     * Get bounding box minimum
     */
    inline const Vector2<kt_double>& GetMinimum() const
    {
      return m_Minimum;
    }

    /**
     * Set bounding box minimum
     */
    inline void SetMinimum(const Vector2<kt_double>& mMinimum)
    {
      m_Minimum = mMinimum;
    }

    /**
     * Get bounding box maximum
     */
    inline const Vector2<kt_double>& GetMaximum() const
    {
      return m_Maximum;
    }

    /**
     * Set bounding box maximum
     */
    inline void SetMaximum(const Vector2<kt_double>& rMaximum)
    {
      m_Maximum = rMaximum;
    }

    /**
     * Get the size of the bounding box
     */
    inline Size2<kt_double> GetSize() const
    {
      Vector2<kt_double> size = m_Maximum - m_Minimum;

      return Size2<kt_double>(size.GetX(), size.GetY());
    }

    /**
     * Add vector to bounding box
     */
    inline void Add(const Vector2<kt_double>& rPoint)
    {
      m_Minimum.MakeFloor(rPoint);
      m_Maximum.MakeCeil(rPoint);
    }

    /**
     * Add other bounding box to bounding box
     */
    inline void Add(const BoundingBox2& rBoundingBox)
    {
      Add(rBoundingBox.GetMinimum());
      Add(rBoundingBox.GetMaximum());
    }

    /**
     * Whether the given point is in the bounds of this box
     * @param rPoint
     * @return in bounds?
     */
    inline kt_bool IsInBounds(const Vector2<kt_double>& rPoint) const
    {
      return (math::InRange(rPoint.GetX(), m_Minimum.GetX(), m_Maximum.GetX()) &&
              math::InRange(rPoint.GetY(), m_Minimum.GetY(), m_Maximum.GetY()));
    }

  private:
    Vector2<kt_double> m_Minimum;
    Vector2<kt_double> m_Maximum;
  };  // BoundingBox2

  template<typename T>
  class Rectangle2//矩形左下角坐标（x, y）和宽度（width）, 高度（height）构成
  {
  public:
    /**
     * Default constructor
     */
    Rectangle2()
    {
    }

    /**
     * Constructor initializing rectangle parameters
     * @param x x-coordinate of left edge of rectangle
     * @param y y-coordinate of bottom edge of rectangle
     * @param width width of rectangle
     * @param height height of rectangle
     */
    Rectangle2(T x, T y, T width, T height)
      : m_Position(x, y)
      , m_Size(width, height)
    {
    }

    /**
     * Constructor initializing rectangle parameters
     * @param rPosition (x,y)-coordinate of rectangle
     * @param rSize Size of the rectangle
     */
    Rectangle2(const Vector2<T>& rPosition, const Size2<T>& rSize)
      : m_Position(rPosition)
      , m_Size(rSize)
    {
    }

    /**
     * Copy constructor
     */
    Rectangle2(const Rectangle2& rOther)
      : m_Position(rOther.m_Position)
      , m_Size(rOther.m_Size)
    {
    }

  public:
    /**
     * Gets the x-coordinate of the left edge of this rectangle
     * @return the x-coordinate of the left edge of this rectangle
     */
    inline T GetX() const
    {
      return m_Position.GetX();
    }

    /**
     * Sets the x-coordinate of the left edge of this rectangle
     * @param x the x-coordinate of the left edge of this rectangle
     */
    inline void SetX(T x)
    {
      m_Position.SetX(x);
    }

    /**
     * Gets the y-coordinate of the bottom edge of this rectangle
     * @return the y-coordinate of the bottom edge of this rectangle
     */
    inline T GetY() const
    {
      return m_Position.GetY();
    }

    /**
     * Sets the y-coordinate of the bottom edge of this rectangle
     * @param y the y-coordinate of the bottom edge of this rectangle
     */
    inline void SetY(T y)
    {
      m_Position.SetY(y);
    }

    /**
     * Gets the width of this rectangle
     * @return the width of this rectangle
     */
    inline T GetWidth() const
    {
      return m_Size.GetWidth();
    }

    /**
     * Sets the width of this rectangle
     * @param width the width of this rectangle
     */
    inline void SetWidth(T width)
    {
      m_Size.SetWidth(width);
    }

    /**
     * Gets the height of this rectangle
     * @return the height of this rectangle
     */
    inline T GetHeight() const
    {
      return m_Size.GetHeight();
    }

    /**
     * Sets the height of this rectangle
     * @param height the height of this rectangle
     */
    inline void SetHeight(T height)
    {
      m_Size.SetHeight(height);
    }

    /**
     * Gets the position of this rectangle
     * @return the position of this rectangle
     */
    inline const Vector2<T>& GetPosition() const
    {
      return m_Position;
    }

    /**
     * Sets the position of this rectangle
     * @param rX x
     * @param rY y
     */
    inline void SetPosition(const T& rX, const T& rY)
    {
      m_Position = Vector2<T>(rX, rY);
    }

    /**
     * Sets the position of this rectangle
     * @param rPosition position
     */
    inline void SetPosition(const Vector2<T>& rPosition)
    {
      m_Position = rPosition;
    }

    /**
     * Gets the size of this rectangle
     * @return the size of this rectangle
     */
    inline const Size2<T>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets the size of this rectangle
     * @param rSize size
     */
    inline void SetSize(const Size2<T>& rSize)
    {
      m_Size = rSize;
    }

    /**
     * Gets the center of this rectangle
     * @return the center of this rectangle
     */
    inline const Vector2<T> GetCenter() const
    {
      return Vector2<T>(m_Position.GetX() + m_Size.GetWidth() * 0.5, m_Position.GetY() + m_Size.GetHeight() * 0.5);
    }

  public:
    /**
     * Assignment operator
     */
    Rectangle2& operator = (const Rectangle2& rOther)
    {
      m_Position = rOther.m_Position;
      m_Size = rOther.m_Size;

      return *this;
    }

    /**
     * Equality operator
     */
    inline kt_bool operator == (const Rectangle2& rOther) const
    {
      return (m_Position == rOther.m_Position && m_Size == rOther.m_Size);
    }

    /**
     * Inequality operator
     */
    inline kt_bool operator != (const Rectangle2& rOther) const
    {
      return (m_Position != rOther.m_Position || m_Size != rOther.m_Size);
    }

  private:
    Vector2<T> m_Position;
    Size2<T> m_Size;
  };  // Rectangle2
}//namespace karto


#endif