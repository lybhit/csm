#ifndef TRANSFORM_H
#define TRANSFORM_H

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

#include "data_structure.h"
#include "Pose.h"
#include "Matrix.h"

namespace karto
{
/**
   * Implementation of a Pose2 transform
   */
  class Transform
  {
  public:
    /**
     * Constructs a transformation from the origin to the given pose
     * @param rPose pose
     */
    Transform(const Pose2& rPose)
    {
      SetTransform(Pose2(), rPose);
    }

    /**
     * Constructs a transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    Transform(const Pose2& rPose1, const Pose2& rPose2)
    {
      SetTransform(rPose1, rPose2);
    }

  public:
    /**
     * Transforms the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 TransformPose(const Pose2& rSourcePose)
    {
      Pose2 newPosition = m_Transform + m_Rotation * rSourcePose;
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() + m_Transform.GetHeading());

      return Pose2(newPosition.GetPosition(), angle);
    }

    /**
     * Inverse transformation of the pose according to this transform
     * @param rSourcePose pose to transform from
     * @return transformed pose
     */
    inline Pose2 InverseTransformPose(const Pose2& rSourcePose)
    {
      Pose2 newPosition = m_InverseRotation * (rSourcePose - m_Transform);
      kt_double angle = math::NormalizeAngle(rSourcePose.GetHeading() - m_Transform.GetHeading());

      // components of transform
      return Pose2(newPosition.GetPosition(), angle);
    }

  private:
    /**
     * Sets this to be the transformation from the first pose to the second pose
     * @param rPose1 first pose
     * @param rPose2 second pose
     */
    void SetTransform(const Pose2& rPose1, const Pose2& rPose2)
    {
      if (rPose1 == rPose2)
      {
        m_Rotation.SetToIdentity();
        m_InverseRotation.SetToIdentity();
        m_Transform = Pose2();
        return;
      }

      // heading transformation
      m_Rotation.FromAxisAngle(0, 0, 1, rPose2.GetHeading() - rPose1.GetHeading());
      m_InverseRotation.FromAxisAngle(0, 0, 1, rPose1.GetHeading() - rPose2.GetHeading());

      // position transformation
      Pose2 newPosition;
      if (rPose1.GetX() != 0.0 || rPose1.GetY() != 0.0)
      {
        newPosition = rPose2 - m_Rotation * rPose1;
      }
      else
      {
        newPosition = rPose2;
      }

      m_Transform = Pose2(newPosition.GetPosition(), rPose2.GetHeading() - rPose1.GetHeading());
    }

  private:
    // pose transformation
    Pose2 m_Transform;

    Matrix3 m_Rotation;
    Matrix3 m_InverseRotation;
  };  // Transform
}//namespace karto

#endif