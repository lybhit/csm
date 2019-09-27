#ifndef _LASER_SCAN_H
#define _LASER_SCAN_H

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
#include "Transform.h"
#include "laser_range_finder.h"


namespace karto
{
  /**
   * Type declaration of range readings vector
   */
  typedef std::vector<kt_double> RangeReadingsVector;

 /**
   * LaserRangeScan representing the range readings from a laser range finder sensor.
   */
  class LaserRangeScan
  {

  public:
    /**
     * Constructs a scan from the given sensor with the given readings
     * @param rSensorName
     */
    LaserRangeScan()
      : m_pRangeReadings(NULL)
      , m_NumberOfRangeReadings(0)
    {
    }

    /**
     * Constructs a scan from the given sensor with the given readings
     * @param rSensorName
     * @param rRangeReadings
     */
    LaserRangeScan(const RangeReadingsVector& rRangeReadings)
      : m_pRangeReadings(NULL)
      , m_NumberOfRangeReadings(0)
    {

      SetRangeReadings(rRangeReadings);
    }

    /**
     * Destructor
     */
    virtual ~LaserRangeScan()
    {
      delete [] m_pRangeReadings;
    }

  public:
    /**
     * Gets the range readings of this scan
     * @return range readings of this scan
     */
    inline kt_double* GetRangeReadings() const
    {
      return m_pRangeReadings;
    }

    inline RangeReadingsVector GetRangeReadingsVector() const
    {
      return RangeReadingsVector(m_pRangeReadings, m_pRangeReadings + m_NumberOfRangeReadings);
    }

    /**
     * Sets the range readings for this scan
     * @param rRangeReadings
     */
    inline void SetRangeReadings(const RangeReadingsVector& rRangeReadings)
    {
      // ignore for now! XXXMAE BUGUBUG 05/21/2010 << TODO(lucbettaieb): What the heck is this??
      // if (rRangeReadings.size() != GetNumberOfRangeReadings())
      // {
      //   std::stringstream error;
      //   error << "Given number of readings (" << rRangeReadings.size()
      //         << ") does not match expected number of range finder (" << GetNumberOfRangeReadings() << ")";
      //   throw Exception(error.str());
      // }

      if (!rRangeReadings.empty())
      {
        if (rRangeReadings.size() != m_NumberOfRangeReadings)
        {
          // delete old readings
          delete [] m_pRangeReadings;

          // store size of array!
          m_NumberOfRangeReadings = static_cast<kt_int32u>(rRangeReadings.size());

          // allocate range readings
          m_pRangeReadings = new kt_double[m_NumberOfRangeReadings];
        }

        // copy readings
        kt_int32u index = 0;
        const_forEach(RangeReadingsVector, &rRangeReadings)
        {
          m_pRangeReadings[index++] = *iter;
        }

        std::cout << "copy readings number = "<< index << std::endl;
      }
      else
      {
        delete [] m_pRangeReadings;
        m_pRangeReadings = NULL;
      }
    }

    // /**
     // * Gets the laser range finder sensor that generated this scan
     // * @return laser range finder sensor of this scan
     // */
    // inline LaserRangeFinder* GetLaserRangeFinder() const
    // {
      // return SensorManager::GetInstance()->GetSensorByName<LaserRangeFinder>(GetSensorName());
    // }

    /**
     * Gets the number of range readings
     * @return number of range readings
     */
    inline kt_int32u GetNumberOfRangeReadings() const
    {
      return m_NumberOfRangeReadings;
    }

  private:
    LaserRangeScan(const LaserRangeScan&);
    const LaserRangeScan& operator=(const LaserRangeScan&);

  private:
    kt_double* m_pRangeReadings;
    kt_int32u m_NumberOfRangeReadings;
  };  // LaserRangeScan
  
  class LocalizedRangeScan : public LaserRangeScan
  {

  public:
    /**
     * Constructs a range scan from the given range finder with the given readings
     */
    LocalizedRangeScan(const RangeReadingsVector& rReadings, LaserRangeFinder* pLaserRangeFinder)
      : LaserRangeScan(rReadings)
      , m_pLaserRangeFinder(pLaserRangeFinder)
      , m_IsDirty(true)
    {
    }

    /**
     * Destructor
     */
    virtual ~LocalizedRangeScan()
    {
    }

  private:
    mutable boost::shared_mutex m_Lock;

  public:
    /**
     * Gets the odometric pose of this scan
     * @return odometric pose of this scan
     */
    inline const Pose2& GetOdometricPose() const
    {
      return m_OdometricPose;
    }

    /**
     * Sets the odometric pose of this scan
     * @param rPose
     */
    inline void SetOdometricPose(const Pose2& rPose)
    {
      m_OdometricPose = rPose;
    }

    /**
     * Gets the (possibly corrected) robot pose at which this scan was taken.  The corrected robot pose of the scan
     * is usually set by an external module such as a localization or mapping module when it is determined
     * that the original pose was incorrect.  The external module will set the correct pose based on
     * additional sensor data and any context information it has.  If the pose has not been corrected,
     * a call to this method returns the same pose as GetOdometricPose().
     * @return corrected pose
     */
    inline const Pose2& GetCorrectedPose() const
    {
      return m_CorrectedPose;
    }

    /**
     * Moves the scan by moving the robot pose to the given location.
     * @param rPose new pose of the robot of this scan
     */
    inline void SetCorrectedPose(const Pose2& rPose)
    {
      m_CorrectedPose = rPose;

      m_IsDirty = true;
    }

    /**
     * Gets barycenter of point readings
     */
    // inline const Pose2& GetBarycenterPose() const
    // {
    //   boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    //   if (m_IsDirty)
    //   {
    //     // throw away constness and do an update!
    //     lock.unlock();
    //     boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
    //     const_cast<LocalizedRangeScan*>(this)->Update();
    //   }

    //   return m_BarycenterPose;
    // }

    /**
     * Gets barycenter if the given parameter is true, otherwise returns the scanner pose
     * @param useBarycenter
     * @return barycenter if parameter is true, otherwise scanner pose
     */
    // inline Pose2 GetReferencePose(kt_bool useBarycenter) const
    // {
    //   boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    //   if (m_IsDirty)
    //   {
    //     // throw away constness and do an update!
    //     lock.unlock();
    //     boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
    //     const_cast<LocalizedRangeScan*>(this)->Update();
    //   }

    //   return useBarycenter ? GetBarycenterPose() : GetSensorPose();
    // }

    /**
     * Computes the position of the sensor
     * @return scan pose
     */
     inline Pose2 GetSensorPose() const
     {
       //return GetSensorAt(m_CorrectedPose);
        return m_CorrectedPose;
     }

    /**
     * Computes the robot pose given the corrected scan pose
     * @param rScanPose pose of the sensor
     */
    // void SetSensorPose(const Pose2& rScanPose)
    // {
    //   Pose2 deviceOffsetPose2 = GetLaserRangeFinder()->GetOffsetPose();
    //   kt_double offsetLength = deviceOffsetPose2.GetPosition().Length();
    //   kt_double offsetHeading = deviceOffsetPose2.GetHeading();
    //   kt_double angleoffset = atan2(deviceOffsetPose2.GetY(), deviceOffsetPose2.GetX());
    //   kt_double correctedHeading = math::NormalizeAngle(rScanPose.GetHeading());
    //   Pose2 worldSensorOffset = Pose2(offsetLength * cos(correctedHeading + angleoffset - offsetHeading),
    //                                   offsetLength * sin(correctedHeading + angleoffset - offsetHeading),
    //                                   offsetHeading);

    //   m_CorrectedPose = rScanPose - worldSensorOffset;

    //   Update();
    // }

    /**
     * Computes the position of the sensor if the robot were at the given pose
     * @param rPose
     * @return sensor pose
     */
    // inline Pose2 GetSensorAt(const Pose2& rPose) const
    // {
    //   return Transform(rPose).TransformPose(GetLaserRangeFinder()->GetOffsetPose());
    // }

    /**
     * Gets the bounding box of this scan
     * @return bounding box of this scan
     */
    // inline const BoundingBox2& GetBoundingBox() const
    // {
    //   boost::shared_lock<boost::shared_mutex> lock(m_Lock);
    //   if (m_IsDirty)
    //   {
    //     // throw away constness and do an update!
    //     lock.unlock();
    //     boost::unique_lock<boost::shared_mutex> uniqueLock(m_Lock);
    //     const_cast<LocalizedRangeScan*>(this)->Update();
    //   }

    //   return m_BoundingBox;
    // }

    /**
     * Get point readings in local coordinates
     */
     inline const PointVectorDouble& GetPointReadings(kt_bool wantFiltered = false)
     {
        Pose2 scanPose = GetSensorPose();

        m_PointReadings.clear();

    // compute point readings
       kt_int32u beamNum = 0;
       kt_double* pRangeReadings = GetRangeReadings();

       std::cout<<"number of RangeReadings = "<<m_pLaserRangeFinder->GetNumberOfRangeReadings()<<std::endl;

       for (kt_int32u i = 0; i < m_pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++)
       {
         kt_double rangeReading = pRangeReadings[i];

         

         // std::cout<< "MinimumRange = "<< m_pLaserRangeFinder->GetMinimumRange()<<std::endl;

         // std::cout<< "RangeThreshold = "<< m_pLaserRangeFinder->GetRangeThreshold()<<std::endl;

      
          if (!math::InRange(rangeReading, m_pLaserRangeFinder->GetMinimumRange(), m_pLaserRangeFinder->GetRangeThreshold()))
          {
            continue;
          }

          //std::cout<< "rangeReading = "<< rangeReading<<std::endl;
        
        //  else
        // {
        //   rangeReading = math::Clip(rangeReading, m_pLaserRangeFinder->GetMinimumRange(), m_pLaserRangeFinder->GetRangeThreshold());
        // }

        kt_double angle = scanPose.GetHeading() + m_pLaserRangeFinder->GetMinimumAngle() + beamNum * m_pLaserRangeFinder->GetAngularResolution();

        Vector2<kt_double> point;

        point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));//全局坐标系下的位姿
        point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));//全局坐标系下的位姿

      // if (pCoordinateConverter != NULL)
      // {
      //   Vector2<kt_int32s> gridPoint = pCoordinateConverter->WorldToGrid(point, flipY);
      //   point.SetX(gridPoint.GetX());
      //   point.SetY(gridPoint.GetY());
      // }

        m_PointReadings.push_back(point);
    }

    std::cout<< "m_PointReadings size is: "<<m_PointReadings.size()<<std::endl;

    return m_PointReadings;

  }

  private:
    /**
     * Compute point readings based on range readings
     * Only range readings within [minimum range; range threshold] are returned
     */
    // virtual void Update()
    // {
    //   LaserRangeFinder* pLaserRangeFinder = GetLaserRangeFinder();

    //   if (pLaserRangeFinder != NULL)
    //   {
    //     m_PointReadings.clear();
    //     m_UnfilteredPointReadings.clear();

    //     kt_double rangeThreshold = pLaserRangeFinder->GetRangeThreshold();
    //     kt_double minimumAngle = pLaserRangeFinder->GetMinimumAngle();
    //     kt_double angularResolution = pLaserRangeFinder->GetAngularResolution();
    //     Pose2 scanPose = GetSensorPose();

    //     // compute point readings
    //     Vector2<kt_double> rangePointsSum;
    //     kt_int32u beamNum = 0;
    //     for (kt_int32u i = 0; i < pLaserRangeFinder->GetNumberOfRangeReadings(); i++, beamNum++)
    //     {
    //       kt_double rangeReading = GetRangeReadings()[i];
    //       if (!math::InRange(rangeReading, pLaserRangeFinder->GetMinimumRange(), rangeThreshold))
    //       {
    //         kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

    //         Vector2<kt_double> point;
    //         point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
    //         point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

    //         m_UnfilteredPointReadings.push_back(point);
    //         continue;
    //       }

    //       kt_double angle = scanPose.GetHeading() + minimumAngle + beamNum * angularResolution;

    //       Vector2<kt_double> point;
    //       point.SetX(scanPose.GetX() + (rangeReading * cos(angle)));
    //       point.SetY(scanPose.GetY() + (rangeReading * sin(angle)));

    //       m_PointReadings.push_back(point);
    //       m_UnfilteredPointReadings.push_back(point);

    //       rangePointsSum += point;
    //     }

    //     // compute barycenter
    //     kt_double nPoints = static_cast<kt_double>(m_PointReadings.size());
    //     if (nPoints != 0.0)
    //     {
    //       Vector2<kt_double> averagePosition = Vector2<kt_double>(rangePointsSum / nPoints);
    //       m_BarycenterPose = Pose2(averagePosition, 0.0);
    //     }
    //     else
    //     {
    //       m_BarycenterPose = scanPose;
    //     }

    //     // calculate bounding box of scan
    //     m_BoundingBox = BoundingBox2();
    //     m_BoundingBox.Add(scanPose.GetPosition());
    //     forEach(PointVectorDouble, &m_PointReadings)
    //     {
    //       m_BoundingBox.Add(*iter);
    //     }
    //   }

    //   m_IsDirty = false;
    // }

  private:
    LocalizedRangeScan(const LocalizedRangeScan&);
    const LocalizedRangeScan& operator=(const LocalizedRangeScan&);

  private:
    /**
     * Odometric pose of robot
     */
    Pose2 m_OdometricPose;

    /**
     * Corrected pose of robot calculated by mapper (or localizer)
     */
    Pose2 m_CorrectedPose;

  protected:

    LaserRangeFinder* m_pLaserRangeFinder;
    /**
     * Average of all the point readings
     */
    Pose2 m_BarycenterPose;

    /**
     * Vector of point readings
     */
    PointVectorDouble m_PointReadings;

    /**
     * Vector of unfiltered point readings
     */
    PointVectorDouble m_UnfilteredPointReadings;

    /**
     * Bounding box of localized range scan
     */
    BoundingBox2 m_BoundingBox;

    /**
     * Internal flag used to update point readings, barycenter and bounding box
     */
    kt_bool m_IsDirty;
  };  // LocalizedRangeScan

   /**
   * Type declaration of LocalizedRangeScan vector
   */
  typedef std::vector<LocalizedRangeScan*> LocalizedRangeScanVector;

}//namespace karto

#endif
