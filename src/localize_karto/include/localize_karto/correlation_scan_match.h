#ifndef CORRELATION_SCAN_MATCH_H
#define CORRELATION_SCAN_MATCH_H


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
#include "Grid.h"
#include "data_structure.h"
#include "laser_scan.h"
#include "Matrix.h"

namespace karto
{
  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

/**
   * Scan matcher
   */
  class KARTO_EXPORT ScanMatcher
  {
  public:
    /**
     * Destructor
     */
    virtual ~ScanMatcher();

  public:
    /**
     * Create a scan matcher with the given parameters
     */
    static ScanMatcher* Create(Mapper* pMapper,
                               kt_double searchSize,
                               kt_double resolution,
                               kt_double smearDeviation,
                               kt_double rangeThreshold,
                               CorrelationGrid* pCorrelationGrid);

    /**
     * Match given scan against set of scans
     * @param pScan scan being scan-matched
     * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doPenalize whether to penalize matches further from the search center
     * @param doRefineMatch whether to do finer-grained matching if coarse match is good (default is true)
     * @return strength of response
     */
    kt_double MatchScan(LocalizedRangeScan* pScan,
                        Pose2& rMean, Matrix3& rCovariance,
                        kt_bool doPenalize = true,
                        kt_bool doRefineMatch = true);

    /**
     * Finds the best pose for the scan centering the search in the correlation grid
     * at the given pose and search in the space by the vector and angular offsets
     * in increments of the given resolutions
     * @param pScan scan to match against correlation grid
     * @param rSearchCenter the center of the search space
     * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
     * @param rSearchSpaceResolution how fine a granularity to search in the search space
     * @param searchAngleOffset searches poses in the angles offset by this angle around search center
     * @param searchAngleResolution how fine a granularity to search in the angular search space
     * @param doPenalize whether to penalize matches further from the search center
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doingFineMatch whether to do a finer search after coarse search
     * @return strength of response
     */
    kt_double CorrelateScan(LocalizedRangeScan* pScan,
                            const Pose2& rSearchCenter,
                            const Vector2<kt_double>& rSearchSpaceOffset,
                            const Vector2<kt_double>& rSearchSpaceResolution,
                            kt_double searchAngleOffset,
                            kt_double searchAngleResolution,
                            kt_bool doPenalize,
                            Pose2& rMean,
                            Matrix3& rCovariance,
                            kt_bool doingFineMatch);

    /**
     * Computes the positional covariance of the best pose
     * @param rBestPose
     * @param bestResponse
     * @param rSearchCenter
     * @param rSearchSpaceOffset
     * @param rSearchSpaceResolution
     * @param searchAngleResolution
     * @param rCovariance
     */
    void ComputePositionalCovariance(const Pose2& rBestPose,
                                     kt_double bestResponse,
                                     const Pose2& rSearchCenter,
                                     const Vector2<kt_double>& rSearchSpaceOffset,
                                     const Vector2<kt_double>& rSearchSpaceResolution,
                                     kt_double searchAngleResolution,
                                     Matrix3& rCovariance);

    /**
     * Computes the angular covariance of the best pose
     * @param rBestPose
     * @param bestResponse
     * @param rSearchCenter
     * @param searchAngleOffset
     * @param searchAngleResolution
     * @param rCovariance
     */
    void ComputeAngularCovariance(const Pose2& rBestPose,
                                  kt_double bestResponse,
                                  const Pose2& rSearchCenter,
                                  kt_double searchAngleOffset,
                                  kt_double searchAngleResolution,
                                  Matrix3& rCovariance);

    /**
     * Gets the correlation grid data (for debugging)
     * @return correlation grid
     */
    inline CorrelationGrid* GetCorrelationGrid() const
    {
      return m_pCorrelationGrid;
    }

  private:
    /**
     * Marks cells where scans' points hit as being occupied
     * @param rScans scans whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     */
    //void AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint);

    /**
     * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
     * @param pScan scan whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     * @param doSmear whether the points will be smeared
     */
    //void AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear = true);

    /**
     * Compute which points in a scan are on the same side as the given viewpoint
     * @param pScan
     * @param rViewPoint
     * @return points on the same side
     */
    //PointVectorDouble FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const;

    /**
     * Get response at given position for given rotation (only look up valid points)
     * @param angleIndex
     * @param gridPositionIndex
     * @return response
     */
    kt_double GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const;

  protected:
    /**
     * Default constructor
     */
    ScanMatcher(Mapper* pMapper)
      : m_pMapper(pMapper)
      , m_pCorrelationGrid(NULL)
      , m_pSearchSpaceProbs(NULL)
      , m_pGridLookup(NULL)
    {
    }

  private:
    Mapper* m_pMapper;

    CorrelationGrid* m_pCorrelationGrid;
    Grid<kt_double>* m_pSearchSpaceProbs;

    GridIndexLookup<kt_int8u>* m_pGridLookup;
  };  // ScanMatcher
}//namespace karto

#endif