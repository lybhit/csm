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
#include <boost/thread.hpp>

#include "localize_karto/correlation_scan_match.h"

namespace karto
{
ScanMatcher::~ScanMatcher()
  {
  //  delete m_pCorrelationGrid;
    m_pCorrelationGrid = NULL;
    delete m_pSearchSpaceProbs;
    delete m_pGridLookup;

  }

  ScanMatcher* ScanMatcher::Create(Mapper* pMapper, kt_double searchSize, kt_double resolution,
                                   kt_double smearDeviation, kt_double rangeThreshold, 
                                   CorrelationGrid* pCorrelationGrid)
  {
    // invalid parameters
    if (resolution <= 0)
    {
      return NULL;
    }
    if (searchSize <= 0)
    {
      return NULL;
    }
    if (smearDeviation < 0)
    {
      return NULL;
    }
    if (rangeThreshold <= 0)
    {
      return NULL;
    }

    assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));

    // calculate search space in grid coordinates
    kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);

    // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
    // if a scan is on the border of the search space)
    // kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));

//    kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;

    // create correlation grid
 //   assert(gridSize % 2 == 1);
//    CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

    // create search space probabilities
    Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize,
                                                                     searchSpaceSideSize, resolution);

    ScanMatcher* pScanMatcher = new ScanMatcher(pMapper);

    // std::cout<< "ok, it is now to make a grid"<<std::endl;

    // std::cout<< "m_pCorrelationGrid " << pCorrelationGrid<<std::endl; 

    // for(int i = 0; i < 60; ++i)
    //   std::cout<< "pCorrelationGrid data i = "<< pCorrelationGrid->GetDataPointer()[i]<<std::endl;

    pScanMatcher->m_pCorrelationGrid = pCorrelationGrid;
    pScanMatcher->m_pSearchSpaceProbs = pSearchSpaceProbs;

    std::cout<<"initialize GridLookup" <<std::endl;
    // for(int i = 0; i < 60; ++i)
    //   std::cout<< "pCorrelationGrid data i = "<< pCorrelationGrid->GetDataPointer()[i]<<std::endl;

    pScanMatcher->m_pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);

    for(kt_int32s i = 0; i < pCorrelationGrid->GetDataSize(); ++i)
    {
      pCorrelationGrid->SmearPoint(i);

    }

    return pScanMatcher;
  }

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
  kt_double ScanMatcher::MatchScan(LocalizedRangeScan* pScan, Pose2& rMean,
                                   Matrix3& rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
  {
    ////////////////////1///////////////////
    // set scan pose to be center of grid

    std::cout << "start matching to the map!" <<std::endl;

    // 1. get scan position
    Pose2 scanPose = pScan->GetSensorPose();

    // scan has no readings; cannot do scan matching
    // best guess of pose is based off of adjusted odometer reading
    if (pScan->GetNumberOfRangeReadings() == 0)
    {
      rMean = scanPose;

      // maximum covariance
      rCovariance(0, 0) = MAX_VARIANCE;  // XX
      rCovariance(1, 1) = MAX_VARIANCE;  // YY
      rCovariance(2, 2) = 4 * math::Square(m_pMapper->m_pCoarseAngleResolution->GetValue());  // TH*TH

      return 0.0;
    }

    // 2. get size of grid
    Rectangle2<kt_int32s> roi = m_pCorrelationGrid->GetROI();

    // 3. compute offset (in meters - lower left corner)
    // Vector2<kt_double> offset;
    // offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * m_pCorrelationGrid->GetResolution()));
    // offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * m_pCorrelationGrid->GetResolution()));

    // // 4. set offset
    // m_pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);

    ///////////////////////////////////////

    // set up correlation grid
    //AddScans(rBaseScans, scanPose.GetPosition());

    // compute how far to search in each direction
    Vector2<kt_double> searchDimensions(m_pSearchSpaceProbs->GetWidth(), m_pSearchSpaceProbs->GetHeight());
    Vector2<kt_double> coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->GetResolution(),
                                          0.5 * (searchDimensions.GetY() - 1) * m_pCorrelationGrid->GetResolution());

    std::cout << "coarseSearchOffset -> x " << 0.5 * (searchDimensions.GetX() - 1) * m_pCorrelationGrid->GetResolution()<<std::endl;

    // a coarse search only checks half the cells in each dimension
    Vector2<kt_double> coarseSearchResolution(2 * m_pCorrelationGrid->GetResolution(),
                                              2 * m_pCorrelationGrid->GetResolution());

    // actual scan-matching
    kt_double bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset, coarseSearchResolution,
                                           m_pMapper->m_pCoarseSearchAngleOffset->GetValue(),
                                           m_pMapper->m_pCoarseAngleResolution->GetValue(),
                                           doPenalize, rMean, rCovariance, false);

    if (m_pMapper->m_pUseResponseExpansion->GetValue() == true)
    {
      if (math::DoubleEqual(bestResponse, 0.0))
      {
#ifdef KARTO_DEBUG
        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
        // try and increase search angle offset with 20 degrees and do another match
        kt_double newSearchAngleOffset = m_pMapper->m_pCoarseSearchAngleOffset->GetValue();
        for (kt_int32u i = 0; i < 3; i++)
        {
          newSearchAngleOffset += math::DegreesToRadians(20);

          bestResponse = CorrelateScan(pScan, scanPose, coarseSearchOffset, coarseSearchResolution,
                                       newSearchAngleOffset, m_pMapper->m_pCoarseAngleResolution->GetValue(),
                                       doPenalize, rMean, rCovariance, false);

          if (math::DoubleEqual(bestResponse, 0.0) == false)
          {
            break;
          }
        }

// #ifdef KARTO_DEBUG
        if (math::DoubleEqual(bestResponse, 0.0))
        {
          std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
        }
// #endif
      }
    }

    if (doRefineMatch)
    {
      Vector2<kt_double> fineSearchOffset(coarseSearchResolution * 0.5);
      Vector2<kt_double> fineSearchResolution(m_pCorrelationGrid->GetResolution(), m_pCorrelationGrid->GetResolution());
      bestResponse = CorrelateScan(pScan, rMean, fineSearchOffset, fineSearchResolution,
                                   0.5 * m_pMapper->m_pCoarseAngleResolution->GetValue(),
                                   m_pMapper->m_pFineSearchAngleOffset->GetValue(),
                                   doPenalize, rMean, rCovariance, true);
    }

// #ifdef KARTO_DEBUG
    std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = "
              << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
// #endif
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    return bestResponse;
  }

  /**
   * Finds the best pose for the scan centering the search in the correlation grid
   * at the given pose and search in the space by the vector and angular offsets
   * in increments of the given resolutions
   * @param rScan scan to match against correlation grid
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
  kt_double ScanMatcher::CorrelateScan(LocalizedRangeScan* pScan, const Pose2& rSearchCenter,
                                       const Vector2<kt_double>& rSearchSpaceOffset,
                                       const Vector2<kt_double>& rSearchSpaceResolution,
                                       kt_double searchAngleOffset, kt_double searchAngleResolution,
                                       kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch)
  {
    assert(searchAngleResolution != 0.0);

    std::cout<< "Start correlation scan!!!"<<std::endl;

    // setup lookup arrays
    m_pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);

    // only initialize probability grid if computing positional covariance (during coarse match)
    if (!doingFineMatch)
    {
      m_pSearchSpaceProbs->Clear();

      // position search grid - finds lower left corner of search grid
      Vector2<kt_double> offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
      m_pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
    }

    // calculate position arrays

    std::vector<kt_double> xPoses;
    kt_int32u nX = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() *
                                          2.0 / rSearchSpaceResolution.GetX()) + 1);

    std::cout<<"SearchSpaceOffset_X"<< rSearchSpaceOffset.GetX() << std::endl;

    std::cout<<"SearchSpaceResolution_X"<< rSearchSpaceResolution.GetX() << std::endl;



    kt_double startX = -rSearchSpaceOffset.GetX();
    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
    {
      xPoses.push_back(startX + xIndex * rSearchSpaceResolution.GetX());
    }

    std::cout<<"nX numbers = "<< xPoses.size() << std::endl;

    assert(math::DoubleEqual(xPoses.back(), -startX));



    std::vector<kt_double> yPoses;
    kt_int32u nY = static_cast
    <kt_int32u>(math::Round(rSearchSpaceOffset.GetY() *
                                          2.0 / rSearchSpaceResolution.GetY()) + 1);

    std::cout<<"nY numbers = "<< nY << std::endl;

    kt_double startY = -rSearchSpaceOffset.GetY();
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      yPoses.push_back(startY + yIndex * rSearchSpaceResolution.GetY());
    }
    assert(math::DoubleEqual(yPoses.back(), -startY));

    // calculate pose response array size
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);

    std::cout<<"nAngles numbers = "<< nAngles << std::endl;

    kt_int32u poseResponseSize = static_cast<kt_int32u>(xPoses.size() * yPoses.size() * nAngles);

    std::cout<<"loop numbers = "<< poseResponseSize << std::endl;

    // allocate array
    std::pair<kt_double, Pose2>* pPoseResponse = new std::pair<kt_double, Pose2>[poseResponseSize];

    Vector2<kt_int32s> startGridPoint = m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX()
                                                                        + startX, rSearchCenter.GetY() + startY));

    std::cout<<"startGridPoint_World_X "<< rSearchCenter.GetX() + startX << std::endl;
    std::cout<<"startGridPoint_World_Y "<< rSearchCenter.GetY() + startY << std::endl;

    std::cout<<"startGridPoint_Grid_X "<< startGridPoint.GetX() << std::endl;
    std::cout<<"startGridPoint_Grid_Y "<< startGridPoint.GetY() << std::endl;

    kt_int32u poseResponseCounter = 0;
    forEachAs(std::vector<kt_double>, &yPoses, yIter)
    {
      kt_double y = *yIter;
      kt_double newPositionY = rSearchCenter.GetY() + y;

      std::cout<<"rSearchCenter_Y "<< rSearchCenter.GetY() << std::endl;

      kt_double squareY = math::Square(y);

      forEachAs(std::vector<kt_double>, &xPoses, xIter)
      {
        kt_double x = *xIter;
        kt_double newPositionX = rSearchCenter.GetX() + x;
        kt_double squareX = math::Square(x);

        Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(Vector2<kt_double>(newPositionX, newPositionY));
        kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

        // std::cout<<"gridPoint_X "<< gridPoint.GetX() << std::endl;

        // std::cout<<"gridPoint_Y "<< gridPoint.GetY() << std::endl;

        // std::cout<< "gridIndex = " << m_pCorrelationGrid->GridIndex(gridPoint)<< std::endl;

        //assert(gridIndex >= 0);

        kt_double angle = 0.0;
        kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
        for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
        {
          angle = startAngle + angleIndex * searchAngleResolution;

          kt_double response = GetResponse(angleIndex, gridIndex);
          if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
          {
            // simple model (approximate Gaussian) to take odometry into account

            kt_double squaredDistance = squareX + squareY;
            kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN *
                                               squaredDistance / m_pMapper->m_pDistanceVariancePenalty->GetValue());
            distancePenalty = math::Maximum(distancePenalty, m_pMapper->m_pMinimumDistancePenalty->GetValue());

            kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
            kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN *
                                            squaredAngleDistance / m_pMapper->m_pAngleVariancePenalty->GetValue());
            anglePenalty = math::Maximum(anglePenalty, m_pMapper->m_pMinimumAnglePenalty->GetValue());

            response *= (distancePenalty * anglePenalty);
          }

          // store response and pose
          pPoseResponse[poseResponseCounter] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY,
                                                                           math::NormalizeAngle(angle)));
          poseResponseCounter++;
        }

        assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
      }
    }

    assert(poseResponseSize == poseResponseCounter);

    // find value of best response (in [0; 1])
    kt_double bestResponse = -1;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      bestResponse = math::Maximum(bestResponse, pPoseResponse[i].first);

      // will compute positional covariance, save best relative probability for each cell
      if (!doingFineMatch)
      {
        const Pose2& rPose = pPoseResponse[i].second;
        Vector2<kt_int32s> grid = m_pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());

        // Changed (kt_double*) to the reinterpret_cast - Luc
        kt_double* ptr = reinterpret_cast<kt_double*>(m_pSearchSpaceProbs->GetDataPointer(grid));
        if (ptr == NULL)
        {
          throw std::runtime_error("Mapper FATAL ERROR - Index out of range in probability search!");
        }

        *ptr = math::Maximum(pPoseResponse[i].first, *ptr);
      }
    }

    // average all poses with same highest response
    Vector2<kt_double> averagePosition;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;
    kt_int32s averagePoseCount = 0;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      if (math::DoubleEqual(pPoseResponse[i].first, bestResponse))
      {
        averagePosition += pPoseResponse[i].second.GetPosition();

        kt_double heading = pPoseResponse[i].second.GetHeading();
        thetaX += cos(heading);
        thetaY += sin(heading);

        averagePoseCount++;
      }
    }

    Pose2 averagePose;
    if (averagePoseCount > 0)
    {
      averagePosition /= averagePoseCount;

      thetaX /= averagePoseCount;
      thetaY /= averagePoseCount;

      averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
    }
    else
    {
      throw std::runtime_error("Mapper FATAL ERROR - Unable to find best position");
    }

    // delete pose response array
    delete [] pPoseResponse;

// #ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
    std::cout << "bestResponse: " << bestResponse << std::endl;
// #endif

    if (!doingFineMatch)
    {
      ComputePositionalCovariance(averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset,
                                  rSearchSpaceResolution, searchAngleResolution, rCovariance);
    }
    else
    {
      std::cout<< "compute angular covariance!!!" << std::endl; 
      ComputeAngularCovariance(averagePose, bestResponse, rSearchCenter,
                              searchAngleOffset, searchAngleResolution, rCovariance);
    }

    rMean = averagePose;

#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
#endif

    if (bestResponse > 1.0)
    {
      bestResponse = 1.0;
    }

    assert(math::InRange(bestResponse, 0.0, 1.0));
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    return bestResponse;
  }

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
  void ScanMatcher::ComputePositionalCovariance(const Pose2& rBestPose, kt_double bestResponse,
                                                const Pose2& rSearchCenter,
                                                const Vector2<kt_double>& rSearchSpaceOffset,
                                                const Vector2<kt_double>& rSearchSpaceResolution,
                                                kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // reset covariance to identity matrix
    rCovariance.SetToIdentity();

    // if best response is vary small return max variance
    if (bestResponse < KT_TOLERANCE)
    {
      rCovariance(0, 0) = MAX_VARIANCE;  // XX
      rCovariance(1, 1) = MAX_VARIANCE;  // YY
      rCovariance(2, 2) = 4 * math::Square(searchAngleResolution);  // TH*TH

      return;
    }

    kt_double accumulatedVarianceXX = 0;
    kt_double accumulatedVarianceXY = 0;
    kt_double accumulatedVarianceYY = 0;
    kt_double norm = 0;

    kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
    kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();

    kt_double offsetX = rSearchSpaceOffset.GetX();
    kt_double offsetY = rSearchSpaceOffset.GetY();

    kt_int32u nX = static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    kt_double startX = -offsetX;
    assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));

    kt_int32u nY = static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    kt_double startY = -offsetY;
    assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));

    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();

      for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
      {
        kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();

        Vector2<kt_int32s> gridPoint = m_pSearchSpaceProbs->WorldToGrid(Vector2<kt_double>(rSearchCenter.GetX() + x,
                                                                                           rSearchCenter.GetY() + y));
        kt_double response = *(m_pSearchSpaceProbs->GetDataPointer(gridPoint));

        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceXX += (math::Square(x - dx) * response);
          accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
          accumulatedVarianceYY += (math::Square(y - dy) * response);
        }
      }
    }

    if (norm > KT_TOLERANCE)
    {
      kt_double varianceXX = accumulatedVarianceXX / norm;
      kt_double varianceXY = accumulatedVarianceXY / norm;
      kt_double varianceYY = accumulatedVarianceYY / norm;
      kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);

      // lower-bound variances so that they are not too small;
      // ensures that links are not too tight
      kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
      kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
      varianceXX = math::Maximum(varianceXX, minVarianceXX);
      varianceYY = math::Maximum(varianceYY, minVarianceYY);

      // increase variance for poorer responses
      kt_double multiplier = 1.0 / bestResponse;
      rCovariance(0, 0) = varianceXX * multiplier;
      rCovariance(0, 1) = varianceXY * multiplier;
      rCovariance(1, 0) = varianceXY * multiplier;
      rCovariance(1, 1) = varianceYY * multiplier;
      rCovariance(2, 2) = varianceTHTH;  // this value will be set in ComputeAngularCovariance
    }

    // if values are 0, set to MAX_VARIANCE
    // values might be 0 if points are too sparse and thus don't hit other points
    if (math::DoubleEqual(rCovariance(0, 0), 0.0))
    {
      rCovariance(0, 0) = MAX_VARIANCE;
    }

    if (math::DoubleEqual(rCovariance(1, 1), 0.0))
    {
      rCovariance(1, 1) = MAX_VARIANCE;
    }
  }

  /**
   * Computes the angular covariance of the best pose
   * @param rBestPose
   * @param bestResponse
   * @param rSearchCenter
   * @param rSearchAngleOffset
   * @param searchAngleResolution
   * @param rCovariance
   */
  void ScanMatcher::ComputeAngularCovariance(const Pose2& rBestPose,
                                             kt_double bestResponse,
                                             const Pose2& rSearchCenter,
                                             kt_double searchAngleOffset,
                                             kt_double searchAngleResolution,
                                             Matrix3& rCovariance)
  {
    // NOTE: do not reset covariance matrix

    // normalize angle difference
    kt_double bestAngle = math::NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());

    Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
    kt_int32s gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);

    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;

    kt_double norm = 0.0;
    kt_double accumulatedVarianceThTh = 0.0;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      kt_double response = GetResponse(angleIndex, gridIndex);

      // response is not a low response
      if (response >= (bestResponse - 0.1))
      {
        norm += response;
        accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
      }
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));

    if (norm > KT_TOLERANCE)
    {
      if (accumulatedVarianceThTh < KT_TOLERANCE)
      {
        accumulatedVarianceThTh = math::Square(searchAngleResolution);
      }

      accumulatedVarianceThTh /= norm;
    }
    else
    {
      accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
    }

    rCovariance(2, 2) = accumulatedVarianceThTh;
  }

  /**
   * Marks cells where scans' points hit as being occupied
   * @param rScans scans whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   */
  // void ScanMatcher::AddScans(const LocalizedRangeScanVector& rScans, Vector2<kt_double> viewPoint)
  // {
  //   m_pCorrelationGrid->Clear();

  //   // add all scans to grid
  //   const_forEach(LocalizedRangeScanVector, &rScans)
  //   {
  //     AddScan(*iter, viewPoint);
  //   }
  // }

  /**
   * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
   * @param pScan scan whose points will mark cells in grid as being occupied
   * @param viewPoint do not add points that belong to scans "opposite" the view point
   * @param doSmear whether the points will be smeared
   */
  // void ScanMatcher::AddScan(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint, kt_bool doSmear)
  // {
  //   PointVectorDouble validPoints = FindValidPoints(pScan, rViewPoint);

  //   // put in all valid points
  //   const_forEach(PointVectorDouble, &validPoints)
  //   {
  //     Vector2<kt_int32s> gridPoint = m_pCorrelationGrid->WorldToGrid(*iter);
  //     if (!math::IsUpTo(gridPoint.GetX(), m_pCorrelationGrid->GetROI().GetWidth()) ||
  //         !math::IsUpTo(gridPoint.GetY(), m_pCorrelationGrid->GetROI().GetHeight()))
  //     {
  //       // point not in grid
  //       continue;
  //     }

  //     int gridIndex = m_pCorrelationGrid->GridIndex(gridPoint);

  //     // set grid cell as occupied
  //     if (m_pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
  //     {
  //       // value already set
  //       continue;
  //     }

  //     m_pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

  //     // smear grid
  //     if (doSmear == true)
  //     {
  //       m_pCorrelationGrid->SmearPoint(gridPoint);
  //     }
  //   }
  // }

  // /**
  //  * Compute which points in a scan are on the same side as the given viewpoint
  //  * @param pScan
  //  * @param rViewPoint
  //  * @return points on the same side
  //  */
  // PointVectorDouble ScanMatcher::FindValidPoints(LocalizedRangeScan* pScan, const Vector2<kt_double>& rViewPoint) const
  // {
  //   const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

  //   // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
  //   const kt_double minSquareDistance = math::Square(0.1);  // in m^2

  //   // this iterator lags from the main iterator adding points only when the points are on
  //   // the same side as the viewpoint
  //   PointVectorDouble::const_iterator trailingPointIter = rPointReadings.begin();
  //   PointVectorDouble validPoints;

  //   Vector2<kt_double> firstPoint;
  //   kt_bool firstTime = true;
  //   const_forEach(PointVectorDouble, &rPointReadings)
  //   {
  //     Vector2<kt_double> currentPoint = *iter;

  //     if (firstTime && !std::isnan(currentPoint.GetX()) && !std::isnan(currentPoint.GetY()))
  //     {
  //       firstPoint = currentPoint;
  //       firstTime = false;
  //     }

  //     Vector2<kt_double> delta = firstPoint - currentPoint;
  //     if (delta.SquaredLength() > minSquareDistance)
  //     {
  //       // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
  //       // Which computes the direction of rotation, if the rotation is counterclock
  //       // wise then we are looking at data we should keep. If it's negative rotation
  //       // we should not included in in the matching
  //       // have enough distance, check viewpoint
  //       double a = rViewPoint.GetY() - firstPoint.GetY();
  //       double b = firstPoint.GetX() - rViewPoint.GetX();
  //       double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
  //       double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;

  //       // reset beginning point
  //       firstPoint = currentPoint;

  //       if (ss < 0.0)  // wrong side, skip and keep going
  //       {
  //         trailingPointIter = iter;
  //       }
  //       else
  //       {
  //         for (; trailingPointIter != iter; ++trailingPointIter)
  //         {
  //           validPoints.push_back(*trailingPointIter);
  //         }
  //       }
  //     }
  //   }

  //   return validPoints;
  // }

  /**
   * Get response at given position for given rotation (only look up valid points)
   * @param angleIndex
   * @param gridPositionIndex
   * @return response
   */
  kt_double ScanMatcher::GetResponse(kt_int32u angleIndex, kt_int32s gridPositionIndex) const
  {
    kt_double response = 0.0;

    std::cout<<"To get the score!!!"<<std::endl;

    // add up value for each point
    kt_int8u* pByte = m_pCorrelationGrid->GetDataPointer() + gridPositionIndex;

    std::cout<<"*pByte"<< *pByte <<std::endl;

    const LookupArray* pOffsets = m_pGridLookup->GetLookupArray(angleIndex);
    assert(pOffsets != NULL);

    // get number of points in offset list
    kt_int32u nPoints = pOffsets->GetSize();
    if (nPoints == 0)
    {
      return response;
    }

    // calculate response
    kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
    for (kt_int32u i = 0; i < nPoints; i++)
    {
      // ignore points that fall off the grid
      kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
      if (!math::IsUpTo(pointGridIndex, m_pCorrelationGrid->GetDataSize()) || pAngleIndexPointer[i] == INVALID_SCAN)
      {
        continue;
      }

      //std::cout<< "Eatch grid score = "<< static_cast<int>(pByte[pAngleIndexPointer[i]])<<std::endl;

      // uses index offsets to efficiently find location of point in the grid
      response += pByte[pAngleIndexPointer[i]];
    }

    std::cout<< "nPoints = " << nPoints << std::endl;
    std::cout<< "GridStates_Occupied = " << GridStates_Occupied << std::endl;

    // normalize response
    response /= (nPoints * GridStates_Occupied);

    std::cout<< "The score is = " << response << std::endl;

    assert(fabs(response) <= 1.0);

    

    return response;
  }
}//namespace karto

