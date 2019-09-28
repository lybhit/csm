#ifndef _GRID_H
#define _GRID_H

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
#include "laser_scan.h"
#include "exception.h"

namespace karto
{

typedef enum
  {
    GridStates_Unknown = 0,
    GridStates_Occupied = 100,
    GridStates_Free = 0
  } GridStates;

/**
   * The CoordinateConverter class is used to convert coordinates between world and grid coordinates
   * In world coordinates 1.0 = 1 meter where 1 in grid coordinates = 1 pixel!
   * Default scale for coordinate converter is 20 that converters to 1 pixel = 0.05 meter
   */
  class CoordinateConverter
  {
  public:
    /**
     * Default constructor
     */
    CoordinateConverter()
      : m_Scale(20.0)
    {
    }

  public:
    /**
     * Scales the value
     * @param value
     * @return scaled value
     */
    inline kt_double Transform(kt_double value)
    {
      return value * m_Scale;
    }

    /**
     * Converts the point from world coordinates to grid coordinates
     * @param rWorld world coordinate
     * @param flipY
     * @return grid coordinate
     */
    inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
    {
      kt_double gridX = (rWorld.GetX() - m_Offset.GetX()) * m_Scale;
      kt_double gridY = 0.0;

      if (flipY == false)
      {
        gridY = (rWorld.GetY() - m_Offset.GetY()) * m_Scale;
      }
      else
      {
        gridY = (m_Size.GetHeight() / m_Scale - rWorld.GetY() + m_Offset.GetY()) * m_Scale;
      }

      return Vector2<kt_int32s>(static_cast<kt_int32s>(math::Round(gridX)), static_cast<kt_int32s>(math::Round(gridY)));
    }

    /**
     * Converts the point from grid coordinates to world coordinates
     * @param rGrid world coordinate
     * @param flipY
     * @return world coordinate
     */
    inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
    {
      kt_double worldX = m_Offset.GetX() + rGrid.GetX() / m_Scale;
      kt_double worldY = 0.0;

      if (flipY == false)
      {
        worldY = m_Offset.GetY() + rGrid.GetY() / m_Scale;
      }
      else
      {
        worldY = m_Offset.GetY() + (m_Size.GetHeight() - rGrid.GetY()) / m_Scale;
      }

      return Vector2<kt_double>(worldX, worldY);
    }

    /**
     * Gets the scale
     * @return scale
     */
    inline kt_double GetScale() const
    {
      return m_Scale;
    }

    /**
     * Sets the scale
     * @param scale
     */
    inline void SetScale(kt_double scale)
    {
      m_Scale = scale;
    }

    /**
     * Gets the offset
     * @return offset
     */
    inline const Vector2<kt_double>& GetOffset() const
    {
      return m_Offset;
    }

    /**
     * Sets the offset
     * @param rOffset
     */
    inline void SetOffset(const Vector2<kt_double>& rOffset)
    {
      m_Offset = rOffset;
    }

    /**
     * Sets the size
     * @param rSize
     */
    inline void SetSize(const Size2<kt_int32s>& rSize)
    {
      m_Size = rSize;
    }

    /**
     * Gets the size
     * @return size
     */
    inline const Size2<kt_int32s>& GetSize() const
    {
      return m_Size;
    }

    /**
     * Gets the resolution
     * @return resolution
     */
    inline kt_double GetResolution() const
    {
      return 1.0 / m_Scale;
    }

    /**
     * Sets the resolution
     * @param resolution
     */
    inline void SetResolution(kt_double resolution)
    {
      m_Scale = 1.0 / resolution;
    }

    /**
     * Gets the bounding box
     * @return bounding box
     */
    inline BoundingBox2 GetBoundingBox() const
    {
      BoundingBox2 box;

      kt_double minX = GetOffset().GetX();
      kt_double minY = GetOffset().GetY();
      kt_double maxX = minX + GetSize().GetWidth() * GetResolution();
      kt_double maxY = minY + GetSize().GetHeight() * GetResolution();

      box.SetMinimum(GetOffset());
      box.SetMaximum(Vector2<kt_double>(maxX, maxY));
      return box;
    }

  private:
    Size2<kt_int32s> m_Size;
    kt_double m_Scale;

    Vector2<kt_double> m_Offset;
  };  // CoordinateConverter



/**
   * Defines a grid class
   */
  template<typename T>
  class Grid
  {
  public:
    /**
     * Creates a grid of given size and resolution
     * @param width
     * @param height
     * @param resolution
     * @return grid pointer
     */
    static Grid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution)
    {
      Grid* pGrid = new Grid(width, height);

      pGrid->GetCoordinateConverter()->SetScale(1.0 / resolution);

      return pGrid;
    }

    /**
     * Destructor
     */
    virtual ~Grid()
    {
      delete [] m_pData;
      delete m_pCoordinateConverter;
    }

  public:
    /**
     * Clear out the grid data
     */
    void Clear()
    {
      memset(m_pData, 0, GetDataSize() * sizeof(T));
    }

    /**
     * Returns a clone of this grid
     * @return grid clone
     */
    Grid* Clone()
    {
      Grid* pGrid = CreateGrid(GetWidth(), GetHeight(), GetResolution());
      pGrid->GetCoordinateConverter()->SetOffset(GetCoordinateConverter()->GetOffset());

      memcpy(pGrid->GetDataPointer(), GetDataPointer(), GetDataSize());

      return pGrid;
    }

    /**
     * Resizes the grid (deletes all old data)
     * @param width
     * @param height
     */
    virtual void Resize(kt_int32s width, kt_int32s height)
    {
      m_Width = width;
      m_Height = height;
      // m_WidthStep = math::AlignValue<kt_int32s>(width, 8);

      if (m_pData != NULL)
      {
        delete[] m_pData;
        m_pData = NULL;
      }

      try
      {
        m_pData = new T[GetDataSize()];

        if (m_pCoordinateConverter == NULL)
        {
          m_pCoordinateConverter = new CoordinateConverter();
        }

        m_pCoordinateConverter->SetSize(Size2<kt_int32s>(width, height));
      }
      catch(...)
      {
        m_pData = NULL;

        m_Width = 0;
        m_Height = 0;
        // m_WidthStep = 0;
      }

      Clear();
    }

    /**
     * Checks whether the given coordinates are valid grid indices
     * @param rGrid
     */
    inline kt_bool IsValidGridIndex(const Vector2<kt_int32s>& rGrid) const
    {
      return (math::IsUpTo(rGrid.GetX(), m_Width) && math::IsUpTo(rGrid.GetY(), m_Height));
    }

    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck default value is true
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = false) const
    {
      // std::cout << "rGrid X = " << rGrid.GetX() <<std::endl;
      // std::cout << "rGrid Y = " << rGrid.GetY() <<std::endl;

      if (boundaryCheck == true)
      {
        if (IsValidGridIndex(rGrid) == false)
        {
          std::cout << "calculate index!"<<std::endl;
          std::stringstream error;
          error << "Index " << rGrid << " out of range.  Index must be between [0; "
                << m_Width << ") and [0; " << m_Height << ")";
          throw Exception(error.str());
        }
      }

      // kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_WidthStep);

      kt_int32s index = rGrid.GetX() + (rGrid.GetY() * m_Width);

      if (boundaryCheck == true)
      {
        assert(math::IsUpTo(index, GetDataSize()));
      }

      return index;
    }

    /**
     * Gets the grid coordinate from an index
     * @param index
     * @return grid coordinate
     */
    Vector2<kt_int32s> IndexToGrid(kt_int32s index) const
    {
      Vector2<kt_int32s> grid;

      // grid.SetY(index / m_WidthStep);
      //grid.SetX(index - grid.GetY() * m_WidthStep);
      grid.SetY(index / m_Width);
      grid.SetX(index - grid.GetY() * m_Width);

      return grid;
    }

    /**
     * Converts the point from world coordinates to grid coordinates
     * @param rWorld world coordinate
     * @param flipY
     * @return grid coordinate
     */
    inline Vector2<kt_int32s> WorldToGrid(const Vector2<kt_double>& rWorld, kt_bool flipY = false) const
    {
      return GetCoordinateConverter()->WorldToGrid(rWorld, flipY);
    }

    /**
     * Converts the point from grid coordinates to world coordinates
     * @param rGrid world coordinate
     * @param flipY
     * @return world coordinate
     */
    inline Vector2<kt_double> GridToWorld(const Vector2<kt_int32s>& rGrid, kt_bool flipY = false) const
    {
      return GetCoordinateConverter()->GridToWorld(rGrid, flipY);
    }

    /**
     * Gets pointer to data at given grid coordinate
     * @param rGrid grid coordinate
     * @return grid point
     */
    T* GetDataPointer(const Vector2<kt_int32s>& rGrid)
    {
      kt_int32s index = GridIndex(rGrid, false);
      return m_pData + index;
    }

    /**
     * Gets pointer to data at given grid coordinate
     * @param rGrid grid coordinate
     * @return grid point
     */
    T* GetDataPointer(const Vector2<kt_int32s>& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid, false);
      return m_pData + index;
    }

    /**
     * Gets the width of the grid
     * @return width of the grid
     */
    inline kt_int32s GetWidth() const
    {
      return m_Width;
    };

    /**
     * Gets the height of the grid
     * @return height of the grid
     */
    inline kt_int32s GetHeight() const
    {
      return m_Height;
    };

    /**
     * Get the size as a Size2<kt_int32s>
     * @return size of the grid
     */
    inline const Size2<kt_int32s> GetSize() const
    {
      return Size2<kt_int32s>(m_Width, m_Height);
    }

    /**
     * Gets the width step in bytes
     * @return width step
     */
    // inline kt_int32s GetWidthStep() const
    // {
    //   return m_WidthStep;
    // }

    /**
     * Gets the grid data pointer
     * @return data pointer
     */
    inline T* GetDataPointer()
    {
      return m_pData;
    }

    /**
     * Gets const grid data pointer
     * @return data pointer
     */
    inline T* GetDataPointer() const
    {
      return m_pData;
    }

    /**
     * Gets the allocated grid size in bytes
     * @return data size
     */
    inline kt_int32s GetDataSize() const
    {
      // return m_WidthStep * m_Height;

      return m_Width * m_Height;
    }

    /**
     * Get value at given grid coordinate
     * @param rGrid grid coordinate
     * @return value
     */
    inline T GetValue(const Vector2<kt_int32s>& rGrid) const
    {
      kt_int32s index = GridIndex(rGrid);
      return m_pData[index];
    }

    /**
     * Gets the coordinate converter for this grid
     * @return coordinate converter
     */
    inline CoordinateConverter* GetCoordinateConverter() const
    {
      return m_pCoordinateConverter;
    }

    /**
     * Gets the resolution
     * @return resolution
     */
    inline kt_double GetResolution() const
    {
      return GetCoordinateConverter()->GetResolution();
    }

    /**
     * Gets the grids bounding box
     * @return bounding box
     */
    inline BoundingBox2 GetBoundingBox() const
    {
      return GetCoordinateConverter()->GetBoundingBox();
    }

  protected:
    /**
     * Constructs grid of given size
     * @param width
     * @param height
     */
    Grid(kt_int32s width, kt_int32s height)
      : m_pData(NULL)
      , m_pCoordinateConverter(NULL)
    {
      Resize(width, height);
    }

  private:
    kt_int32s m_Width;       // width of grid
    kt_int32s m_Height;      // height of grid
    // kt_int32s m_WidthStep;   // 8 bit aligned width of grid
    T* m_pData;              // grid data

    // coordinate converter to convert between world coordinates and grid coordinates
    CoordinateConverter* m_pCoordinateConverter;
  };  // Grid

/**
   * An array that can be resized as long as the size
   * does not exceed the initial capacity
   */
  class LookupArray
  {
  public:
    /**
     * Constructs lookup array
     */
    LookupArray()
      : m_pArray(NULL)
      , m_Capacity(0)
      , m_Size(0)
    {
    }

    /**
     * Destructor
     */
    virtual ~LookupArray()
    {
      assert(m_pArray != NULL);

      delete[] m_pArray;
      m_pArray = NULL;
    }

  public:
    /**
     * Clear array
     */
    void Clear()
    {
      memset(m_pArray, 0, sizeof(kt_int32s) * m_Capacity);
    }

    /**
     * Gets size of array
     * @return array size
     */
    kt_int32u GetSize() const
    {
      return m_Size;
    }

    /**
     * Sets size of array (resize if not big enough)
     * @param size
     */
    void SetSize(kt_int32u size)
    {
      assert(size != 0);

      if (size > m_Capacity)
      {
        if (m_pArray != NULL)
        {
          delete [] m_pArray;
        }
        m_Capacity = size;
        m_pArray = new kt_int32s[m_Capacity];
      }

      m_Size = size;
    }

    /**
     * Gets reference to value at given index
     * @param index
     * @return reference to value at index
     */
    inline kt_int32s& operator [] (kt_int32u index)
    {
      assert(index < m_Size);

      return m_pArray[index];
    }

    /**
     * Gets value at given index
     * @param index
     * @return value at index
     */
    inline kt_int32s operator [] (kt_int32u index) const
    {
      assert(index < m_Size);

      return m_pArray[index];
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer()
    {
      return m_pArray;
    }

    /**
     * Gets array pointer
     * @return array pointer
     */
    inline kt_int32s* GetArrayPointer() const
    {
      return m_pArray;
    }

  private:
    kt_int32s* m_pArray;
    kt_int32u m_Capacity;
    kt_int32u m_Size;
  };  // LookupArray

  class CorrelationGrid;
  
  /**
   * Create lookup tables for point readings at varying angles in grid.
   * For each angle, grid indexes are calculated for each range reading.
   * This is to speed up finding best angle/position for a localized range scan
   *
   * Used heavily in mapper and localizer.
   *
   * In the localizer, this is a huge speed up for calculating possible position.  For each particle,
   * a probability is calculated.  The range scan is the same, but all grid indexes at all possible angles are
   * calculated.  So when calculating the particle probability at a specific angle, the index table is used
   * to look up probability in probability grid!
   *
   */
  template<typename T>
  class GridIndexLookup
  {
  public:
    /**
     * Construct a GridIndexLookup with a grid
     * @param pGrid
     */
    GridIndexLookup(Grid<T>* pGrid)
      : m_pGrid(pGrid)
      , m_Capacity(0)
      , m_Size(0)
      , m_ppLookupArray(NULL)
    {
      if(NULL == m_pGrid)
        std::cout<< "m_pGird is a nullptr"<<std::endl;
      // for(int i = 0; i < 6000; ++i)
      //   std::cout<<"m_pGrid data i = " << static_cast<int>(m_pGrid->GetDataPointer()[i])<< std::endl;

      std::cout<<"m_pGrid_X = "<< m_pGrid->GetCoordinateConverter()->GetOffset().GetX()<<std::endl;
    }

    /**
     * Destructor
     */
    virtual ~GridIndexLookup()
    {
      DestroyArrays();
    }

  public:
    /**
     * Gets the lookup array for a particular angle index
     * @param index
     * @return lookup array
     */
    const LookupArray* GetLookupArray(kt_int32u index) const
    {
      assert(math::IsUpTo(index, m_Size));

      return m_ppLookupArray[index];
    }

    /**
     * Get angles
     * @return std::vector<kt_double>& angles
     */
    const std::vector<kt_double>& GetAngles() const
    {
      return m_Angles;
    }

    /**
     * Compute lookup table of the points of the given scan for the given angular space
     * @param pScan the scan
     * @param angleCenter
     * @param angleOffset computes lookup arrays for the angles within this offset around angleStart
     * @param angleResolution how fine a granularity to compute lookup arrays in the angular space
     */
    void ComputeOffsets(LocalizedRangeScan* pScan,
                        kt_double angleCenter,
                        kt_double angleOffset,
                        kt_double angleResolution)
    {
      assert(angleOffset != 0.0);
      assert(angleResolution != 0.0);

      kt_int32u nAngles = static_cast<kt_int32u>(math::Round(angleOffset * 2.0 / angleResolution) + 1);
      std::cout << "Angle size = " << nAngles << std::endl; 
      SetSize(nAngles);

      //////////////////////////////////////////////////////
      // convert points into local coordinates of scan pose

      const PointVectorDouble& rPointReadings = pScan->GetPointReadings();

      // compute transform to scan pose
      Transform transform(pScan->GetSensorPose());

      Pose2Vector localPoints;//position and angle
      const_forEach(PointVectorDouble, &rPointReadings)
      {
        // do inverse transform to get points in local coordinates
        Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
        localPoints.push_back(vec);
      }

      std::cout<<"localpoints size is "<< localPoints.size()<<std::endl;

      //////////////////////////////////////////////////////
      // create lookup array for different angles
      kt_double angle = 0.0;
      kt_double startAngle = angleCenter - angleOffset;

      std::cout<<"startAngle = "<<startAngle<<std::endl;

      for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
      {
        angle = startAngle + angleIndex * angleResolution;
        ComputeOffsets(angleIndex, angle, localPoints, pScan);
      }
      // assert(math::DoubleEqual(angle, angleCenter + angleOffset));
    }

  private:
    /**
     * Compute lookup value of points for given angle
     * @param angleIndex
     * @param angle
     * @param rLocalPoints
     */
    void ComputeOffsets(kt_int32u angleIndex, kt_double angle, const Pose2Vector& rLocalPoints, LocalizedRangeScan* pScan)
    {
      std::cout<<"ComputeOffsets"<<std::endl;

      std::cout<<"number of LocalPoints = "<<static_cast<kt_int32u>(rLocalPoints.size())<<std::endl;

      m_ppLookupArray[angleIndex]->SetSize(static_cast<kt_int32u>(rLocalPoints.size()));
      m_Angles.at(angleIndex) = angle;

      // set up point array by computing relative offsets to points readings
      // when rotated by given angle

      const Vector2<kt_double>& rGridOffset = m_pGrid->GetCoordinateConverter()->GetOffset();

      // for(int i = 0; i < 600; ++i)
      //   std::cout<<"m_pGrid data i = " << m_pGrid->GetDataPointer()[i]<< std::endl;

      std::cout<<"m_pGrid offset X = "<<m_pGrid->GetCoordinateConverter()->GetOffset().GetX()<<std::endl;

      //std::cout<<"m_pGrid offset Y = "<<m_pGrid->GetCoordinateConverter()->GetOffset().GetY()<<std::endl;

      kt_double cosine = cos(angle);
      kt_double sine = sin(angle);

      kt_int32u readingIndex = 0;

      kt_int32s* pAngleIndexPointer = m_ppLookupArray[angleIndex]->GetArrayPointer();

      //kt_double maxRange = pScan->GetLaserRangeFinder()->GetMaximumRange(); //此处没有用到吗？

      const_forEach(Pose2Vector, &rLocalPoints)
      {
        const Vector2<kt_double>& rPosition = iter->GetPosition();

        // std::cout<<"rPosition_X = "<<rPosition.GetX()<<std::endl;
        // std::cout<<"rPosition_Y = "<<rPosition.GetY()<<std::endl;


        if (std::isnan(pScan->GetRangeReadings()[readingIndex]) || std::isinf(pScan->GetRangeReadings()[readingIndex]))
        {
          pAngleIndexPointer[readingIndex] = INVALID_SCAN;
          readingIndex++;
          continue;
        }


        // counterclockwise rotation and that rotation is about the origin (0, 0).
        Vector2<kt_double> offset;
        offset.SetX(cosine * rPosition.GetX() - sine * rPosition.GetY());
        offset.SetY(sine * rPosition.GetX() + cosine * rPosition.GetY());

        // have to compensate for the grid offset when getting the grid index
        Vector2<kt_int32s> gridPoint = m_pGrid->WorldToGrid(offset + rGridOffset);

        // use base GridIndex to ignore ROI
        kt_int32s lookupIndex = m_pGrid->Grid<T>::GridIndex(gridPoint, false);

        pAngleIndexPointer[readingIndex] = lookupIndex;

        // std::cout<<"lookupIndex = "<<lookupIndex<<std::endl;

        readingIndex++;
      }

      std::cout<<"num of readingindex = "<<readingIndex<<std::endl;
      assert(readingIndex == rLocalPoints.size());
    }

    /**
     * Sets size of lookup table (resize if not big enough)
     * @param size
     */
    void SetSize(kt_int32u size)
    {
      assert(size != 0);

      if (size > m_Capacity)
      {
        if (m_ppLookupArray != NULL)
        {
          DestroyArrays();
        }

        m_Capacity = size;
        m_ppLookupArray = new LookupArray*[m_Capacity];
        for (kt_int32u i = 0; i < m_Capacity; i++)
        {
          m_ppLookupArray[i] = new LookupArray();
        }
      }

      m_Size = size;

      m_Angles.resize(size);
    }

    /**
     * Delete the arrays
     */
    void DestroyArrays()
    {
      for (kt_int32u i = 0; i < m_Capacity; i++)
      {
        delete m_ppLookupArray[i];
      }

      delete[] m_ppLookupArray;
      m_ppLookupArray = NULL;
    }

  private:
    Grid<T>* m_pGrid;

    kt_int32u m_Capacity;
    kt_int32u m_Size;

    LookupArray **m_ppLookupArray;

    // for sanity check
    std::vector<kt_double> m_Angles;
  };  // class GridIndexLookup

  class CorrelationGrid : public Grid<kt_int8u>
  {
  public:
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
       if(m_pKernel)
         delete [] m_pKernel;
    }

  public:
    /**
     * Create a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param resolution
     * @param smearDeviation
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(kt_int32s width,
                                       kt_int32s height,
                                       kt_double resolution,
                                       kt_double smearDeviation)
    {
      assert(resolution != 0.0);

      // +1 in case of roundoff
      // kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;

      kt_int32u borderSize = 0;

      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);

      return pGrid;
    }

    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid
     * @param boundaryCheck
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Vector2<kt_int32s>& rGrid, kt_bool boundaryCheck = false) const
    {
      kt_int32s x = rGrid.GetX() + m_Roi.GetX();
      kt_int32s y = rGrid.GetY() + m_Roi.GetY();

      std::cout<< "m_Roi X = " << m_Roi.GetX() <<std::endl;
      std::cout<< "m_Roi Y = " << m_Roi.GetY() <<std::endl;

      return Grid<kt_int8u>::GridIndex(Vector2<kt_int32s>(x, y), boundaryCheck);
    }

    /**
     * Get the Region Of Interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2<kt_int32s>& GetROI() const
    {
      return m_Roi;
    }

    /**
     * Sets the Region Of Interest (ROI)
     * @param roi
     */
    inline void SetROI(const Rectangle2<kt_int32s>& roi)
    {
      m_Roi = roi;
    }

    /**
     * Smear cell if the cell at the given point is marked as "occupied"
     * @param rGridPoint
     */
    inline void SmearPoint(const kt_int32s& index)
    {
      assert(m_pKernel != NULL);

      if (GetDataPointer()[index] != GridStates_Occupied)
      {
        return;
      }

      kt_int32s halfKernel = m_KernelSize / 2;

      // apply kernel
      for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
      {
        kt_int32s offset = index + j * GetWidth();

        if(offset < halfKernel || offset > GetWidth()*GetHeight()-halfKernel)
        {
          continue;
        }

        kt_int8u* pGridAdr = GetDataPointer() + offset;

        kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);

        // if a point is on the edge of the grid, there is no problem
        // with running over the edge of allowable memory, because
        // the grid has margins to compensate for the kernel size
        for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
        {
          kt_int32s kernelArrayIndex = i + kernelConstant;

          kt_int8u kernelValue = m_pKernel[kernelArrayIndex];
          if (kernelValue > pGridAdr[i])
          {
            // kernel value is greater, so set it to kernel value
            pGridAdr[i] = kernelValue;
          }
        }
      }
    }

  protected:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width
     * @param height
     * @param borderSize
     * @param resolution
     * @param smearDeviation
     */
    /*               Grid构造示意图
    ---------------------------------------------
    *                                            *
    *                                            *
    *                                            *
    *                                            *
    *                      width                 *
    *               -----------------            *
    *               *               *            *
    *               *               * height     *
    *               *               *            *
    *  boardersize  *               *            *
    * <------------>-----------------            *
    *                                            *
    *                                            *
    *                                            *
    *                                            *
    *--------------------------------------------*
     */
    CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize,
                    kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
      , m_SmearDeviation(smearDeviation)
    {
      GetCoordinateConverter()->SetScale(1.0 / resolution);

      // setup region of interest
       m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);

      // calculate kernel
       CalculateKernel();
    }

    /**
     * Sets up the kernel for grid smearing.
     */
    virtual void CalculateKernel()
    {
      kt_double resolution = GetResolution();

      assert(resolution != 0.0);
      assert(m_SmearDeviation != 0.0);

      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
      const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;

      // check if given value too small or too big
      if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        std::stringstream error;
        error << "Mapper Error:  Smear deviation too small:  Must be between "
              << MIN_SMEAR_DISTANCE_DEVIATION
              << " and "
              << MAX_SMEAR_DISTANCE_DEVIATION;
        throw std::runtime_error(error.str());
      }

      // NOTE:  Currently assumes a two-dimensional kernel

      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;

      // allocate kernel
      m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
      if (m_pKernel == NULL)
      {
        throw std::runtime_error("Unable to allocate memory for kernel!");
      }

      // calculate kernel
      kt_int32s halfKernel = m_KernelSize / 2;
      for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
      {
        for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
        {
#ifdef WIN32
          kt_double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
          kt_double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
          kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));

          kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
          assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));

          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
        }
      }
    }

    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param smearDeviation
     * @param resolution
     * @return kernel half-size based on the parameters
     */
    static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
    {
      assert(resolution != 0.0);

      return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
    }

  private:
    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    kt_double m_SmearDeviation;

    // Size of one side of the kernel
     kt_int32s m_KernelSize;

    // Cached kernel for smearing
     kt_int8u* m_pKernel;

    // region of interest
     Rectangle2<kt_int32s> m_Roi;
  };  // CorrelationGrid

}//namespace karto

#endif