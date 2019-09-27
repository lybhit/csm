#ifndef PARAMETER_H
#define PARAMETER_H

#include "Types.h"

namespace karto
{

template <class T>
class Parameter
{
	public:
	void SetValue(T val)
	{
		m_data = val;
	}
	
	T GetValue()
	{
		return m_data;
	}
	
	private:
	T m_data;

};

class Mapper
{
	public:
    Mapper():m_pCorrelationSearchSpaceDimension(NULL),
             m_pCorrelationSearchSpaceResolution(NULL),
             m_pCorrelationSearchSpaceSmearDeviation(NULL),
             m_pDistanceVariancePenalty(NULL),
             m_pAngleVariancePenalty(NULL),
             m_pFineSearchAngleOffset(NULL),
             m_pCoarseSearchAngleOffset(NULL),
             m_pMinimumAnglePenalty(NULL),
             m_pMinimumDistancePenalty(NULL),
             m_pUseResponseExpansion(NULL)
             {}




	Parameter<kt_double>* m_pCorrelationSearchSpaceDimension;
	Parameter<kt_double>* m_pCorrelationSearchSpaceResolution;
    Parameter<kt_double>* m_pCorrelationSearchSpaceSmearDeviation;

	Parameter<kt_double>* m_pDistanceVariancePenalty;
	Parameter<kt_double>* m_pAngleVariancePenalty;
	Parameter<kt_double>* m_pFineSearchAngleOffset;
	Parameter<kt_double>* m_pCoarseSearchAngleOffset;
	Parameter<kt_double>* m_pCoarseAngleResolution;
	Parameter<kt_double>* m_pMinimumAnglePenalty;
	Parameter<kt_double>* m_pMinimumDistancePenalty;
	Parameter<kt_bool>* m_pUseResponseExpansion;
	
	public:
	~Mapper()
	{
		if(m_pCorrelationSearchSpaceDimension)
			delete m_pCorrelationSearchSpaceDimension;
		if(m_pCorrelationSearchSpaceResolution)
			delete m_pCorrelationSearchSpaceResolution;
		if(m_pCorrelationSearchSpaceSmearDeviation)
			delete m_pCorrelationSearchSpaceSmearDeviation;

		if(m_pCoarseAngleResolution)
			delete m_pCoarseAngleResolution;
		if(m_pDistanceVariancePenalty)
			delete m_pDistanceVariancePenalty;
		if(m_pAngleVariancePenalty)
			delete m_pAngleVariancePenalty;
		if(m_pFineSearchAngleOffset)
			delete m_pFineSearchAngleOffset;
		if(m_pCoarseSearchAngleOffset)
			delete m_pCoarseSearchAngleOffset;
		if(m_pCoarseAngleResolution)
			delete m_pCoarseAngleResolution;
		if(m_pMinimumAnglePenalty)
			delete m_pMinimumAnglePenalty;
		if(m_pMinimumDistancePenalty)
			delete m_pMinimumDistancePenalty;
		if(m_pUseResponseExpansion)
			delete m_pUseResponseExpansion;
	}
	
	
	public:
	
	// Setting Scan Matcher Parameters from the Parameter Server

	void setParamCorrelationSearchSpaceDimension(kt_double val)
	{
		m_pCorrelationSearchSpaceDimension = new Parameter<kt_double>();
		m_pCorrelationSearchSpaceDimension->SetValue(val);
	}

	void setParamCorrelationSearchSpaceResolution(kt_double val)
	{
		m_pCorrelationSearchSpaceResolution = new Parameter<kt_double>();
		m_pCorrelationSearchSpaceResolution->SetValue(val);
	}

	void setParamCorrelationSearchSpaceSmearDeviation(kt_double val)
	{
		m_pCorrelationSearchSpaceSmearDeviation = new Parameter<kt_double>();
		m_pCorrelationSearchSpaceSmearDeviation->SetValue(val);
	}


    void setParamDistanceVariancePenalty(kt_double val)
	{
		m_pDistanceVariancePenalty = new Parameter<kt_double>();
		m_pDistanceVariancePenalty->SetValue(val);
	}


    void setParamAngleVariancePenalty(kt_double val)
	{
		m_pAngleVariancePenalty = new Parameter<kt_double>();
		m_pAngleVariancePenalty->SetValue(val);
	}


    void setParamFineSearchAngleOffset(kt_double val)
	{
		m_pFineSearchAngleOffset = new Parameter<kt_double>();
		m_pFineSearchAngleOffset->SetValue(val);
	}


    void setParamCoarseSearchAngleOffset(kt_double val)
	{
		m_pCoarseSearchAngleOffset = new Parameter<kt_double>();
		m_pCoarseSearchAngleOffset->SetValue(val);
	}


    void setParamCoarseAngleResolution(kt_double val)
	{
		m_pCoarseAngleResolution = new Parameter<kt_double>();
		m_pCoarseAngleResolution->SetValue(val);
	}


    void setParamMinimumAnglePenalty(kt_double val)
	{
		m_pMinimumAnglePenalty = new Parameter<kt_double>();
		m_pMinimumAnglePenalty->SetValue(val);
	}


    void setParamMinimumDistancePenalty(kt_double val)
	{
		m_pMinimumDistancePenalty = new Parameter<kt_double>();
		m_pMinimumDistancePenalty->SetValue(val);
	}


    void setParamUseResponseExpansion(bool val)
	{
		m_pUseResponseExpansion = new Parameter<bool>();
		m_pUseResponseExpansion->SetValue(val);
	}
};
}//namespace karto

#endif
