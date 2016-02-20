#pragma once

#include "mathclass.h"
/**
 * abstract class of interpolation classes
 *  Usage : initialize �� calcWeight �Ǵ� calcWeightNormalized�� call�Ѵ�. (parameter�� normalize�� parameter���� ���ο� ����)
 * \author taesoo
 *
 *
 */
class InterpolationNormalize
{
public:
	InterpolationNormalize(void);
	virtual ~InterpolationNormalize(void);

	/**
	 * �̰� call�ϸ�, �־��� min, max�� 0, 1�� �ǵ��� normalize�Ѵ�.
	 * ��, normalize���� �������� min�� 0, max�� 1�� �ش�. 
	 * ���Լ��� call���� ������, sourcesample�� min, max�� �ڵ����� �����Ͽ� ����Ѵ�.
	 */
	void setMinMax(const vectorn& min, const vectorn& max) {m_cNormalizingInterval.assign(min, max);}
	/**
	 * using input query, store all normalize samples into m_aNormalizedSamples
	 * do additional initialization in the derived class
	 * \param sourceSamples input query.
	 */
	virtual void initialize(const matrixn& sourceSamples);

	/// initialize �Ŀ� ����� �� �ִ� �Լ�. 
	int dimension()	const						{ return m_aNormalizedSamples.cols();}
	int numSample()	const						{ return m_aNormalizedSamples.rows();}	
	vectornView sample(int i) const			{ return m_aNormalizedSamples.row(i);}
    const matrixn& allSample() const			{ return m_aNormalizedSamples;}



	/// parameter�� normalized�Ȱ��
	virtual void calcWeightNormalized(const vectorn& normalizedParameter, intvectorn& index, vectorn& weight)=0;
	/// parameter�� normalized���� ���� ���
	void calcWeight(const vectorn& parameter, intvectorn& index, vectorn& weight);

	void normalize(const vectorn& unnormalizedSample, vectorn& normalizedSample) const;
	void unnormalize(const vectorn& normalizedSample, vectorn& unnormalizedSample) const;

	virtual m_real score(const vectorn& vec) const;

	/// normalize�� data���� min
	const vectorn& minimum() const					{ return m_vResultMin;}
	/// normalize�� data���� max
	const vectorn& maximum() const					{ return m_vResultMax;}
	/// normalize�� date���� mean
	const vectorn& mean() const						{ return m_vMean;}
	/// normalize�� date���� stddev
	const vectorn& stddev()	const					{ return m_vStddev;}

	// normalize���� ���� ���� space���� ���� return
	m_real minimum(int dim) const					{ return m_cNormalizingInterval[dim].interpolate(m_vResultMin[dim]); }
	m_real maximum(int dim) const					{ return m_cNormalizingInterval[dim].interpolate(m_vResultMax[dim]); }

	/// standard normal distribution ������ z���� �Է����� �޴´�. �� z�� 0�̸�, mean�� ��µȴ�.
	void calcNormalizedParameter(const vectorn& aZ, vectorn& normalizedParameter) const;
	
	/// 1�������� interpolation�� �����Ѵ�. ��� weight�ñ׳��� testID_weight.bmp�� ����ȴ�.
	void Test(const vectorn& signal1D, const char* testID, int TestResolution=100);

	const intervalN& normalizingInterval() const	{ return m_cNormalizingInterval;}

protected:
	matrixn m_aNormalizedSamples;
	intervalN m_cNormalizingInterval;

	// m_vMin, m_vMax�� [0,1]������ scale������ ���� data���� min, max����.
	vectorn m_vResultMin;	
	vectorn m_vResultMax;
	vectorn m_vMean;
	vectorn m_vStddev;
};


/// Find closest sample
class NoInterpolation : public InterpolationNormalize
{
public:
	NoInterpolation(Metric* pMetric=NULL){ if(pMetric) m_pMetric=pMetric; else m_pMetric=new L2Metric();}
	virtual ~NoInterpolation(){ delete m_pMetric; }
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	Metric* m_pMetric;
};

/// Find k closest samples. This is fast version of IDW interpolation.
class KNearestInterpolation : public InterpolationNormalize
{
public:
	KNearestInterpolation (Metric* pMetric, int k=4, float power=2.f);
	virtual ~KNearestInterpolation (){}
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
	/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
	int m_nK;
	float m_fK;

	Metric* m_pMetric;	
};
/*
class KNearestBoxerInterpolation : public Interpolation
{
public:
	KNearestBoxerInterpolation (int k=4, float power=2.f);
	virtual ~KNearestBoxerInterpolation (){}
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);

	/// weight= 1/distance^m_fK - 1/maxDistanceAmongK^m_fK
	int m_nK;
	float m_fK;

	CBoxerMetric metric;
};*/

/// Inverse distance weighted interpolation by taesoo. This is the best interpolation class in almost every case.
class IDWInterpolation : public InterpolationNormalize
{
public:
	IDWInterpolation(){m_fK=2.f;}
	virtual ~IDWInterpolation(){}
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
	
	/// weight= 1/distance^m_fK (default:2)
	float m_fK;
};

#include "LCBInterpolator.h"

/// Linear Cardinal Basis interpolation
class LCBInterpolation : public InterpolationNormalize
{
public:
	LCBInterpolation(){}
	virtual ~LCBInterpolation(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
private:
	LCBInterpolator m_interpolator;
};

/// Linear Cardinal Basis interpolation2 by taesoo
class LCBInterpolation2 : public InterpolationNormalize
{
public:
	LCBInterpolation2(){}
	virtual ~LCBInterpolation2(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
protected:	
	matrixn m_aLinearCoef;
	matrixn m_aRadialCoef;
	vectorn m_aDilationFactor;
};

/// Radial Basis interpolation by taesoo
class RBInterpolation : public InterpolationNormalize
{
public:
	RBInterpolation(){}
	virtual ~RBInterpolation(){}
	
	virtual void initialize(const matrixn& sourceSamples);
	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight);
private:	
	matrixn m_aRadialCoef;
	vectorn m_aDilationFactor;
};

class TestInterpolation: public InterpolationNormalize
{
public:
	TestInterpolation(){}
	TestInterpolation(const intvectorn& index, const vectorn& weight):m_index(index), m_weight(weight){}
    virtual ~TestInterpolation(){}

	void setTestVariable(const intvectorn& index, const vectorn& weight)
	{
		m_index=index;
		m_weight=weight;
	}

	virtual void calcWeightNormalized(const vectorn& parameter, intvectorn& index, vectorn& weight)
	{
		index=m_index;
		weight=m_weight;
	}
	intvectorn m_index;
	vectorn m_weight;
};