#pragma once

#include "interpolation.h"
class Function
{
public:
	Function():mIsLearned(false){}
	virtual ~Function(){}

	// Test any function class using simple one or two dimensional samples.
	void test(const char* id);
	// numSample<20
	void test2(const char* id, int numSample);
	// plot�� �غ���, ���� ���ؼ� learning�Ѵ�. 
	void plot(const matrixn& sources, const matrixn& targets, const char* identifier) ;
	virtual void learn(const matrixn& sources, const matrixn& targets)	{mIsLearned=true; mDimSource=sources.cols();mDimTarget=targets.cols();}
	// PCA�� GPLVM���� target space�� hidden�� ��� ���Լ��� ����� ����Ѵ�. 
	virtual void learn(const matrixn& sources)	{ mIsLearned=true; mDimSource=sources.cols(); mDimTarget=-1;}
	virtual bool isLearned() const { return mIsLearned;}
	int dimDomain() const		{ return mDimSource;}
	int dimRange() const		{ return mDimTarget;}

	TArray<TString> mSourceDimName;
	TArray<TString> mTargetDimName;
	virtual void mapping(const vectorn& source, vectorn& target) const =0;
	void mapping(const matrixn& sources, matrixn& targets) const;

	virtual m_real logProb(const vectorn& source) const							{Msg::error("logProb"); return 0.0;}

	// �Ϲ������� function class�� �ϳ��� source�� ���ؼ� distribution�� ����س�����,
	// �پ��� target�� ���ؼ� ������ Ȯ���� ����� �� �ִ�. �� ��� �ӵ��� ����ȭ�ϱ� ���� ���� �и��س��Ҵ�.
	virtual void prepare(const vectorn& source) const							{ Msg::error("prepare"); }
	virtual m_real logProbPrepared(const vectorn& target) const					{ Msg::error("logprobprepaired");return 0.0;}

	// P(target|source)
	virtual m_real logProb(const vectorn& source, const vectorn& target) const		
	{
		prepare(source);
		return logProbPrepared(target);
	}

	virtual void variance(const vectorn& source, vectorn& var) const			{var.setSize(0);}

	// following functions are deprecated
	// some function classes provide the probability estimation
	
	// max_{Y* in Y} P(Y*|source)
	//virtual m_real logProbMax(const vectorn& source) const						{Msg::error("logProbMax"); return 0.0;}
	// �Է����� ���� sample�� ���� Ȯ���� �������� �����Ѵ�. 
	//virtual void mappingNearest(const vectorn& source, vectorn& target) const { Msg::error("MappingNear");}

	//	virtual m_real prob(const vectorn& source) const							{Msg::error("prob"); return 1.0;}
//	virtual m_real prob(const vectorn& source, const vectorn& target) const		{Msg::error("prob2"); return 1.0;}

protected:
	int mDimSource;
	int mDimTarget;	
	bool mIsLearned;
};

class BijectiveFunction : public Function
{
public:
	BijectiveFunction(void);
	virtual ~BijectiveFunction(void);

	virtual void inverseMapping(const vectorn& target, vectorn& source) const =0;
};

template <class OtherFunction>
class TClampFunction : public OtherFunction
{
public:
	TClampFunction(){}
	virtual ~TClampFunction(){}

	virtual void learn(const matrixn& sources, const matrixn& targets)
	{
		mRange.calcRange(sources);
		OtherFunction::learn(sources, targets);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn clampedSource;
		mRange.project(source, clampedSource);
		OtherFunction::mapping(clampedSource, target);
	}
	intervalN mRange;
};


class UniversalFunction : public Function
{
public:
	// parameters will be deleted in the destructor.
	UniversalFunction (Function* TrendEstimator, Function* ResidualEstimator)
		:mTrendEstimator(TrendEstimator), mResidualEstimator(ResidualEstimator) {}
	virtual ~UniversalFunction() { delete mTrendEstimator; delete mResidualEstimator;}

	virtual void learn(const matrixn & sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;

private:
	Function* mTrendEstimator;
	Function* mResidualEstimator;
	matrixn mTrends;
	matrixn mResiduals;
};

template <class TrendEstimator,class ResidualEstimator>
class TUniversalFunction : public Function
{
public:
	// parameters will be deleted in the destructor.
	TUniversalFunction ():mTrendEstimator(), mResidualEstimator() {}
	virtual ~TUniversalFunction() {}

	virtual void learn(const matrixn & sources, const matrixn& targets)
	{
		__super::learn(sources, targets);

		mTrendEstimator.learn(sources, targets);

		mTrends.setSameSize(targets);

		for(int i=0; i<targets.rows(); i++)
			mTrendEstimator.mapping(sources.row(i), mTrends.row(i));

		mResiduals.subtract(targets, mTrends);

		mResidualEstimator.learn(sources, mResiduals);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn trend;
		mTrendEstimator.mapping(source, trend);
		mResidualEstimator.mapping(source, target);
		target+=trend;	
	}

private:
	TrendEstimator mTrendEstimator;
	ResidualEstimator mResidualEstimator;
	matrixn mTrends;
	matrixn mResiduals;
};

class NormalizedFunction : public Function
{
public:
	NormalizedFunction (Function* other):mFunction(other){}
	virtual ~NormalizedFunction () {delete mFunction;}

	/**
	* �̰� call�ϸ�, �־��� min, max�� 0, 1�� �ǵ��� normalize�Ѵ�.
	* ��, normalize���� �������� min�� 0, max�� 1�� �ش�. 
	* ���Լ��� call���� ������, sourcesample�� min, max�� �ڵ����� �����Ͽ� ����Ѵ�.
	*/
	void setMinMax(const vectorn& min, const vectorn& max) {m_cNormalizingInterval.assign(min, max);}

	virtual void learn(const matrixn & sources, const matrixn& targets)
	{
		__super::learn(sources, targets);
		_initialize(sources);
		mFunction->learn(m_aNormalizedSamples, targets);
	}

	virtual void mapping(const vectorn& source, vectorn& target) const
	{
		vectorn normalizedSource;
		m_cNormalizingInterval.uninterpolate(source, normalizedSource);
		mFunction->mapping(normalizedSource, target);
	}

	/// _initialize �Ŀ� ����� �� �ִ� �Լ�. 
	int dimension()	const						{ return m_aNormalizedSamples.cols();}
	int numSample()	const						{ return m_aNormalizedSamples.rows();}	
	vectornView sample(int i) const			{ return m_aNormalizedSamples.row(i);}
	const matrixn& allSample() const			{ return m_aNormalizedSamples;}

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

protected:
	void _initialize(const matrixn & sources);
	matrixn m_aNormalizedSamples;
	intervalN m_cNormalizingInterval;

	// m_vMin, m_vMax�� [0,1]������ scale������ ���� data���� min, max����.
	vectorn m_vResultMin;	
	vectorn m_vResultMax;
	vectorn m_vMean;
	vectorn m_vStddev;

	Function* mFunction;
};

class LinearFunction : public BijectiveFunction
{
public:
	LinearFunction(){}
	virtual ~LinearFunction(){}

	void learn(const matrixn& sources, const matrixn& targets);

	virtual void mapping(const vectorn& source, vectorn& target) const;
	virtual void inverseMapping(const vectorn& target, vectorn& source) const;

	matrixn mLinearCoef;
	matrixn mInvLinearCoef;
};

class InterpolationFunction : public Function
{
public: 
	InterpolationFunction (InterpolationNormalize* pint):mInterpolation(pint){}
	virtual ~InterpolationFunction (){delete mInterpolation;}

	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;

	InterpolationNormalize* mInterpolation;
	matrixn mTarget;
};

class NonlinearFunctionIDW : public Function
{
public:
	NonlinearFunctionIDW (int k=30, float power=2.f):Function(),mInterpolation(new L2Metric(), k, power){}
	NonlinearFunctionIDW (Metric* pMetric, int k=30, float power=2.f):Function(),mInterpolation(pMetric, k, power){}
	virtual ~NonlinearFunctionIDW (){}

	void changeMetric(Metric* pMetric){mInterpolation.m_pMetric=pMetric;}
	virtual void learn(const matrixn& sources, const matrixn& targets);
	virtual void mapping(const vectorn& source, vectorn& target) const;
	void mapping(const vectorn& source, vectorn& target, vectorn& weights) const;

	KNearestInterpolation mInterpolation;
	matrixn mTarget;
};

class tpros;
class TPros :public Function
{
public:
	TPros (const char* option="tpros spec/tspec");
	virtual ~TPros ();
	virtual void learn(const matrixn& sources, const matrixn& targets)	;
	virtual void mapping(const vectorn& source, vectorn& target) const ;
	
	m_real bestProb(const vectorn& source) const;
	
	virtual m_real prob(const vectorn& source) const	{		return bestProb(source);	}
	virtual m_real prob(const vectorn& source, const vectorn& target) const;
	
	void pack(BinaryFile& file);
	void unpack(BinaryFile& file);

	TString mOption;
	std::vector< tpros* > pGP;
	int mNumTrainingData;
	virtual bool isLearned() const { return pGP.size()!=0; }
};

class TProsFromFile:public TPros
{
public:
	TProsFromFile(const char* option="tpros spec/tspec"):TPros(option){}
	virtual ~TProsFromFile(){}
	virtual void learn(const matrixn& sources, const matrixn& targets);	
	static bool mbSave;
	static BinaryFile* mpFile;
	static void testPackUnpack();
};

#include "pca.h"

class FunctionUtil
{
public:
	FunctionUtil(){}
	virtual ~FunctionUtil(){}

	void addSample(const vectorn& source, const vectorn& target)
	{
		mSource.pushBack(source);
		mTarget.pushBack(target);
	}

	void learn(Function& func)	
	{
		if(mName.length()) _dump(mSource, mTarget);
		func.learn(mSource, mTarget);
	}
	void learnInSubspace(Function& func, PCA& pca)	
	{
		pca.Projection(mSource, mReducedSource);
		if(mName.length()) _dump(mReducedSource, mTarget);
		func.learn(mReducedSource, mTarget);
	}

	void learnInSubspace(Function& func, PCA& pca, PCA& tpca)	
	{
		pca.Projection(mSource, mReducedSource);
		pca.Projection(mTarget, mReducedTarget);
		
		if(mName.length()) _dump(mReducedSource, mReducedTarget);
		func.learn(mReducedSource, mReducedTarget);
	}
	
	void plot(Function& func, const char* id) {func.plot(mSource, mTarget, id);}

	int numSample() const	{ return mSource.rows();}


	// utility funcitons.
	void enableDump(const char* filename)	{	mName=filename;	}
	void learnFromDumpfile(const char* filename, Function& func, int outdim=-1);
	matrixn const& source() const		{return mSource;}
	matrixn const& target() const		{return mTarget;}
private:
	void _dump(matrixn const& source, matrixn const& target);
	TString mName;
	matrixn mSource, mTarget;
	matrixn mReducedSource;
	matrixn mReducedTarget;
};