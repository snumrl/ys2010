#pragma once

#include "../utility/tarray.h"

class matrixn;
class intvectorn;
class Metric;
/// wrapper class of CFuzzyCluster
/** ���߿��� �ʿ��� ��ɸ� ����� CFuzzyCluster�� ������ų ��ȹ*/
class Cluster
{
public:
	Cluster(void);
	~Cluster(void);

	void ChangeDistanceMetric(const Metric& Metric);

	// cluster methods
	void DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_thr, int& cluster_n, intvectorn& group_index, matrixn& aCenter, matrixn& refPattern);
	void ShinCluster(const matrixn &aInputVec, m_real inner_thr, int &GroupNum, intvectorn& aGroupIndex, matrixn& aCenter);
	void KMeanCluster(const matrixn& aInputvec, int cluster_n, intvectorn& groupIndex, matrixn& centers, const matrixn& input_centers);
private:

	Metric* m_pMetric;
	void CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat);
	//pGroupIndex�� aVectorGroup���·� �ٲ۴�. �� ���° �׷쿡 ���� �ִ��� ����.
	static void MakeGroupIndex(CTArray<intvectorn>& aVectorGroup, int* pGroupIndex);
	static void CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intvectorn>& aVectorGroup);
	m_real CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intvectorn>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength);
	m_real CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength);
	static bool CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex);
};


/// Cluastering class�� interface�� ���Ͻ�Ű�� ���� ����.
struct Clustering
{
	Clustering() {}	
	~Clustering() {}
	/// aInputVec : array of input vectors. Each vector can have different length from others.
	virtual void cluster(const TArray<vectorn>& aInputVec) {}

	// query outputs
	int numGrp()				{ return m_nNumGroup;}
	int groupIndex(int elt)		{ return m_aGroupIndex[elt];}
	intvectorn& groupIndex()	{ return m_aGroupIndex;}

	// test routines
	void test(const char* fn);
	virtual void plot(matrixn const& samples,const char* fn);
	static void test();
protected:
	//	all outputs are save into members
	int m_nNumGroup;
	intvectorn m_aGroupIndex;
};

struct KMeanCluster : public Clustering
{
	KMeanCluster(int numCluster){m_nNumGroup=numCluster;}
	~KMeanCluster(){}

	virtual void cluster(const TArray<vectorn>& aInputVec);
	
};

namespace Clust
{
	class ClassSig;
};
struct GMMCluster: public Clustering
{
	enum Option { FULL, DIAG, SPHR};

	// Determine the optimal number of clusters
	GMMCluster(int init_num_of_subclasses, Option option1=FULL);
	// Number of clusters are predetermined.
	GMMCluster(Option option1, int numCluster);

	virtual void cluster(const TArray<vectorn>& aInputVec);
	virtual void plot(matrixn const& samples,const char* fn);

	Option mOption;
	int mDesiredNumCluster;
	Clust::ClassSig* mSig;
};

struct ExactCluster : public Clustering
{
	ExactCluster() {}
	~ExactCluster()	{}

	virtual void cluster(const TArray<vectorn>& aInputVec);
};

struct FuzzyCluster : public Clustering
{
	/// use either thr or numCluster to adjust clustering
	FuzzyCluster(m_real thr=FLT_MAX, int numCluster=1):m_fRadii(thr), m_numCluster(numCluster) {}
	~FuzzyCluster() {}
	virtual void cluster(const TArray<vectorn>& aInputVec);
	int findNearestGroup(const vectorn& inputVec);
	m_real m_fRadii;
	matrixn m_aCenter;
	int m_numCluster;
};

struct AggloCluster : public Clustering
{
	enum { MIN_LINKAGE , AVERAGE_LINKAGE , MAX_LINKAGE };
	/// use either thr or numCluster to adjust clustering
	AggloCluster(const Metric& metric, int eLinkage, m_real thr=FLT_MAX, int numCluster=1);
	~AggloCluster();
	virtual void cluster(const TArray<vectorn>& aInputVec);
	int m_eLinkage;
	m_real m_fThr;
	Metric* m_pMetric;
	matrixn m_distMat;
	matrixn m_interDist;
	vectorn m_innerDist;	

	struct Factory : public TFactory<Clustering>
	{
		Factory(Metric* pMetric, int eLinkage, m_real fThr=FLT_MAX, int numCluster=1):m_pMetric(pMetric), m_eLinkage(eLinkage), m_fThr(fThr), m_numCluster(numCluster) {} 
		virtual~Factory(){ delete m_pMetric; }
		virtual Clustering* create(int index)	{ return new AggloCluster(*m_pMetric, m_eLinkage, m_fThr, m_numCluster);}
		Metric* m_pMetric;
		int m_eLinkage;
		m_real m_fThr;
		int m_numCluster;
	};
};

struct TwoPhaseClustering : public Clustering
{
	/// pC1, pC2F�� ���� �����ؼ� �־��ٰ�. TwoPhaseClustering�� ������ �ڵ����� �����ȴ�.
	TwoPhaseClustering(Clustering* pC1, TFactory<Clustering>* pC2F, int numC1Elt);
	~TwoPhaseClustering();

	/// aInputVec�� ���Ұ����� �ι迡 �ش��ϴ� array. ���� ���� C1�� ���� feature�� �����ϰ�, ���� ���� C2���� ���� feature�� �����Ѵ�.
	virtual void cluster(const TArray<vectorn>& aInputVec);
	
	int m_numC1Elt;
	Clustering* m_pC1;
	TArray<Clustering> m_aC2;
	TArray<intvectorn> m_aTargetIndex;	//!< m_aC2�� i��° cluster�� j��° ������ aInputVec�ε����� m_aTargetIndex[i][j]
	intvectorn m_aGlobalGroupStartIndex; 
};

