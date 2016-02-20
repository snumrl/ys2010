#pragma once

#include <deque>
#include <vector>
#include "../utility/TArray.h"
typedef std::vector<int> intVector;

#include "mathclass.h"

class CFuzzyCluster
{
public:
	CFuzzyCluster(void);
	~CFuzzyCluster(void);

	static void Subtractive(matrixn& x, int n, std::deque<vectorn>& centers);
	static void Subtractive(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers);
	//�� ������ ���� out�̰� radii�� vectorn type�� x �� �� component�� ���� influence range (������ ���� threshold��� ���� --; )
	// n�� ��� data ����
	static void Subtractive(matrixn& x, int n, m_real* radii, std::deque<vectorn>& centers);
	static void Subtractive2(vectorn* x, int n, m_real radii, std::deque<vectorn>& centers);
	static void Subtractive2(vectorn* x, int n, m_real* radii, std::deque<vectorn>& centers);
private:
	static m_real min_value(vectorn& x);
	static m_real max_value(vectorn& x);
	static m_real min_value(m_real* x, int n, int& index);
	static m_real max_value(m_real* x, int n, int& index);
	static void min_value(vectorn* x, int n, vectorn& minX);
	static void max_value(vectorn* x, int n, vectorn& maxX);
	static m_real Distance(vectorn &v1, vectorn &v2, vectorn &factor);
	
public:
	static void Fcm(matrixn& x, int n, int cluster_n, std::deque<vectorn>& centers, int* group_index);
	static void Fcm(matrixn& x, int n, std::deque<vectorn>& centers, int* group_index);
	static void FcmRadii(matrixn& x, int n, m_real radii, std::deque<vectorn>& centers, int* group_index);
	static void InitFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers);
	static m_real StepFcm(matrixn& x, vectorn* U, int cluster_n, int expo, std::deque<vectorn>& centers);
	
	// Followings are taesoo favorite interface version.

	static void Subtractive(const matrixn& inputvectors, m_real radii, std::deque<vectorn>& centers);
	static void FcmRadii(const matrixn& inputvectors,  m_real radii, matrixn& aCenter, int* group_index);
	// binary search�� ���ϴ� �Ÿ��� thr���� �ȿ��� ã�´�. 
	//�� desired_ncenter+-thr������ ���� ������ ���� �����. �� max_iteration=20�̴�.��������� ���� radii�� return�Ѵ�.
	static m_real FcmKCenter(const matrixn& aInputVec,  int numCluster, int thr, matrixn& aCenter, int* group_index, int max_iter=20);
	
	// bUseInputCenters�� true�� ��� �Էµ� centers�� initial center�� ��� �ִٰ� ����
	static void KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, bool bUseInitialCenters=false);
	static void KMeanCluster(const matrixn& inputvectors, int cluster_n, matrixn& centers, int *group_index, const matrixn& input_centers);
	static void FcmKMeanCluster(const matrixn &inputvectors, int cluster_n, matrixn& centers, int *pGroupIndex);
	// kmean clustering�� empty cluster�� ����� ��� �Ʒ� �Լ��� �����ش�.
	static bool CheckNonEmpty(const matrixn& aInputVec, int &numCluster, matrixn& aCenter, int *pGroupIndex);

	static void GenerateOptimumCluster(const matrixn &aInputVec, int &GroupNum, int *pGroupIndex, m_real thr, matrixn& aCenter);

	//pGroupIndex�� aVectorGroup���·� �ٲ۴�. �� ���° �׷쿡 ���� �ִ��� ����.
	static void CalcVectorGroup(int numInput, int numCluster, int* pGroupIndex, CTArray<intVector>& aVectorGroup);
	static m_real CalcInnerClusterScore(const matrixn& aInputVec, const matrixn& aCenter, const CTArray<intVector>& aVectorGroup, int& maxGroupIndex, int& maxEltIndex, m_real& maxLength);
	static m_real CalcInterClusterScore(const matrixn& aInputVec, const matrixn& aCenter, int numCluster, int& minGroup1, int& minGroup2, m_real& minLength);

	static void CalcDTWDistanceMatrix(const matrixn& aInputVec, int numRow, int numColumn, matrixn& distMat);

	// Ŭ������ ������ cluster_n���� �ɶ����� ����
	static void AggloCluster(const matrixn& distMat, int cluster_n, int* group_index);
	// inner cluster distance�� inner_cluster_thr�� ���� �ʴ� �ּ� Ŭ������.
	static void AggloCluster(const matrixn& distMat, m_real inner_cluster_thr, int& cluster_n, int* group_index);
	static void AggloClusterCore(const matrixn& distMat, matrixn& interDist, vectorn& innerDist, m_real inner_cluster_thr, int& cluster_n, int* group_index, int eLinkage=0);
	
	static void DTWAggloCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL);
	static void DTWKmeanCluster(const matrixn &aInputVec, int numRow, int numColumn, m_real inner_cluster_thr, int& cluster_n, int* group_index, matrixn& aCenter, matrixn& refPattern, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL);
	
	// ������ ������ ���͸� ���� Ŭ�����Ϳ� �� �� �ִ�.
	static void LexicoCluster(const matrixn& distMat, int& cluster_n, int* group_index);

	// needed for Classify
	class SubClustering
	{
	public:
		void Init();
		void AddElement(int targetIndex, int GroupIndex, const matrixn& temporalFeature);
		void DelElement(int targetIndex);
		void RemoveEmpty();
		void Cluster(int nNumSample, int nSizePostureFeature, m_real maxInner);
		int CountSameGroupElements(int targetIndex);
		int FindCluster(matrixn& postureFeatures, m_real maxInner);
		matrixn aTemporal;
		matrixn aCenter;		  // clustering ����� ���� ������ �ʱ⵵ �Ѵ�.
		matrixn referencePattern; // clustering ����� ���� ������ �ʱ⵵ �Ѵ�.
		intvectorn aGroupIndex;
		intvectorn aTargetIndex;
		int numGroup;		
		int globalGroupStartIndex;
	};

	// �ð��� feature�� structural ������ concat�� ���Ͱ� �Է��̴�. structure ������ lexico cluster�� ��, subcluster���� dtw�� agglo clustering�Ѵ�.
	static void Classify(const matrixn& aInputVec, int nNumSample, int nSizePostureFeature, m_real INNER_THR, int& numGroup, int* _aGroupIndex, CTArray<SubClustering>& subClusters, m_real *pfMaxInner=NULL, m_real* pfMinInter=NULL, bool bUseAgglo=false);	
};
