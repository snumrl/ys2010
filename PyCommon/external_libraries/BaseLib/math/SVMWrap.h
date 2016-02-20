#pragma once

#include "mathclass.h"
#include "svm/svm.h"
#include <vector>
#include "../utility/tarray.h"

struct SClass
{
	SClass(int data, int c)	{ hData=data; hClass=c;};
	int hData;
	int hClass;
};

class CSVMWrap
{
public:
	
	// �ϳ��� ����Ÿ�� data�� vector�ΰ��.
	CSVMWrap(matrixn& data, bool bNormalize=true, int oneDataNumRow=0);	// �� �Լ� �ȿ��� data�� normalize�Ǹ鼭 ������ �ٲ�ٴ� ��ǿ� ������ ��.
	// �ϳ��� ����Ÿ�� data�� vector�ǳ��� �� matrix�ΰ��. (DynamicTimeWarping kernel�� ���� ��찡 ��ǥ��)
	CSVMWrap(CTArray<matrixn>& data, bool bNormalize=true);	// �� �Լ� �ȿ��� data�� normalize�Ǹ鼭 ������ �ٲ�ٴ� ��ǿ� ������ ��.
	
	virtual ~CSVMWrap();

	// utility functions
	void AddTrainingData(int hData, int hClass)			{ m_setTrainingData.push_back(SClass(hData,hClass));};
	int GetPredictedClass(int hData)					{ return m_sAllData.y[hData];};
	int GetUserSpecifiedClass(int hdata)	
	{
		for(int i=0; i<m_setTrainingData.size(); i++)	
			if(m_setTrainingData[i].hData==hdata) return m_setTrainingData[i].hClass; 
		return -1;
	}

	/// Train and predict
	/**
	m_sAllData�� ��� ����Ÿ �� ����ڰ� specify���� ���� ����Ÿ���� ��� �����ϰ� �ִ�. �����ڿ��� ���������.
	
	m_setTrainingData�� ����ڰ� ���° data�� Ŭ������ �������� pair�� list�� ���� �ִ�. EG) (0,PUNCH)->(10, KICK)->...
	  :: TrainAndPredict�� �����ϱ� ���� �ݵ�� ������ش�.

	����� m_sAllData�� y�� ����ȴ�. EG) m_sAllData.y[0]<-PUNCH, m_sAllData.y[1]<-KICK...

	  Option: trainning option.
	    -s svm_type : set type of SVM (default 0)
			0 -- C-SVC
			1 -- nu-SVC
			2 -- one-class SVM
			3 -- epsilon-SVR
			4 -- nu-SVR
		-t kernel_type : set type of kernel function (default 2)
			0 -- linear: u'*v
			1 -- polynomial: (gamma*u'*v + coef0)^degree
			2 -- radial basis function: exp(-gamma*|u-v|^2)
			3 -- sigmoid: tanh(gamma*u'*v + coef0)
			4 -- GDTW: exp(-gamma*DistanceDTW) , this object should be created by CSVMWrap(CTArray<matrixn>& data..) constructor function.
			5 -- Kovar metric: exp(-gamma*kovardist)
			6 -- Quater metric: exp(-gamma*Quaterdist)
		-d degree : set degree in kernel function (default 3) or Number of elements in one data when using GDTW kernel.
		-g gamma : set gamma in kernel function (default 1/k)
		-r coef0 : set coef0 in kernel function (default 0)
		-c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)
		-n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)
		-p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)
		-m cachesize : set cache memory size in MB (default 40)
		-e epsilon : set tolerance of termination criterion (default 0.001)
		-h shrinking: whether to use the shrinking heuristics, 0 or 1 (default 1)
		-wi weight: set the parameter C of class i to weight*C, for C-SVC (default 1)
		-v n: n-fold cross validation mode
	*/
	void TrainAndPredict(const char* option);

	// pair of data index and its class. This should be set before TrainAndPredict.
	std::vector<SClass> m_setTrainingData;	

	// this is constructed in constructor funcition. Also result will be in m_sAllData.y
	svm_problem m_sAllData;

private:
	void ScaleData(matrixn& data);
	double fGamma;
	svm_node *m_xspace;
	int m_nOneDataNumRow;
	void Initialize(matrixn& data, bool bNormalize);
	static void GetMatrix(const svm_node* pNode, int nMatrixRow, matrixn& mat);
	static void GetVector(const svm_node* pNode, vectorn& vec);
	friend class Kernel;
};

