#pragma once

#include "vector_n.h"

class Metric
{
public:
	Metric(void);
	virtual ~Metric(void);

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b)=0;
	virtual Metric* Clone() const =0;
};

class L2Metric : public Metric
{
public:
	L2Metric(){}
	virtual ~L2Metric(){}

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class WeightedL2Metric : public Metric
{
public:
	WeightedL2Metric(){}
	virtual ~WeightedL2Metric(){}

	vectorn m_vWeight;
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class QuaterMetric : public Metric
{
public: 
	QuaterMetric (){}
	virtual ~QuaterMetric(){}

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const	{ return new QuaterMetric();}	
};

class KovarMetric : public Metric
{
public:
	KovarMetric();
	virtual ~KovarMetric(){}

	// inputs
	vectorn m_weights;//!< default: (1,1,1,....,1)
	vector3 m_axis;	//!< default: (0,1,0)

	// outputs
	matrix4 m_transfB;
	matrixn m_srcA, m_srcB, m_transformedB;	//!< Three n*3 matrixes.
	
	/**
	 * point cloud B�� m_axis������ m_transfB�� ��ȯ�ϸ� weighted �ּ� �Ÿ� �������� �ǵ��� matching�ȴ�. �׶��� L2 �Ÿ��� return
	 * 
	 * \param a a�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \param b b�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \return a�� b�� KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};


class PointCloudMetric: public Metric
{
public:
	PointCloudMetric(){}
	virtual ~PointCloudMetric(){}

	// outputs
	matrix4 m_transfB;
	matrixn m_transformedB;	
	
	/**
	 * point cloud B�� y������ m_transfB�� ��ȯ�ϸ� weighted �ּ� �Ÿ� �������� �ǵ��� matching�ȴ�. �׶��� L2 �Ÿ��� return
	 * 
	 * \param a a�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \param b b�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \return a�� b�� KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
};

class CDynamicTimeWarping;
class DTWMetric : public Metric
{
public:
	DTWMetric(int numColumn);
	virtual ~DTWMetric();

	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
private:
	matrixn m_srcA, m_srcB;
	CDynamicTimeWarping* m_pDTW;
	int m_nColumn;
};

