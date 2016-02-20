#pragma once

namespace MotionUtil
{
	class KovarMetricY : public Metric
{
public:
	KovarMetricY(){}
	virtual ~KovarMetricY(){}

	// outputs
	matrix4 m_transfB;
	matrixn matA, matB, m_transformedB;	//!< Three n*3 matrixes.
	
	/**
	 * point cloud B�� y������ m_transfB�� ��ȯ�ϸ� weighted �ּ� �Ÿ� �������� �ǵ��� matching�ȴ�. �׶��� L2 �Ÿ��� return
	 * 
	 * \param a a�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \param b b�� n*3 matrix�κ��� ��ȯ�� ������ �����Ѵ�. �� (x1,y1,z1,x2,y2,z2,...)
	 * \return a�� b�� KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
	
	static void extractFeature(const Motion& mot, vectorn& feature, intvectorn const& frames);
};

class TransitionCost
{
	matrixn mFeatureVectors;
	bitvectorn mValid;
	int mMetric;
public:
	enum {GLOBAL_POS, FIXED_POS, ANGLE};	// GLOBAL_POS�� ��� kovar metric���. 
	TransitionCost(const Motion& mot, int metric=ANGLE);
	virtual ~TransitionCost(void);

	m_real transitionCost(int from, int to) const;
};

}