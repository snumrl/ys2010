// stdafx.h : �� ������� �ʰ� ���� ����ϴ�
// ǥ�� �ý��� ���� ���� �� ������Ʈ ���� ���� ������ 
// ��� �ִ� ���� �����Դϴ�.

#pragma once

#include "MATHCLASS/mathclass.h"
#include "PmQm/pm.h"

inline jhm::quater ToJHM(quater q)
{
	return jhm::quater(q.w, q.x, q.y, q.z);
}

inline jhm::vector ToJHM(vector3 q)
{
	return jhm::vector(q.x, q.y, q.z);
}

inline quater ToBase(jhm::quater q)
{
	return quater(q.w(), q.x(), q.y(), q.z());
}

inline vector3 ToBase(jhm::vector q)
{
	return vector3(q.x(), q.y(), q.z());
}

inline vector3 ToBase(jhm::position	q)
{
	return vector3(q.x(), q.y(), q.z());
}
class ConvertMotionToPmQm
{
	MotionLoader const& mSkeleton;
	PmHuman* mHuman;
	int _searchUnusedEnum();
	void createPmHuman(PmHuman* newHuman);
	
public:

	PmMaskType enumToMask(int _enum);

	int NUM_ENUM;
	intvectorn treeIndex2enum;
//	intvectorn enum2jointIndex;

	ConvertMotionToPmQm(MotionLoader const& skel);

	PmHuman*			getBody() const		{ return mHuman;}
	MotionLoader const& getSkeleton() const	{ return mSkeleton;}

	void setPmPosture(Posture const& pose, PmPosture& posture);
	void getPmPosture(PmPosture const& posture, Posture& pose);

	// motion already has a body (PmHuman)
	void setPmLinearMotion(Motion const& mot, PmLinearMotion& motion);
	void getPmLinearMotion(PmLinearMotion const& motion, Motion & mot);
};