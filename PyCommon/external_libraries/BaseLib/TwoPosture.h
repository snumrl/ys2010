#pragma once

#include "motion/postureip.h"
class TwoPosture : public Posture
{
public:
	TwoPosture(void);
	TwoPosture(const Posture& other);
	~TwoPosture(void);

	// in local coordinate (�̶� m_realRotAxis_y�� ����ؾ� �Ѵ�. m_rotAxis_y�� ����ϸ�, (0,0,1)�� �Ǳ� ����.
	// ���߿� �ѹ��� �̸��� m_localTargetDir�� �ٲܰ���.
	vector3 m_oppenentPos;	
	vector3 m_targetDir;	// in global coordinate

	virtual Posture* clone() const;

	virtual void Blend(const Posture& a, const Posture& b, m_real weight);
	virtual void BlendJustForPhysics( const Posture& b, m_real weight);
	//m_oppenetPos�� ���� ������ �Ʒ� �Լ��� ���ؼ��� �ۼ��� �ִ�.
	virtual void Blend(const Posture** apPostures, const vectorn& aWeights);
	// m_aRotations[0]->m_rotAxis_y*m_offset_q�� decompose
	virtual void decomposeRot() const;

	mutable quater m_realRotAxis_y; //!< y ���� ȸ������ �����ϴ� rotation
	mutable quater m_realOffset_q;  //!< m_dq�� �������� �ʴ� x �� z ���� ���������̼� ����, �� ����� local ��ǥ�迡�� ���ǵȴ�.
	
	virtual void Clone(const Posture* pPosture);
private:
	TwoPosture(const TwoPosture& other) {assert(0);}
	TwoPosture& operator=(const TwoPosture&) {assert(0);}	
	TwoPosture& operator=(const Posture&) {assert(0);}
};
