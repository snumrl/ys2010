// PostureIP.h: interface for the Posture class.

#pragma once

#include "Node.h"

enum {CONSTRAINT_LEFT_FOOT=0, CONSTRAINT_RIGHT_FOOT, CONSTRAINT_LEFT_HAND, CONSTRAINT_RIGHT_HAND,
CONSTRAINT_LEFT_TOE,CONSTRAINT_LEFT_HEEL, CONSTRAINT_RIGHT_TOE, CONSTRAINT_RIGHT_HEEL,
CONSTRAINT_INTERACTION, CONSTRAINT_CRI_INTERACTION,WHICH_FOOT_IS_SUPPORTED,IS_FOOT_SUPPORTED, POSE_IS_INVALID, 
CONSTRAINT_LEFT_FINGERTIP, CONSTRAINT_RIGHT_FINGERTIP, NUM_CONSTRAINT};

enum {R_IS_SUPPORTED=0, L_IS_SUPPORTED=1};
enum {ONE_FOOT_IS_SUPPORTED,NO_FOOT_IS_SUPPORTED};

class MotionLoader;
//! BVH�� ����� Matrix���� ����
/*! Rendering�� AlzzaPostureIP���� �� ������ ����� animation�� �����Ѵ�. ��, �� Ŭ������ PostureIP�� AlzzaPostureIP���� ���ÿ� ���Ǳ� ������, ����ϰ� compact�� ���¸� �׻� �����Ͽ��� �Ѵ�.
	\ingroup group_motion
*/
class Posture
{
public:
	Posture();
	Posture(const Posture& other);
	virtual ~Posture();	
	
	virtual void Init(int numRotJoint, int numTransJoint);
	int numRotJoint() const	{ return m_aRotations.size();}
	int numTransJoint() const	{ return m_aTranslations.size();}

	vector3N m_aTranslations;	// !<numTransJoint() ��ŭ�� translation�� ���´�.
	quaterN m_aRotations;	//!< numRotJoint()��ŭ�� rotation�� ���´�.


	Posture& operator=(const Posture& other)	{ Clone(&other); return *this;}

	virtual Posture* clone() const;
	virtual void Blend(const Posture& a, const Posture& b, m_real t);	//!< a*(1-t)+b*(t)
	virtual void Blend(const Posture& b, m_real t)	{	Blend(*this, b, t);}
	virtual void Blend(const Posture** apPostures, const vectorn& aWeights);
	virtual void Align(const Posture& other);

	void pack(BinaryFile & bf, int version) const;
	void unpack(BinaryFile & bf, int version) ;

	vector3 front();
	transf rootTransformation();
	void setRootTransformation(transf const& rootTransf);

	// m_aRotations[0]->m_rotAxis_y*m_offset_q�� decompose
	virtual void decomposeRot() const;


	BitArray constraint;

	///////////////////////////////////////////////
	// Inter-frame difference���� ����
	///////////////////////////////////////////////

	/// ���� �����Ӱ� root�� planar ����(���� �������� local ��ǥ�迡�� ǥ��)
	/** Pose(i).m_aTranslations[0] = Pose(i-1).m_aTranslations[0] + Pose(i-1).m_rotAxis_y.rotate(Pose(i).m_dv) */
	vector3 m_dv;			
	/// ���� �����Ӱ� root�� planar ���������̼� ����(���� �������� local ��ǥ�迡�� ǥ��) 
	/** Pose(i).m_rotAxis_y= Pose(i).m_dq * Pose(i-1).m_rotAxis_y */
	quater m_dq;			 
	
	// decomposeRot �� ����� ����Ǵ� ������ pose�� ���������� ����� ��� mutable�� ��.
	mutable m_real m_offset_y;		//!< m_dv�� ���� ���� �ʴ� y value
	mutable quater m_offset_q;		//!< m_dq�� �������� �ʴ� x �� z ���� ���������̼� ����, �� ����� local ��ǥ�迡�� ���ǵȴ�.
	/// y ���� ȸ������ �����ϴ� rotation
	/** m_aRotations[0] = m_rotAxis_y*m_offset_q (in "quater" sense. Not "D3DXQUATERNION" sense)*/
	mutable quater m_rotAxis_y;

	
	///////////////////////////////////////////////
	// �Ʒ� �������� ���� ������� ����.
	///////////////////////////////////////////////

	// linear term�� linear������ ���ȴ�.
	vectorn m_additionalLinear;
	// size�� 4�� ���. quaternion blending�� �� 4�÷������� �߶� ����ȴ�.
	vectorn m_additionalQuater;

	

	// �Ʒ� �Լ��� dynamic type checking�� ���� �ʴ´�. �𸣸� ������� ����. operator=�̳� clone()�Լ��� ����Ұ�.
	void _clone(const Posture* pPosture);

	// type checking. (derived class���� type checking ����)
	virtual void Clone(const Posture* pPosture)	{_clone(pPosture);}

	//////////////////
	// �ʿ��Ѵ�� �� ��.
	vector3 m_conToeL;
	vector3 m_conToeR;
	vector3& conPosition(int constraint){return (constraint==CONSTRAINT_LEFT_TOE)?m_conToeL:m_conToeR;}


protected:
	
};


// deprecated
// �Ʒ� �Լ��� ��� MotionUtil::Coordinate ���.
void dep_toLocalDirection(Posture const&, const vector3& gdir, vector3& ldir, bool bOnlyVerticalAxis=false) ;
void dep_toGlobalDirection(Posture const&, const vector3& ldir, vector3& gdir, bool bOnlyVerticalAxis=false) ;
void dep_toLocal(Posture&,const vector3& pos, vector3& localPos);
void dep_toGlobal(Posture&,const vector3& pos, vector3& globalPos);
vector3 dep_toLocal(Posture&,const vector3& pos);
vector3 dep_toGlobal(Posture&,const vector3& pos);

