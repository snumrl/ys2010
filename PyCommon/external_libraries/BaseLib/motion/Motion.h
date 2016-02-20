//  Motion.h: interface for the Motion class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_)
#define AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "MotionLoader.h"

#include "PostureIP.h"
#include "../TwoPosture.h"
#include "../math/PCA.h"
#include "../math/filter.h"
#include "stitch.h"


class PLDPrimSkin;
class ModelLoader;
class MotionLoader;


//! ���� ������ �ش� ��� ������ ���� Ŭ����.
/*! ���� ������ m_pSkeleton�� ������ִ�. ��, ��� data�� m_pMotionData�� ����ȴ�. ��ǿ� ���� High level manipulation�Լ����� �� Ŭ������ �ְ�, 
low level manipulation�Լ����� ����� m_pMotionData�� PostureIPŬ������ ���� �ִ�.
--> taesoo�� refectoring���̴�. ��� data�� MotinoClass�� ������ �ϰ�, PostureIP�� ���� ����. ��ǿ� ���� HighLevel manipulation�Լ����� MotionUtil namespace�� �̵�.
	\ingroup group_motion
*/
 
class Motion  : public Node
{
private:
	float m_fFrameTime;
	TString m_strIdentifier;	//!< default�δ� NULL string, �ʿ��ϸ� SetIdentifier�ؼ� ����Ͻÿ�.
	bitvectorn m_aDiscontinuity;	//!< ���� ���� �ʴ� ���� Ŭ���� concat�Ѱ�� ù�����Ӹ��� discontinuity�� true�� ���õȴ�.

	int m_numFrame;
	int m_maxCapacity;
	TArray<Posture> keyvalue;//!< PLD�� ����Ǿ� �ִ� �ð��� ���� translation�� 
	MotionLoader* m_pSkeleton;	//!< �׻� reference�̴�. �� �� bvh���� MotionLoader�� �̸� ������ ���� �����͸� �Ѱܼ� ����Ѵ�.

public:
	
	inline MotionLoader& skeleton() const	{ return *m_pSkeleton;};

	// make skeleton() be Pose(iframe)
	void setSkeleton(int iframe) const;
	void setPose(int iframe, const Posture& pose);
	


	Motion();
	/**
	 * source�κ��� motion�� �����Ѵ�. 
	 * \param pSource Original motion�� ���� �ִ�.
	 * \param pNew NULL�� ��� m_pMotionData�� ������ reference�� �ȴ�. new ���� ������ PostureIP�� �ѱ��, ������ original���� �����Ѵ�.
	 * \return 
	 */
	Motion(MotionLoader* pSource);
	Motion(const Motion& srcMotion, int startFrame, int endFrame=INT_MAX);
	Motion(Motion const& other);	// copy constructor;
	Motion& operator=(const Motion& other);
	virtual ~Motion();

	
	

	// ������ ���̴� "Ű������ ����-1" �� �����Ѵ�. �� ���������Ӱ� ���̸� 1�� ����Ѱ�.
	int length() const				{return NumFrames()-1;}
	void changeLength(int length)	{ Resize(length+1); }

	// Init 
	void empty();

	void setMaxCapacity(int maxCapacity)	{ Msg::verify(m_numFrame==0, "SetMaxCapacityError"); m_maxCapacity=maxCapacity;}

	/// frame time�� skeleton�� source�� �����ϰ� �Ѵ�.
	void InitEmpty(const Motion& source, int numFrames);

	/// frame time�� MotionLoader�� ���� �ִ� ���� ����Ÿ�� ���� �����Ѵ�. ���� ���� ����Ÿ�� ���°�� 30FPS�� ���õȴ�. �ʿ��ϸ� frameTime()�� ���� �������ٰ�.
	void InitEmpty(MotionLoader* pSource, int numFrames);
	
	// skeleton�� �ʱ�ȭ
	void InitSkeleton(MotionLoader* pSource);

	// skeleton�� �ʱ�ȭ�� ��, skeleton�� ���۵���Ÿ(m_cPostureIP)�� ������ �ִ°�� ���۵� �ʱ�ȭ.
	void Init(MotionLoader* pSource);
	
	//! �Ȱ��� ����� �����.(�Ϻκ��� ���ͼ� ���� ���� �ִ�.)
	void Init(const Motion& srcMotion, int startFrame=0, int endFrame=INT_MAX); 
	//! �Ȱ��� ����� �����.(�Ϻκ��� ���ͼ� ���� ���� �ִ�.)
	Motion* Clone(int startFrame=0, int endFrame=INT_MAX) const { return new Motion(*this, startFrame, endFrame);}
	
	void SetIdentifier(const char* id)			{ m_strIdentifier=id;};			//!< use as you want.
	const char* GetIdentifier() const			{ return m_strIdentifier;};		//!< use as you want.

	// skeleton�� ����� ��� ����. ".mot"�� export. loading�� MotionLoader::loadAnimaion���� �����ϴ�. 
	void exportMOT(const char* filename) const;	

	// 0 <= criticalTime <= numFrames()-1
	// ��, 0�϶� Pose(0)��, numFrames()-1�϶� Pose(numFrames()-1)�� return�ȴ�.
	// �������� ���� ����Ÿ�� discrete�� �������� ���ұ� ������, �� SamplePose�Լ��� ����������
	// 2006�� 6�� ���� ���ʹ� ���۵����͸� ���ӵ� Ŀ���� ���ø� ������ ����ϰ� �ִ�. ���� �� �Լ��� �����ϴ�.
	void samplePose(Posture& pose, m_real criticalTime) const;

	//! pose����� ũ�⸦ �ٲ۴�. ���̰� �þ�� ��� �� pose���� ���ʿ� �����.
	void Resize(int frame);
	// endFrame���� ����. �ٸ����� ���߱� ���ؼ� ���ƽ��ϴ�.
	void Concat(const Motion* pAdd, int startFrame=0, int endFrame=INT_MAX, bool bTypeCheck=true);
		
	int NumJoints()	const				{ return ((m_numFrame>0)?Pose(0).numRotJoint():0);}
	int NumTransJoints()	const		{ return ((m_numFrame>0)?Pose(0).numTransJoint():0);}
	int NumFrames() const				{ return m_numFrame;}

	float totalTime() const				{ return m_fFrameTime*length();}
	// length+1 �� �����Ѵ�.
	int numFrames(float second) const	{ return ROUND(second/FrameTime())+1;}
	
	float FrameTime() const				{ return m_fFrameTime;}
	void FrameTime(float ftime)			{ m_fFrameTime=ftime;}
	int FrameRate() const				{ return int(1.f/FrameTime()+0.5f);}
	int KernelSize(float fTime) const	{ return Filter::CalcKernelSize(fTime, FrameTime());};
	
	bool isConstraint(int fr, int eConstraint) const;
	void setConstraint(int fr, int con, bool bSet=true);

	bool IsDiscontinuous(int fr) const			{ return m_aDiscontinuity[fr%m_maxCapacity];}	
	void setDiscontinuity(int fr, bool value)	{ m_aDiscontinuity.setValue(fr%m_maxCapacity, value);}


	int Parent(int jointIndex) const;//			{ return m_pSkeleton->GetParentJoint(jointIndex);};
	inline Posture& Pose(int iframe) const;
	TwoPosture& pose2(int iframe) const;
										
	enum { LOCAL_COORD, GLOBAL_COORD, FIXED_COORD, FIRST_ARRANGED_COORD, PELVIS_LOCAL_COORD, PELVIS_FIXED_COORD, NUM_COORD };

	void ChangeCoord(int eCoord);
	void CalcInterFrameDifference(int startFrame = 0);
	void ReconstructDataByDifference(int startFrame = 0, bool bForward=true)  
			{ 	_reconstructRotByDifference(startFrame,bForward);	_reconstructPosByDifference(startFrame, bForward);}
	void _reconstructPosByDifference(int startFrame = 0, bool bForward=true);
	void _reconstructRotByDifference(int startFrame = 0, bool bForward=true);


	//void changeFactory(TFactory<Posture>* pFactory)	;

	// do not use the followin functions unless you exactly know what you are doing.
	virtual void _Init(int nkey, int numRotJoint, int numTransJoint, float fFrameTime);
	virtual void _unpack(BinaryFile& File, int nVersion);
	virtual void _pack(BinaryFile& File, int nVersion) const;

	/////////////////////////////////////////////////////////////////////
	// �Ʒ��� ��� deprecated functions: ������� ����.
	/////////////////////////////////////////////////////////////////////

	// 0.5 <=criticalTime< numFrames()-0.5
	// ��, 0.5�϶� Pose(0)��, numFrames()-0.5�϶� Pose(numFrames()-1)�� return �ȴ�.
	void SamplePose(Posture& pose, m_real criticalTime) const;
	// totalTime�� TotalTime�� �ٸ������ ���ٴµ� �����Ұ�. -> TotalTime�� deprecated function.
	float TotalTime() const				{ return m_fFrameTime*NumFrames();}

	// If no frame in the interval is constrained, return -1. otherwise, return the indexed of first constrained frame;
	int isConstraint(int start, int end, int eConstraint) const;

	bool IsValid(int startFrame, int endFrame) const;
	bool IsValid(int iframe) const		{ return iframe>=0 && iframe<NumFrames();}
	bitvectorn& discontinuity()			{ Msg::verify(m_numFrame<m_maxCapacity, "discontinuity vector is invalid"); return m_aDiscontinuity;}
	bool IsContinuous(int iframe) const { return (iframe<NumFrames() && !IsDiscontinuous(iframe)); }

	// numFrames�� �ٸ� ����� ����.
	int NumFrames(float fTime) const	{ return int(fTime/FrameTime()+0.5);}

private:
	void Release();
		
	// called only in constructor.	
	void _firstinit();

	void cloneFrom(const Motion& other, int otherStartFrame, int otherEndFrame, int thisStart, bool bTypeCheck=true);
};

#endif // !defined(AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_)
