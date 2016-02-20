#pragma once

class Path2D
{
	vector3N mPos;
	quaterN mOri;	// y component
	vector3N mDV;
	quaterN mDQ;
	int mStart;
public:
	// start > 0 because mStartPos and mStartOri are from start-1.
	Path2D(Motion const& mot, int start, int end);
	virtual ~Path2D(void);

	// copy pos and ori back to the motion assuming that motion's offset_q and height information is correct.
	void setMotion(Motion & mot, int start);
	void setMotionDelta(Motion & mot, int start);
	vector3& pos(int i) const	{ return mPos[i+1-mStart];}
	quater& ori(int i) const	{ return mOri[i+1-mStart];}
	vector3& dv(int i) const	{ return mDV[i-mStart];}
	quater& dq(int i) const		{ return mDQ[i-mStart];}

	int size()	{ return mDV.size();}
	int start()	{ return mStart;}

	// update ori(i) based on dq(i) 
	void updateOri(int i);
	// update pos(i) based on dv(i) 
	void updatePos(int i);
	// update ori(i) and pos(i) based on dq(i) and dv(i)
	void updatePath(int i);
	// update dv(i) based on pos(i) and pos(i-1) and ori(i-1)
	void updateDV(int i);
	// update dq(i) based on ori(i-1) and ori(i)
	void updateDQ(int i);
};

class TwoPath2D
{
	int mStart;
	Path2D mPath1;
	Path2D mPath2;
	vector3N mTarget[2];
public:
	TwoPath2D(Motion const& mot1, Motion const& mot2, int start, int end);

	Path2D& path(int i)						{return (i==0)?mPath1:mPath2;}
	Path2D const& path(int i) const			{return (i==0)?mPath1:mPath2;}
	
	vector3 calcTargetDir(int ePath=0, int i=0, bool bGlobal=false) const;
	m_real calcRetargetAngle(int ePath, int correctTime) const;
	m_real calcRetargetAngle2(int ePath, int correctTime) const;

	// weight�� 0 �϶� target 0, 1�϶� target 1�� distanace�� ���.
	m_real calcRetargetDistance(int correctTime, m_real weight) const;
	
	// �� distance�� MIN�� MAX���̿��� �������� �ּ����� retarget
	m_real calcRetargetDistance(int correctTime) const;
	m_real calcRetargetDistance2(int correctTime) const;
	
	// weight�� 0 �϶� target 0 �� �������� �Ÿ��� �����ϰ�, 1�϶� target 1�� �������� �Ÿ� ����.	
	void retarget(m_real weight=0.5, int end=INT_MAX);
};