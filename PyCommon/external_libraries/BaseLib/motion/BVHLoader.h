// BVHLoader.h: interface for the BVHLoader class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "motionloader.h"
class Parser;
//! BVH������ tree������ �� ��忡 �ش��Ѵ�. 
/*! \ingroup group_motion */
class BVHTransform : public Bone
{
public:
	//! TYPE
	typedef enum { BVH_DUMMY, BVH_ROOT,BVH_JOINT,BVH_END, BVH_NO_MORE_NODE} BVHType;
		
	BVHTransform(BVHType NodeType);
	virtual ~BVHTransform();

	Bone::BoneType	ToBoneType(BVHType type);

	//! bone offset (translation term)
	vector3 m_offset;

	void Unpack(Parser* file);
	static BVHType CheckNodeType(Parser* file);

};

//! ���� ���� ������ Motion�κ��� �״�� �����Ѵ�. ���߿� PostureIP�� ������ �����ȴ�.
class BVHIP
{
public:
	int m_numChannel;
	int m_numFrames;
	m_real m_fFrameTime;
	m_real **m_aaKeyvalue;

	void Unpack(Parser* file);

	BVHIP();
	~BVHIP();
};


//! BVH�� �а� tree hierarchy�� PostureIP�� �����.
/*! \ingroup group_motion */
class BVHLoader : public MotionLoader
{
public:
	// option=="loadSkeletonOnly" or NULL
	BVHLoader(const char *filename, const char* option=NULL);
	virtual ~BVHLoader();

	//virtual void SetTree(PLDPrim* pTarget)				{ SetTree(pTarget);}; 
	//virtual void SetTreeSkin(PLDPrimSkin *pTarget)		{ SetTreeSkin(pTarget);};	

	virtual void loadAnimation(Motion& mot, const char* filename) const;

	int countTotalChannels() const;
private:
	void MakePositionIPfromBVHIP(Motion& mot, const BVHIP& cBVHIP);
};

