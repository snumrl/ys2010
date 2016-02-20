#pragma once

// TRCLoader.h: interface for the TRCLoader class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "motionloader.h"
class Parser;
//! TRC������ tree������ �� ��忡 �ش��Ѵ�. 
class TRCTransform : public Bone
{
public:
	int parentMarkerIndex;
	int markerIndex;

	bool bTranslational;
	vector3 vOffsetDir;

	bool isConnector() const	{return markerIndex==parentMarkerIndex;}
	TRCTransform(BoneType NodeType);
	virtual ~TRCTransform();
};

//! TRC ���� ������ Motion�κ��� �״�� �����Ѵ�. ���߿� Motion���� ������ �����ȴ�.
class TRCIP
{
public:
	int m_numChannel;
	int m_numFrames;
	int m_numMarkers;
	m_real m_fFrameTime;
	
	std::vector<matrixn> m_aaKeyvalue;

	void Unpack(Parser* file, bool isTranspose);

	TRCIP();
	~TRCIP();
};
#include "../baselib/math/optimize.h"

//! TRC�� �а� tree hierarchy�� PostureIP�� �����.
/*! \ingroup group_motion */
class TRCLoader : public MotionLoader
{
	TRCIP cTRCIP;

public:
	TRCLoader(const char *filename, const char* option);
	void setValue(TRCTransform *ptr, char *name);
	virtual ~TRCLoader();

	/// parentMarkerIndex=-1 if there is no parent marker.
	void createTranslationalJoint(int parentMarkerIndex, int markerIndex, const char* tx, const char* rx);
	void createRotationalJoint(int parentMarkerIndex, int markerIndex, vector3 offsetDir, 
const char* axis);
	void createRotationalJoint2(int parentMarkerIndex, int markerIndex, vector3 offsetDir, 	const char* axis1, const char* axis2);
void createEffector(int parentMarkerIndex, int markerIndex, vector3 offsetDir);
private:
	std::vector<TRCTransform*> _temporaryNodes;
	IndexMapping _treeIndexToMarkerIndex;
	void buildHierarchy(TRCIP& cTRCIP);
	void MakePositionIPfromTRCIP(const TRCIP& cTRCIP, bool isTranspose);
	
};

