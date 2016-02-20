// ASFLoader.h: interface for the ASFLoader class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ASFLOADER_H__7A09CE2C_AAAC_41AD_8601_629999FAB82A__INCLUDED_)
#define AFX_ASFLOADER_H__7A09CE2C_AAAC_41AD_8601_629999FAB82A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "MotionLoader.h"
#include <vector>

class CTextFile;
class ASFRoot
{
public:
	ASFRoot()	{numOrder=0;}
	~ASFRoot()	{;}
	void Unpack(CTextFile* pFile);
	char axis[4];
	int numOrder;
	int order[6];
	vector3 position;
	vector3 orientation;
};

class ASFBoneData
{
public:
#define MAX_NAME_LEN 20
	ASFBoneData()	{numDof=0;}
	~ASFBoneData()	{;}
	void Unpack(CTextFile* pFile);
	int id;
	char name[MAX_NAME_LEN];
	vector3 direction;
	float length;
	vector3 vecAxis;
	char axis[4];
	int numDof;
	int dof[3];
};

class ASFTransform : public Bone
{
public:
	ASFTransform();
	ASFTransform(const ASFRoot& root);
	ASFTransform(const ASFBoneData& bone);
	virtual ~ASFTransform();
	quater c;
	quater c_inv;
};
//! ASF�� �а� tree hierarchy�� PostureIP�� �����.
/*! \ingroup group_motion */
class ASFLoader : public MotionLoader  
{
public:
	ASFLoader(const char* filename);
	virtual ~ASFLoader();
	void LoadAMC(const char* filename, Motion& motionData) const;

	virtual void loadAnimation(Motion& mot, const char* filename) const
	{
		TString fn=filename;
		if(fn.right(4).toUpper()==".AMC") LoadAMC(filename, mot);
		else MotionLoader::loadAnimation(mot, filename);
	}

private:
	void LoadASF(const char* filename);
	void ReadHierarchy(CTextFile& file, ASFRoot& cRoot, std::vector<ASFBoneData>& aBoneData);
	void ConvertASFTreeToBVHTree();
	TString m_strName;
	bool m_bDataDegree;
};

#endif // !defined(AFX_ASFLOADER_H__7A09CE2C_AAAC_41AD_8601_629999FAB82A__INCLUDED_)
