#pragma once

#include "node.h"
#include "ModelLoader.h"
#include "modelLoader.h"
#include "motion.h"
#include "../utility/nametable.h"

class Bone : public Node
{
public:
	enum BoneType { DUMMY, ROOT,JOINT,END } m_eType;
	Bone ();
	virtual ~Bone();

	inline Bone* child() const		{ return (Bone*)m_pChildHead;}
	inline Bone* sibling() const	{ return (Bone*)m_pSibling;}
	inline Bone* parent() const		{ return (Bone*)m_pParent;}
	inline int treeIndex() const	{ return GetIndex();}

	//! CHANNEL (loading �� ����)
	enum { XPosition, YPosition, ZPosition, ZRotation, XRotation, YRotation};
	int m_numChannel;
	int *m_aeChannels;

	/// e.g. setChannels("XYZ", "ZXY")
	void setChannels(const char* translation_axis, const char* rotation_axis);
	/// e.g. "ZXY"=getRotationalChannels()
	TString getRotationalChannels() const;
	TString getTranslationalChannels() const;

	m_real length() const;
	void getOffset(vector3& offset) const;
	void getTranslation(vector3& trans) const;
	vector3 getTranslation() const;
	void getRotation(quater& rot) const;

	matrix4 m_matRotOrig;	//!< translation�� rotation�ϴ� matrix, initial posture�� �����Ѵ�. (BVH ���Ͽ����� translation�� ���� �ִ�.)
	matrix4 m_matRot;		//!< translation�� rotation�ϴ� matrix, MotionLoader::SetPose���� ���ȴ�.
	matrix4 m_matCombined;	//!< MotionLoader::SetPose���� ���ȴ�. global rotation�ϰ� translation�� ����Ǿ� �ִ�.

	virtual void pack(BinaryFile& bf, int version) const;
	virtual void unpack(BinaryFile& bf, int version);

	void packBVH(FILE* file, int level, MotionLoader* pLoader);



};

/**
 * \ingroup group_motion
 *
 * ������ skeleton�� �ε��ϰ� tree �ڷᱸ���� ���´�. �ε��� ���� ����Ÿ�� m_cPostureIP�� �����Ѵ�.
 *
 * Bone�� �ϳ��� transformation��Ʈ������ �ش��ϰ�, �ϳ��� ���� rotationalJoint�� translationJoint�� ���ÿ� �ش��Ҽ��� �ְ� ���� �ϳ����� �ش��� �� �ִ�. ���� joint�� ���� ������� �������� �ִ�. �̰�� �� Ʈ������ ��Ʈ������ constant�� �ȴ�.
 */

class MotionLoader : public ModelLoader 
{
	TFactory<Posture>* m_pFactory;
public:
	MotionLoader();

	// filename: "~.skl" or "~.mot"  (.mot�ΰ�� m_cPostureIP�� ���������.)
	// option=="loadSkeletonOnly" or NULL
	MotionLoader(const char* filename, const char* option=NULL);
	virtual ~MotionLoader();

	// exportSkeleton("~.skl");
	void exportSkeleton(const char* filename) const;

	// loadAnimation("~.anim" or "~.mot"), ����� Ŭ���������� "~.bvh" �� "~.amc"�� ���� ���� ���ȴ�.
	virtual void loadAnimation(Motion& mot, const char* filename) const;

	void ReadJointIndex(const char* filename);

	int GetNumRotJoint() const;	//!< KeyFrame�Ǿ� �ִ� ����Ʈ�� ������ ���Ѵ�.
	int GetNumTransJoint() const;
	int GetNumBone() const; //!< 

	void setPose(const Posture& pose);
	// ��� Ʈ���� update���� �ʰ�, �Ѱ��� chain�� update�Ѵ�.
	void setChain(const Posture& pose, int ijoint);

	Motion m_cPostureIP;	//!< file���� �о��� �Ϲ����� ��� 

	// Posture��� ����� Ŭ������ ����ϰ� ������� ȣ��.
	void changeFactory(TFactory<Posture>* pFactory)	;
	const TFactory<Posture>* factory() const	{ return m_pFactory;}
	///////////////////////////////////////////////////////
	// ���� ����
	///////////////////////////////////////////////////////
	/// child�� transform�� identity�� �ǰ�, bMoveChildren==true�ϰ�� parent�� children�� ��� ���ο� ���ϵ忡 �ٴ´�.
	void insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren=true);	
	/// type="T" (translational-���� �����ȵ�) or "R" (rotational) or "RT" (both)
	void insertJoint(Bone& target, const char* type);
	///////////////////////////////////////////////////////
	// ���� �޾ƿ���.
	///////////////////////////////////////////////////////


	Bone& getBoneByTreeIndex(int index)	const;
	Bone& getBoneByRotJointIndex(int iRotJoint)	const;
	Bone& getBoneByTransJointIndex(int iTransJoint)	const;
	Bone& getBoneByVoca(int jointVoca)	const;
	Bone& getBoneByName(const char*) const;

	int getTreeIndexByName(const char* name) const					{ return GetIndex(name);}
	int getTreeIndexFromRotJointIndex(int rotjointIndex) const		{ return m_aRotJoint2TreeIndex[rotjointIndex];};
	int getTreeIndexFromTransJointIndex(int transjointIndex) const  { return m_aTransJoint2TreeIndex[transjointIndex];}
	int getTreeIndexFromVoca(int jointVoca) const					{ return m_aRotJoint2TreeIndex[m_aVoca2RotJointIndex[jointVoca]];}

	// slow because of full search.
	int getRotJointIndexByName(const char* nameID) const;
	int getRotJointIndexFromTreeIndex(int treeIndex) const;
	int getRotJointIndexFromVoca(int jointVoca) const;
	int getJointVocaFromRotJointIndex(int rotjointIndex) const;

	int getTransJointIndexByName(const char* nameID);
	int getTransJointIndexFromTreeIndex(int treeIndex) const;

	void UpdateInitialBone();	//!<���� ������ global joint������ ���ϱ� ���ؼ� ���ȴ�.


	void Scale(float fScale);
	void scale(float fScale, Motion& mot);


	// almost private
	void _changeJointIndex(int jointVoca, int newJointIndex)	{ m_aVoca2RotJointIndex[jointVoca]=newJointIndex;}
	
	// �Ʒ��� deprecated..
	// ���� : NodeIndex�� JointIndex�� �ٸ���. GetTreeIndex�� �ϸ� tree node�� index�� ������, getRotJointIndexFromTreeIndex�� �ϸ� joint node�� index�� ���´�.
	// joint node�� tree node�� ��� ĸ�� ����Ÿ�� ����� ����, ������ ���� �ű���̴�. �� PostureIP�� m_aRotations�� �ε����̴�..
	
	

	/*
	

	vector3 pos(int rotjoint) const					{ vector3 trans; getBoneByRotJointIndex(rotjoint).getTranslation(trans); return trans;}
	vector3 pos(Bone& bone) const					{ vector3 trans; bone.getTranslation(trans); return trans;}
*/

	// translation table (joint vocaburary)
	enum { HIPS, LEFTHIP , LEFTKNEE, LEFTANKLE, RIGHTHIP, RIGHTKNEE, RIGHTANKLE, CHEST, CHEST2, LEFTCOLLAR, LEFTSHOULDER, LEFTELBOW, LEFTWRIST, RIGHTCOLLAR, RIGHTSHOULDER, RIGHTELBOW, RIGHTWRIST, NECK, HEAD, NUM_JOINT_VOCA};
	
	

	void pack(BinaryFile & bf, int nVersion) const;	

	int getHandle() const							{ return m_handle;}


	// translation table (joint vocaburary)
	NameTable m_translationTable;

protected:
	
	int unpack(BinaryFile & bf) ;

	// Motion Manager specific
	friend class MotionManager;
	int m_handle;

	void UpdateBone();	//!< global joint������ ���ϱ� ���ؼ� ���ȴ�.

	// tree�� ������, node array�� ���� ��� �Ǵ� node array�� �ֱ�� ������ ������ DFS������ ������� �ʴ� ��� call���ش�.
	void MakeBoneArrayFromTree(int& numChannel);


	//////////////////////////////////////////////////////////////////
	// �Ʒ� ������ ��� MakeBoneArrayFromTree����� ����
	//////////////////////////////////////////////////////////////////

	// Posture�� Rotational joint���� 
	intvectorn m_aRotJoint2TreeIndex;	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index
	intvectorn m_aTree2RotJointIndex;	//!< m_aTree2RotJointIndex[tree node index]=joint index
	intvectorn m_aVoca2RotJointIndex;		//!< m_aVoca2RotJointIndex[voca]=joint index

	// Posture�� Translational joint���� 
	intvectorn m_aTransJoint2TreeIndex;
	intvectorn m_aTree2TransJointIndex;

	// ������ ��� ����. 
	int m_nNumRotNode;
	int m_nNumTransNode;
	int m_nNumEndNode;	//!< EndNode(site)�� ��ǰ� ����Ǿ����� ����.
	
};


// deprecated.
int dep_GetParentJoint(MotionLoader const&, int rotjointIndex) ;
Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) ;	// ToeBone
Bone& dep_GetSiteBone(MotionLoader const& ml, int ijoint)	;
Bone& dep_GetSiteBoneVoca(MotionLoader const& ml, int jointVoca) ;
void dep_MakeQuaternionFromChannel(Bone& bone, int channelStartIndex, m_real *aValue, quater& q, bool bRightToLeft=false);
void dep_makeEulerAngleFromChannel(Bone& bone, int channelStartIndex, quater const& q, m_real *aValue, bool bRightToLeft=false);
