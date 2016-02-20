// ConstraintMarking.cpp: implementation of the CConstraintMarking class.
//
//////////////////////////////////////////////////////////////////////


#include "stdafx.h"
#include "../baselib/math/mathclass.h"
#include "ConstraintMarking.h"
#include "motionutil.h"
#include "Motion.h"
#include "../baselib/image/Image.h"
#include "../baselib/image/ImagePixel.h"
#include "../baselib/image/ImageProcessor.h"
#include "../baselib/utility/configtable.h"
#include "../baselib/motion/motionloader.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


namespace MotionUtil
{
	//////////////////////////////////////////////////////////////////////
	// Construction/Destruction
	//////////////////////////////////////////////////////////////////////

	ConstraintMarking::ConstraintMarking(Motion* pMotion , bool bCleanup, bool bFillGap, int minConDuration, int reduceCon)
	{
		m_bCleanup=bCleanup;
		m_pAccurateMotion=pMotion;
		m_bFillGap=bFillGap;
		m_minConDuration=minConDuration;
		m_reduceCon=reduceCon;

		m_abLeftFoot.setSize(m_pAccurateMotion->NumFrames());
		m_abRightFoot.setSize(m_pAccurateMotion->NumFrames());

		for(int i=0; i<NUM_CON; i++)
		{
			m_abLeft[i].setSize(m_pAccurateMotion->NumFrames());
			m_abRight[i].setSize(m_pAccurateMotion->NumFrames());
		}	
	}

	ConstraintMarking::~ConstraintMarking()
	{
	}

	void ConstraintMarking::encodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 const& conPos)
	{

		if(con==CONSTRAINT_LEFT_TOE)
			skeleton.setChain(pose, skeleton.getRotJointIndexFromVoca(MotionLoader::LEFTANKLE));
		else if(con==CONSTRAINT_RIGHT_TOE)
			skeleton.setChain(pose, skeleton.getRotJointIndexFromVoca(MotionLoader::RIGHTANKLE));
		else
			Msg::error("Constraint is not con toe!");

		Bone& hips=skeleton.getBoneByVoca(MotionLoader::HIPS);

		quater q;
		hips.getRotation(q);

		vector3 p;
		hips.getTranslation(p);

		vector3 dv;
		dv.difference(p, conPos);

		quater inv_q;
		inv_q.inverse(q);

		if(con==CONSTRAINT_LEFT_TOE)
			pose.m_conToeL.rotate(inv_q, dv);
		else
			pose.m_conToeR.rotate(inv_q, dv);
	}

	vector3 ConstraintMarking::decodeCon(Motion const& mot, int iframe, int con)	
	{
		vector3 conPos;
		decodeCon(con, mot.Pose(iframe), mot.skeleton(), conPos);
		return conPos;
	}

	void ConstraintMarking::decodeCon(int con, Posture& pose, MotionLoader& skeleton, vector3 & conPos)
	{
		skeleton.setChain(pose, skeleton.getRotJointIndexFromVoca(MotionLoader::HIPS));


		Bone& hips=skeleton.getBoneByVoca(MotionLoader::HIPS);

		quater q;
		hips.getRotation(q);

		vector3 p;
		hips.getTranslation(p);

		vector3 dv;
		if(con==CONSTRAINT_LEFT_TOE)
			dv.rotate(q, pose.m_conToeL);
		else
			dv.rotate(q, pose.m_conToeR);

		conPos.add(p, dv);
	}

	void ConstraintMarking::fillGap(bitvectorn& ab, int numPosture, int interval)
	{
		bitvectorn original;
		original.setSize(numPosture);

		for(int i=0; i<numPosture; i++)
		original.setValue(i,ab[i]);

		int j;
		int gapCount;
		// interval frame������ ������ �ٿ��ش�.
		for(int i=0; i<numPosture; i++)
		{
			if(original[i]==false)
			{
				j=original.find(i);
				if(j==numPosture) break;
				gapCount=j-i;
				if(gapCount<interval)
				{
					for(int k=0; k<gapCount; k++)
					{
						ab.setAt(i+k);
					}
				}
				i=j-1;
			}
		}	
	}

	void ConstraintMarking::fillGapUsingPos(bitvectorn &ab, int numPosture, const matrixn& pos, int interval, float thr)
	{
		bitvectorn original;
		original.setSize(numPosture);

		for(int i=0; i<numPosture; i++)
		original.setValue(i,ab[i]);

		int gapCount,j;
		// interval frame������ ������ �ٿ��ش�.
		for(int i=1; i<numPosture; i++)
		{
			if(original[i]==false)
			{
				j=original.find(i);
				if(j==numPosture) break;
				gapCount=j-i;
				if(gapCount<interval)
				{
					ASSERT(original[i-1] || i==1);

					if(pos.row(i-1).distance(pos.row(j)) < thr)
					{
						for(int k=0; k<gapCount; k++)
							ab.setAt(i+k);
					}
				}
				i=j-1;
			}
		}
	}

	void ConstraintMarking::calcConstraintPos(int constraint)
	{
		MotionUtil::GetSignal getSignal(*m_pAccurateMotion);
		
		bitvectorn conToe;
		getSignal.constraint(constraint, conToe);

		intvectorn conInterval;

		MotionUtil::SegmentFinder sf(*m_pAccurateMotion, 0, m_pAccurateMotion->NumFrames());

		intvectorn grp;
		intvectorn domain;
		vectorn footPrint(3);
		
		int aargMin=0;

		for(int iseg=0; iseg<sf.numSegment(); iseg++)
		{
			int startSeg=sf.startFrame(iseg);
			int endSeg=sf.endFrame(iseg);

			conInterval.runLengthEncode(conToe, startSeg, endSeg);
			int numConGrp=conInterval.size()/2;
			
			for(int grp=0; grp<numConGrp; grp++)
			{
				int start=conInterval[grp*2];
				int end=conInterval[grp*2+1];

				vector3 conPos(0,0,0);
				vector3 pos;
								
				for(int i=start; i<end; i++)
				{
					m_pAccurateMotion->skeleton().setPose(m_pAccurateMotion->Pose(i));			
					Bone& bone=dep_GetBoneFromCon(m_pAccurateMotion->skeleton(),constraint);
						
					bone.getTranslation(pos);
					conPos+=pos;						
				}
				conPos/=float(end-start);

				for(int i=start; i<end; i++)
				{
					if(constraint==CONSTRAINT_LEFT_TOE)
						dep_toLocal(m_pAccurateMotion->Pose(i), conPos, m_pAccurateMotion->Pose(i).m_conToeL);
					else if(constraint==CONSTRAINT_RIGHT_TOE)
						dep_toLocal(m_pAccurateMotion->Pose(i), conPos, m_pAccurateMotion->Pose(i).m_conToeR);
					else Msg::error("Constraint is not con toe!");
				}
			}

			// fill gap (linearly blend constraint positions inbetween constrained frames.)
			for(int grp=0; grp<=numConGrp; grp++)
			{
				int prevEnd, nextStart;
				vector3 prevConPos, nextConPos;
				
				if(grp==0)
					prevEnd=startSeg;
				else
					prevEnd=conInterval[(grp-1)*2+1];
				
				if(grp==numConGrp)
					nextStart=endSeg;
				else
					nextStart=conInterval[grp*2];
				
#define CONTOE(y, x) if(constraint==CONSTRAINT_LEFT_TOE) y=m_pAccurateMotion->Pose(x).m_conToeL;\
					else y=m_pAccurateMotion->Pose(x).m_conToeR
				if(grp==0)
				{
					CONTOE(prevConPos, nextStart);
				}
				else
				{
					CONTOE(prevConPos, prevEnd-1);
				}
				
				if(grp==numConGrp)
				{
					CONTOE(nextConPos, prevEnd-1);
				}
				else
				{
					CONTOE(nextConPos, nextStart);
				}

				for(int i=prevEnd; i<nextStart; i++)
				{
					vector3 toepos;
					toepos.interpolate( (m_real)(i-prevEnd+1)/(m_real)(nextStart-prevEnd+1), prevConPos, nextConPos);

					if(constraint==CONSTRAINT_LEFT_TOE)
						m_pAccurateMotion->Pose(i).m_conToeL=toepos;
					else
						m_pAccurateMotion->Pose(i).m_conToeR=toepos;
				}
			}
		}
	}

	void ConstraintMarking::calcConstraint(float toe_height_thr, float toe_speed_thr, float heel_height_thr, float heel_speed_thr, bool bOutputImage, int eHowToChooseConstraint)
	{
		// importance���
		int numJoint=m_pAccurateMotion->NumJoints();
		int numPosture=m_pAccurateMotion->NumFrames();

		int jointL=m_pAccurateMotion->skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTANKLE);
		int jointR=m_pAccurateMotion->skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTANKLE);

		float fFrameTime=m_pAccurateMotion->FrameTime();

		hypermatrixn posL, posR;
		hypermatrixn velL, velR;

		posL.setSize(NUM_CON, numPosture, 3);
		posR.setSize(NUM_CON, numPosture, 3);
		velL.setSize(NUM_CON, numPosture, 3);
		velR.setSize(NUM_CON, numPosture, 3);

		MotionLoader& skel=m_pAccurateMotion->skeleton();

		MotionUtil::GetSignals sig(*m_pAccurateMotion);
		sig.jointPosVel(dep_GetBoneFromCon(skel,CONSTRAINT_LEFT_TOE), posL[CON_TOE], velL[CON_TOE]);
		sig.jointPosVel(dep_GetBoneFromCon(skel, CONSTRAINT_RIGHT_TOE), posR[CON_TOE],   velR[CON_TOE] );
		sig.jointPosVel(skel.getBoneByRotJointIndex(jointL), posL[CON_HEEL], velL[CON_HEEL] );
		sig.jointPosVel(skel.getBoneByRotJointIndex(jointR), posR[CON_HEEL], velR[CON_HEEL] );

		matrixn m_aaHeightLeft;	
		matrixn m_aaHeightRight;

		m_aaSpeedLeft.setSize(2,numPosture);
		m_aaSpeedRight.setSize(2,numPosture);
		m_aaHeightLeft.setSize(2,numPosture);
		m_aaHeightRight.setSize(2,numPosture);
		vectorn min_height(NUM_CON);
		vectorn height_thr(NUM_CON);
		vectorn speed_thr(NUM_CON);

		for(int con=0; con<NUM_CON; con++)
		{
			m_aaSpeedLeft.row(con).aggregate(CAggregate::LENGTH, velL[con]);
			m_aaSpeedRight.row(con).aggregate(CAggregate::LENGTH, velR[con]);
			posL[con].getColumn(1, m_aaHeightLeft.row(con));
			posR[con].getColumn(1, m_aaHeightRight.row(con));
			min_height[con]=MIN(m_aaHeightLeft.row(con).minimum(), m_aaHeightRight.row(con).minimum());
		}

		height_thr[CON_TOE]=toe_height_thr+min_height[CON_TOE];
		height_thr[CON_HEEL]=heel_height_thr+min_height[CON_HEEL];
		speed_thr[CON_TOE]=toe_speed_thr;
		speed_thr[CON_HEEL]=heel_speed_thr;

		for(int con=0; con<NUM_CON; con++)
		{
			calcConstraintSep(m_pAccurateMotion, height_thr[con], speed_thr[con], posL[con], 
				m_aaSpeedLeft.row(con), m_abLeft[con], m_bCleanup);
			calcConstraintSep(m_pAccurateMotion, height_thr[con], speed_thr[con], posR[con], 
				m_aaSpeedRight.row(con), m_abRight[con], m_bCleanup);
		}

		if(bOutputImage)
		{
			vectorn min_speed(NUM_CON);
			vectorn max_speed(NUM_CON);
			min_speed.setAllValue(0.f);
			max_speed.mult(speed_thr, 2.f);


			
			CImageProcessor::SafeDelete(
				CImageProcessor::DrawChart(m_aaHeightLeft, CImageProcessor::LINE_CHART, min_height,min_height+(height_thr-min_height)*2.f,height_thr.dataPtr()), "foot_left_height.bmp");
			CImageProcessor::SafeDelete(
				CImageProcessor::DrawChart(m_aaHeightRight, CImageProcessor::LINE_CHART, min_height,min_height+(height_thr-min_height)*2.f,height_thr.dataPtr()), "foot_right_height.bmp");
			CImageProcessor::SafeDelete(
				CImageProcessor::DrawChart(m_aaSpeedLeft, CImageProcessor::LINE_CHART, min_speed, max_speed, speed_thr.dataPtr()), "foot_left_speed.bmp");
			CImageProcessor::SafeDelete(
				CImageProcessor::DrawChart(m_aaSpeedRight, CImageProcessor::LINE_CHART, min_speed, max_speed, speed_thr.dataPtr()), "foot_right_speed.bmp");
		}

		for(int i=0; i<numPosture; i++)
		{
			switch(eHowToChooseConstraint)
			{
			case CHOOSE_AND:
				m_abLeftFoot.setValue(i,m_abLeft[CON_TOE][i]&&m_abLeft[CON_HEEL][i]);
				m_abRightFoot.setValue(i,m_abRight[CON_TOE][i]&&m_abRight[CON_HEEL][i]);
				break;
			case CHOOSE_OR:
				m_abLeftFoot.setValue(i,m_abLeft[CON_TOE][i]||m_abLeft[CON_HEEL][i]);
				m_abRightFoot.setValue(i,m_abRight[CON_TOE][i]||m_abRight[CON_HEEL][i]);
				break;
			case CHOOSE_TOE:
				m_abLeftFoot.setValue(i,m_abLeft[CON_TOE][i]);
				m_abRightFoot.setValue(i,m_abRight[CON_TOE][i]);
				break;
			case CHOOSE_HEEL:
				m_abLeftFoot.setValue(i,m_abLeft[CON_HEEL][i]);
				m_abRightFoot.setValue(i,m_abRight[CON_HEEL][i]);
				break;
			}
		}

		for(int i=0; i<m_abLeftFoot.size(); i++)
		{
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_FOOT, m_abLeftFoot[i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_FOOT, m_abRightFoot[i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_TOE, m_abLeft[CON_TOE][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_TOE, m_abRight[CON_TOE][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_LEFT_HEEL, m_abLeft[CON_HEEL][i]);
			m_pAccurateMotion->setConstraint(i, CONSTRAINT_RIGHT_HEEL, m_abRight[CON_HEEL][i]);
		}

		calcConstraintPos(CONSTRAINT_LEFT_TOE);
		calcConstraintPos(CONSTRAINT_RIGHT_TOE);
	}

	void ConstraintMarking::calcConstraintSep(Motion* pMot, float cut_value, float vel_cutvalue
										, const matrixn& pos, const vectorn& aSpeed, bitvectorn& abCon, bool bCleanup)
	{
		float fFrameTime=pMot->FrameTime();

		int numPosture=pos.rows();


		abCon.clearAll();

		for(int i=0; i<numPosture; i++)
		{
			
		if(pos[i][1]<cut_value && aSpeed[i]<vel_cutvalue)
			abCon.setAt(i);

		}

		//���ൿ�ۿ��� �� �����ϴ��ǵ� �±ǵ����� ������ �־ �ɼ����� ����.
		// ���� ���ΰ͵��� ���� foot print�� �������, foot print���Ϳ� 1�̻� �־����� ���� �����ع�����.
		if(bCleanup)	
		{

			MotionUtil::SegmentFinder sf(*pMot, 0, pMot->NumFrames());


			int aargMin=0;

			for(int iseg=0; iseg<sf.numSegment(); iseg++)
			{
				cleanup(abCon, sf.startFrame(iseg), sf.endFrame(iseg), aSpeed, pos);
			}

		}

		removeShortCon(abCon);
		shortenCon(abCon);
    	
		if(m_bFillGap)
		{
		// ���� �������� �ٷ� �ٴ� ���(0.1��  ����) �� ������ �̾��ش�.
		fillGap(abCon, numPosture, ROUND(0.1/fFrameTime));

		// 0.16���� ������ ���͹��ε�, ��ġ�̵��� ���� ���� ��� �� ������ �̾��ش�.
		fillGapUsingPos(abCon, numPosture, pos, ROUND(0.16/fFrameTime), 3.0);
		}

		
	}
	
	void ConstraintMarking::removeShortCon(bitvectorn & abCon)
	{
		
		if(m_minConDuration==1) return;
		intvectorn grp;

		grp.runLengthEncode(abCon);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			if(right-left<m_minConDuration)
				for(int k=left; k<right; k++)
					abCon.clearAt(k);
		}
		
	}

	void ConstraintMarking::shortenCon(bitvectorn & abCon)
	{
		if(m_reduceCon==0) return;
		intvectorn grp;

		grp.runLengthEncode(abCon);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			if(right-left<=m_reduceCon*2)
			{
				for(int k=left; k<right; k++)
					abCon.clearAt(k);
				abCon.setAt((left+right)/2);
			}
			else
			{
				for(int k=left; k<left+m_reduceCon; k++)
					abCon.clearAt(k);

				for(int k=right-m_reduceCon; k<right; k++)
					abCon.clearAt(k);
			}
		}
	}

	void ConstraintMarking::cleanup(bitvectorn& abCon, int startFrame, int endFrame, const vectorn& aSpeed, const matrixn& pos )
	{
		intvectorn grp;

		intvectorn domain;
		vectorn footPrint(3);

		grp.runLengthEncode(abCon, startFrame, endFrame);
		for(int i=0; i<grp.size()/2; i++)
		{
			int left=grp[i*2];
			int right=grp[i*2+1];

			// foot print�� average position
			//footPrint.setAllValue(0);
			//for(int j=left; j<right; j++)
			//footPrint+=pos[j];
			//footPrint/=(m_real)(right-left);

			// foot print�� �ӵ��� ���� ������.
			domain.colon(left,right);
			int argMin=aSpeed.range(left, right).argMin()+left;
			footPrint=pos.row(argMin);

			for(int j=left; j<right; j++)
			{
				if(footPrint.distance(pos.row(j))>2.0)
					abCon.clearAt(j);
			}
		}
	}
}


