
#include "stdafx.h"
#include "../baselib/math/mathclass.h"
#include "footprint.h"
#include "motion/motion.h"
#include "motion/motionutil.h"
#include "motion/motionloader.h"
#include "motion/iksolver.h"
#include "ConstraintMarking.h"
#include "../math/intervals.h"
#include "../math/operator.h"
using namespace MotionUtil;
void FootPrint::getFootInterval(const Motion& mot, int iframe, int constraint, int& left, int& right) const
{
	if(mot.isConstraint(iframe, constraint))
	{
		bool bBrake=false;		
		for(left=iframe; !bBrake && left>=0 && mot.isConstraint(left, constraint); left--)
		{
			if(mot.IsDiscontinuous(left))
				bBrake=true;
		}
		left++;

		for(right=iframe+1; right<mot.NumFrames()  && !mot.IsDiscontinuous(right) && mot.isConstraint(right, constraint); right++);

		ASSERT(right>left);
	}
	else
	{
		Msg::error("%d Is not constrained by con %d", iframe, constraint);
	}
}

void CalcFootPrint::getFootPrints(const Motion& mot, int start, int end, int constraint, intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=conInterval[grp*2];
		int end=conInterval[grp*2+1];

		vector3 conPos(0,0,0);
		vector3 pos;
						
		for(int i=start; i<end; i++)
		{
			mot.skeleton().setPose(mot.Pose(i));			
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				
			bone.getTranslation(pos);
			conPos+=pos;						
		}
		conPos/=float(end-start);

		aFootPositions.row(grp).assign(conPos);		

	}
}

void GetFootPrint::getFootPrints(const Motion& mot, int start, int end, int constraint, intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int start=conInterval[grp*2];
		int end=conInterval[grp*2+1];

		vector3 conPos(0,0,0);
						
		for(int i=start; i<end; i++)
		{
			conPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
		}
		conPos/=float(end-start);

		aFootPositions.row(grp).assign(conPos);		
	}	
}

GetFootPrintOnline::GetFootPrintOnline(m_real minHeight, m_real maxHeight, m_real lengthThr, m_real distThr)
:mHeightInterval(minHeight, maxHeight),
mLengthThr(lengthThr),
mDistThr(distThr)
{
	
	
}

void GetFootPrintOnline::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	intIntervals cons;

	cons.runLengthEncode(conToe);
	cons.offset(start);

	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);
		for(int i=startCon; i<endCon; i++)
			Msg::verify(mot.isConstraint(i, constraint), "???");
	}

	aFootPositions.setSize(cons.size(),3);
	
	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);
		bool bContinuedCon=false;
		vector3 conPos(0,0,0);
		vector3 startPos;
		mot.skeleton().setPose(mot.Pose(startCon));
		Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
		bone.getTranslation(startPos);

		if(startCon==start && start!=0 && mot.isConstraint(start-1, constraint))
		{
			//printf("continued con %d\n", start);

			/*
			mot.skeleton().setPose(mot.Pose(start-1));

			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);*/
			
			// global ��ǥ�迡�� ����Ǿ��ִٰ� �����Ѵ�.
			
			conPos=mot.Pose(start-1).conPosition(constraint);
			bContinuedCon=true;
		}
		else
		{
			vector3 pos;
			vector3 startConPos;
			vector3 avgPos(0,0,0);
			
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.Pose(i));
				mot.Pose(i).conPosition(constraint)=vector3(0,100,0);	// for debug
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				bone.getTranslation(pos);

				if(i==startCon) 
				{
					startConPos=pos;
				}

				//avgPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
				avgPos+=pos;
			}
			avgPos/=float(endCon-startCon);
					
			// 3 ������ �̻��� ������ ������, �׳� avgPos���
			m_real t=sop::clampMap(startCon, start, start+3);
			
            conPos.lerp(startConPos, avgPos, t);

			conPos=avgPos;
			conPos.y=mHeightInterval.project(conPos.y);
		
		}
		
		/*if(endCon<=startCon)//if(conPos.y>5 || endCon<=startCon)// || conPos.distance(startPos)>10)
		{
			cons.removeInterval(grp);
			grp--;			
		}
		else*/
		{
			aFootPositions.row(grp).assign(conPos);
		}
	}

	aFootPositions.resize(cons.size(),3);
	cons.encodeIntoVector(conInterval);

	/*
	// �ٲ�� ����.
	// save constraint (������ �̾�ٿ����� ����)
	for(int j=start; j<end; j++)
		((Motion&)mot).setConstraint(j, constraint, false);

	for(int i=0; i<cons.numInterval(); i++)
	{
		for(int j=cons.start(i); j< cons.end(i); j++)
			((Motion&)mot).setConstraint(j, constraint, true);
	}*/

}



GetFootPrintOnlineHeu::GetFootPrintOnlineHeu(m_real minHeight, m_real maxHeight, m_real lengthThr, m_real distThr)
:mHeightInterval(minHeight, maxHeight),
mLengthThr(lengthThr),
mDistThr(distThr)
{
	
	
}

void GetFootPrintOnlineHeu::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	intIntervals cons;

	cons.runLengthEncode(conToe);
	cons.offset(start);

	aFootPositions.setSize(cons.size(),3);
	
	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);

		vector3 conPos(0,0,0);
		vector3 startPos;
		mot.skeleton().setPose(mot.Pose(startCon));
		Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
		bone.getTranslation(startPos);

		if(startCon==start && start!=0 && mot.isConstraint(start-1, constraint))
		{
			mot.skeleton().setPose(mot.Pose(start-1));

			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);

			// adjust con time and pos
			for(int i=startCon; i<endCon; i++)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.skeleton().setPose(mot.Pose(i));

				if(!IKSolver::isIKpossible(mot.skeleton(), constraint, conPos, mLengthThr, mDistThr))
				{
					endCon=i;
					break;
				}

				vector3 trans;
				bone.getTranslation(trans);
				/*if(trans.y>mHeightThr)
				{
					endCon=i;
					break;
				}*/
			}
		}
		else
		{
			vector3 pos;
			vector3 startConPos;
			vector3 avgPos(0,0,0);
			
			for(int i=startCon; i<endCon; i++)
			{
				if(i==startCon) 
				{
					mot.skeleton().setPose(mot.Pose(i));
					Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
					bone.getTranslation(pos);
					startConPos=pos;
				}

				avgPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
			}
			avgPos/=float(endCon-startCon);
					
            m_real t=MIN((startCon-start)/(MIN((end-start)/2.f,4)), 1.f);
			s1::SMOOTH_TRANSITION(t, t);
			
            conPos.lerp(startConPos, avgPos, t);

			conPos=avgPos;
			conPos.y=mHeightInterval.project(conPos.y);
			
			int argMin=s2::AVG(startCon, endCon);
			for(int i=startCon; i<argMin-1; i++)
			{
				// �ʹ� �־����� constraint�� ����.	(���� ��Ÿ�����ؼ� �����ϰ� ����� ����)			
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					startCon=i;
					break;
				}
			}

			for(int i=endCon-1; i>argMin+1; i--)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					endCon=i+1;
					break;
				}
			}
		}
		
		if(conPos.y>5 || endCon<=startCon)// || conPos.distance(startPos)>10)
		{
			cons.removeInterval(grp);
			grp--;			
		}
		else
		{
			vector3 pos;
			vector3 avgPos(0,0,0);
			vector3 startConPos;

/*			int dur=endCon-startCon;
			{
				startCon+=(dur+6)/10;	// duration�� �յڷ� 10%���� �ٿ��ش�. duration�� 4�̸� ������ duration�� 2�� �ȴ�. (3�̸� 2, 2���ϸ� ����)
				endCon-=(dur+7)/10;
			}*/

			for(int i=startCon; i<endCon; i++)
			{
				if(i==startCon) 
				{
					mot.skeleton().setPose(mot.Pose(i));
					Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
					bone.getTranslation(pos);
					startConPos=pos;
				}

				avgPos+=MotionUtil::ConstraintMarking::decodeCon(mot, i, constraint);
			}
			avgPos/=float(endCon-startCon);
			if(avgPos.y<0) avgPos.y=0;

			m_real t=MIN((startCon-start)/(MIN((end-start)/2.f,4)), 1.f);
			s1::SMOOTH_TRANSITION(t, t);

			conPos.lerp(startConPos, avgPos, t);
			conPos.y=mHeightInterval.project(conPos.y);

			aFootPositions.row(grp).assign(conPos);
		}
	}

	aFootPositions.resize(cons.size(),3);
	cons.encodeIntoVector(conInterval);

	// save constraint (������ �̾�ٿ����� ����)
	for(int j=start; j<end; j++)
		((Motion&)mot).setConstraint(j, constraint, false);

	for(int i=0; i<cons.numInterval(); i++)
	{
		for(int j=cons.start(i); j< cons.end(i); j++)
			((Motion&)mot).setConstraint(j, constraint, true);
	}

}

void CalcFootPrintSpeed::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	conInterval.runLengthEncode(conToe);
	conInterval+=start;

	int numConGrp=conInterval.size()/2;
	aFootPositions.setSize(numConGrp,3);
	
	for(int grp=0; grp<numConGrp; grp++)
	{
		int startCon=conInterval[grp*2];
		int endCon=conInterval[grp*2+1];

		vector3 conPos(0,0,0);

		if(startCon==start)
		{
			mot.skeleton().setPose(mot.Pose(startCon));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);
		}
		else if(endCon==end)
		{
			mot.skeleton().setPose(mot.Pose(endCon-1));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);
		}
		else
		{
			vector3 pos;
							
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.Pose(i));			
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
					
				bone.getTranslation(pos);
				conPos+=pos;						
			}
			conPos/=float(endCon-startCon);
		}

		aFootPositions.row(grp).assign(conPos);

	}
}

void CalcFootPrintOnline::heuristicFootDetection(const Motion& mot, int start, int end, int constraint)
{
	MotionUtil::GetSignal getSignal(mot);

	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	//�������� ���� ����̹Ƿ� �������� ������ �ְ� �̸� �������ش�.
	// ���� �������� �ٷ� �ٴ� ���(0.1��  ����) �� ������ �̾��ش�.
	//ConstraintMarking::fillGap(conToe, conToe.size(), ROUND(0.1/mot.FrameTime()));
	intIntervals cons;

	cons.runLengthEncode(conToe);
	cons.offset(start);

	vector3 conPos, pos;
	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);

		if(startCon==start && start>0 && mot.isConstraint(start-1, constraint))
		{
			// ������ġ�� conpos
			mot.skeleton().setPose(mot.Pose(start-1));
			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);

			// adjust con time and pos
			for(int i=startCon; i<endCon; i++)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.skeleton().setPose(mot.Pose(i));

				if(!IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					endCon=i;
					break;
				}
			}
		}
		else
		{
			vector3 avgPos(0,0,0);
			// �����ġ�� conpos
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.Pose(i));
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				bone.getTranslation(pos);
				avgPos+=pos;						
			}
			avgPos/=float(endCon-startCon);
			conPos=avgPos;
			int argMin=(startCon+endCon)/2;
			for(int i=startCon; i<argMin-1; i++)
			{
				// �ʹ� �־����� constraint�� ����.	(���� ��Ÿ�����ؼ� �����ϰ� ����� ����)			
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					startCon=i;
					break;
				}
			}

			for(int i=endCon-1; i>argMin+1; i--)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					endCon=i+1;
					break;
				}
			}

		}

		if(endCon<=startCon)
		{
			cons.removeInterval(grp);
			grp--;			
		}
	}

	// save constraint
	for(int j=start; j<end; j++)
		((Motion&)mot).setConstraint(j, constraint, false);

	for(int i=0; i<cons.numInterval(); i++)
	{
		for(int j=cons.start(i); j< cons.end(i); j++)
			((Motion&)mot).setConstraint(j, constraint, true);
	}

}

void CalcFootPrintOnline::getFootPrints(const Motion& mot,int start, int end, int constraint, 
									intvectorn& conInterval, matrixn& aFootPositions) const
{

	
	MotionUtil::GetSignal getSignal(mot);
		
	bitvectorn conToe;
	getSignal.constraint(constraint, conToe, start, end);

	if(m_bSupportFootOnly)
	{
		for(int i=0; i<conToe.size(); i++)
		{
			bool constraint=conToe[i];
			bool desiredSupportFoot;
			if(constraint==CONSTRAINT_LEFT_TOE || constraint==CONSTRAINT_LEFT_HEEL || constraint==CONSTRAINT_LEFT_FOOT)
				desiredSupportFoot=L_IS_SUPPORTED;
			else
				desiredSupportFoot=R_IS_SUPPORTED;

			if(desiredSupportFoot!=mot.isConstraint(i, WHICH_FOOT_IS_SUPPORTED))
				conToe.clearAt(i);
		}
	}
	intIntervals cons;

	cons.runLengthEncode(conToe);
	cons.offset(start);

	aFootPositions.setSize(cons.size(),3);
	
	for(int grp=0; grp<cons.size(); grp++)
	{
		int startCon=cons.start(grp);
		int endCon=cons.end(grp);

		vector3 conPos(0,0,0);
		vector3 startPos;
		mot.skeleton().setPose(mot.Pose(startCon));
		Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
		bone.getTranslation(startPos);

		if(startCon==start && start!=0 && mot.isConstraint(start-1, constraint))
		{
			mot.skeleton().setPose(mot.Pose(start-1));

			Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
			bone.getTranslation(conPos);

			// adjust con time and pos
			for(int i=startCon; i<endCon; i++)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.skeleton().setPose(mot.Pose(i));

				if(!IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					endCon=i;
					break;
				}

				vector3 trans;
				bone.getTranslation(trans);
				if(trans.y>4)
				{
					endCon=i;
					break;
				}
			}
		}
		else
		{
			vector3 pos;
			vector3 startConPos;
			vector3 avgPos(0,0,0);
			static matrixn footPos;
			footPos.resize(endCon-startCon,3);
			
			for(int i=startCon; i<endCon; i++)
			{
				mot.skeleton().setPose(mot.Pose(i));
				Bone& bone=dep_GetBoneFromCon(mot.skeleton(),constraint);
				bone.getTranslation(pos);
				footPos.row(i-startCon).assign(pos);
				if(i==startCon) startConPos=pos;
				avgPos+=pos;						
			}
			avgPos/=float(endCon-startCon);

            m_real t=MIN((startCon-start)/(MIN((end-start)/2.f,4)), 1.f);
			Msg::output("t", "%f",t);
			s1::SMOOTH_TRANSITION(t, t);

			
            conPos.lerp(startConPos, avgPos, t);
			
			// adjust con time and pos

			m_real minDist=FLT_MAX;
			int argMin;
			for(int i=startCon; i<endCon; i++)
			{
				m_real dist=footPos.row(i-startCon).toVector3().distance(conPos);
				if(dist<minDist)
				{
					minDist=dist;
					argMin= i;
				}
			}

			if(minDist>5) 
			{
				argMin=(endCon+startCon)/2;
				conPos=footPos.row(argMin-startCon).toVector3();
			}


			if(conPos.y<0) conPos.y=0;
			//if(conPos.y>4) conPos.y=4;
			
			for(int i=startCon; i<argMin-1; i++)
			{
				// �ʹ� �־����� constraint�� ����.	(���� ��Ÿ�����ؼ� �����ϰ� ����� ����)			
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					startCon=i;
					break;
				}
			}

			for(int i=endCon-1; i>argMin+1; i--)
			{
				// �ʹ� �־����� constraint�� ����.
				mot.setSkeleton(i);
				if(IKSolver::isIKpossible(mot.skeleton(), constraint, conPos))
				{
					endCon=i+1;
					break;
				}
			}
		}

		if(conPos.y>5 || endCon<=startCon)// || conPos.distance(startPos)>10)
		{
			cons.removeInterval(grp);
			grp--;			
		}
		else
		{
			aFootPositions.row(grp).assign(conPos);
		}
	}

	aFootPositions.resize(cons.size(),3);
	cons.encodeIntoVector(conInterval);

	// save constraint (������ �̾�ٿ����� ����)
	for(int j=start; j<end; j++)
		((Motion&)mot).setConstraint(j, constraint, false);

	for(int i=0; i<cons.numInterval(); i++)
	{
		for(int j=cons.start(i); j< cons.end(i); j++)
			((Motion&)mot).setConstraint(j, constraint, true);
	}

}
