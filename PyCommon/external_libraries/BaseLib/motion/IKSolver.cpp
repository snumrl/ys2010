#include "stdafx.h"
#include "../baselib/math/mathclass.h"
#include ".\iksolver.h"
#include "motion/motion.h"
#include "motion/motionutil.h"
#include "motion/motionloader.h"

using namespace MotionUtil;


inline quater
IKSolver::findNearest( const quater& or, const quater& ref, const vector3& axis )
{
	quater org;
	if( or%ref<0 ) org = -or;
	else			org = or;
	vector3 ax;
	vector3	vs,v0;
	ax.normalize( axis );
	m_real	ws = ref.real();
	vs.imaginaries(ref);
	m_real	w0 = org.real();
	v0.imaginaries(org);
	
	m_real	a = ws*w0 + vs%v0;
	m_real	b = w0*(ax % vs) - ws*(ax % v0) + vs%(ax * v0);

	m_real	alpha = (m_real)atan( a/b );

	m_real	t1 = (m_real)(-2.0f*alpha + M_PI);
	m_real	t2 = (m_real)(-2.0f*alpha - M_PI);
	m_real	t = t2;

	quater qexp1, qexp2, ret;
	qexp1.exp((m_real)t1*ax/2.0);
	qexp2.exp((m_real)t2*ax/2.0);
	if ( ref % ( qexp1 * org )
			> ref % ( qexp2 * org ) )
		t = t1;
	ret.exp( (m_real)t*ax/2.0 );
	ret*=org;
	return ret;
}

inline void IKSolver::angleRot(m_real& theta, vector3 const& v1, vector3 const& v2, quater* qk)
{
	vector3 vn1, vn2, vn3;
	vn1.normalize(v1), vn2.normalize(v2), vn3.normalize(vn1*vn2);
	theta=(m_real)atan2( (vn1*vn2).length(), vn1%vn2 );
	if(qk) qk->exp( theta/2*vn3 );		
}


m_real Alpha(m_real t)
{
	// Kovar paper.
	// \integral alpha 
	return t*t*t*(0.5*t-1.0)+t;
}

m_real F(m_real x, m_real ro)
{
	m_real t;
	t=(x-ro)/(M_PI-ro);
	//ASSERT(t>=0.0-FERR && t<=1.0+FERR);
	return Alpha(t)*(M_PI-ro);
}
m_real integral_f(m_real a, m_real b, m_real ro)
{
	if(a>b)
		return -1*integral_f(b,a,ro);

	ASSERT(a<=b);
	ASSERT(b>=ro);

	if(a<ro)
	{
		// integral_a^ro + integral_ro^b
		return ro-a+F(b, ro); // -F(ro, ro) is omit because F(ro, ro)=0
	}
	else
		return F(b, ro)-F(a, ro);
}
/*
void integral_test()
{
	m_real ro=TO_RADIAN(160);
	m_real t;
	matrixn mat;
	for(m_real b=TO_RADIAN(160); b<M_PI; b+=0.1)
	{
		for(m_real i=0; i<M_PI; i+=0.03)
		{
			if(b>ro || i>ro)
				t=b+integral_f(b, i, ro);
			else
				t=i;
			mat.pushBack(vectorn(3, b, i, t));				
		}
	}

	mat.op0(m0::drawSignals("integral.bmp", 0,0, true));
}*/

// l1�� l2 ������ ����.  in [0, M_PI]
std::pair<m_real,bool> angle(m_real l1, m_real l2, m_real l3)
{
	m_real cos_theta=(l1*l1+l2*l2-l3*l3)/(2*l1*l2);
	if( cos_theta>1 || cos_theta<-1 )
		return std::pair<m_real, bool>(	M_PI, false);

	return std::pair<m_real, bool>(acos(cos_theta), true);
}

// limb ik solver
// goal is the goal position of wrist (global)
// sh is the shoulder position (global)
// v1 is the vector from the shoulder to the elbow (in neutral pose : offset) 
// v2 is the vector from the elbow to the wrist : offset
// qq1 is the input and output shoulder joint angle (global)
// qq2 is the input and output elbow joint angle (global)
// ii is the importance value
// axis is the elbow rotation axis
//
// v3 is the vector from the shoulder to the elbow (in current pose != offset) 
// v4 is the vector from the elbow to the wrist (in current pose)

int limbIK_new( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, 
			quater& qq1, quater& qq2, m_real ii , const vector3& axis , vector3* v3, vector3* v4)
{
	// 1�� shoulder, 2�� elbow, 3�� wrist
	vector3& d12=*v3;
	vector3& d23=*v4;
	vector3 d13=d12+d23;
	vector3 d1G=goal-sh;
	m_real d12_len=d12.length();
	m_real d23_len=d23.length();
	m_real d13_len=d13.length();
	m_real d1G_len=d1G.length();

	m_real desiredAngle=angle(d12_len, d23_len, d1G_len).first;
	m_real currAngle;
	quater q2;
	q2.axisToAxis(d23, -d12);

	vector3 axis_original;
	q2.toAxisAngle(axis_original, currAngle);


	// knee damping suggested in Foot skate cleanup by kovar
	if(v3)
	{
		m_real theta_o=currAngle;
		m_real& theta=desiredAngle;
		
		m_real dtheta=theta-theta_o;

		// f(x) = 1, x< ro
		// f(x) = 2x^3+3x^2+1, otherwise
		// theta= theta_o + \integral _theta_o ^ {theta+dtheta}  f(x) dx 

		m_real ro=TO_RADIAN(160);

		if(theta>ro || theta_o >ro )
			theta=theta_o+integral_f(theta_o, theta, ro);
	}


	quater q_delta2(currAngle-desiredAngle, axis_original);

	vector3 d23_after;
	d23_after.rotate(q_delta2, d23);

	vector3 d13_after=d12+d23_after;

	quater q_delta1;
	q_delta1.axisToAxis(d13_after, d1G);

	qq1=q_delta1*qq1;
	qq2=q_delta1*q_delta2*qq2;

	return 0;
}

// limb ik solver
// goal is the goal position of wrist (global)
// sh is the shoulder position (global)
// v1 is the vector from the shoulder to the elbow (in neutral pose : offset) 
// v2 is the vector from the elbow to the wrist : offset
// qq1 is the input and output shoulder joint angle (global)
// qq2 is the input and output elbow joint angle (global)
// ii is the importance value
// axis is the elbow rotation axis
//
// optional (only if applying knee damping)
// v3 is the vector from the shoulder to the elbow (in current pose != offset) 
// v4 is the vector from the elbow to the wrist (in current pose)
inline int
IKSolver::limbIK( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, 
			quater& qq1, quater& qq2, m_real ii , const vector3& axis , vector3* v3, vector3* v4)
{
	vector3 d, dx;
	quater qk, q1, q2;
	m_real theta, l1, l2, ccc;
	int ret=0;

	d=goal-sh;

	angleRot(theta, v1, v2, &qk); // qk.axisToAxis(v1,v2); taesoo's interpretation.
	

	m_real theta_o=theta;
	l1=v1.length(), l2=v2.length();
	ccc=(l1*l1+l2*l2-d%d)/(2*l1*l2);
	if( ccc>1 || ccc<-1 ){	theta = 0; ret=1; }
	else	
		theta=(m_real)M_PI-(m_real)acos( ccc );
	
	// knee damping suggested in Foot skate cleanup by kovar
	if(v3)
	{
		m_real theta_o;
		angleRot(theta_o, *v3, *v4);
		
		// KOVAR uses M_PI at flat leg unlike our case (where theta=0 if leg is flat).
		theta=M_PI-theta;
		theta_o=M_PI-theta_o;

		m_real dtheta=theta-theta_o;

		// f(x) = 1, x< ro
		// f(x) = 2x^3+3x^2+1, otherwise
		// theta= theta_o + \integral _theta_o ^ {theta+dtheta}  f(x) dx 

		m_real ro=TO_RADIAN(130);

		if(theta>ro || theta_o >ro )
			theta=theta_o+integral_f(theta_o, theta, ro);

		theta=M_PI-theta;
	}

	/*
	((vector3&)axis).cross(v2, v1);
	((vector3&)axis).normalize();*/
	

	q2.exp(theta/2*axis);

	dx.rotate(q2,v2);
	dx+=v1;

	quater qexp;
	angleRot(theta, dx, d, &qexp);

	d.normalize();

	q1=findNearest( qexp, qq1, d );  // 
	quater iqk;
	iqk.inverse(qk);
	q2=q1*q2*iqk;
	if( qq1%q1<0 ) q1=-q1;
	if( qq2%q2<0 ) q2=-q2;
	qq1.slerp(qq1,q1,ii);
	qq2.slerp(qq2,q2,ii);
	return ret;
}

m_real IKSolver::calcIKlength(const MotionLoader& skeleton, int con)
{
	vector3 goal;
	int index_upper, index_middle, index_lower;
	vector3 axis, currHip, currToe, currAnkle;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	m_real l1,l2;
	l1=skeleton.getBoneByRotJointIndex(index_middle).length();
	l2=skeleton.getBoneByRotJointIndex(index_lower).length();

	skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(currHip);
	
	return currAnkle.distance(currHip)/(l1+l2);
}

m_real IKSolver::calcAnkleAngle(const MotionLoader& skeleton, int con)
{
	vector3 goal;
	int index_upper, index_middle, index_lower;
	vector3 axis, currKnee, currToe, currAnkle;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	m_real l1,l2;
	l1=skeleton.getBoneByRotJointIndex(index_middle).length();
	l2=skeleton.getBoneByRotJointIndex(index_lower).length();

	skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
	dep_GetBoneFromCon(skeleton,con).getTranslation(currToe);	
	skeleton.getBoneByRotJointIndex(index_middle).getTranslation(currKnee);
	
	return (currAnkle-currToe).angle(currAnkle-currKnee);
}


bool IKSolver::isIKpossible(const MotionLoader& skeleton, int con, const vector3& input_goal, m_real lengthGoal, m_real distGoal)
{
	vector3 goal;
	int index_upper, index_middle, index_lower;
	vector3 axis, currHip, currToe, currAnkle;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	m_real l1,l2;
	l1=skeleton.getBoneByRotJointIndex(index_middle).length();
	l2=skeleton.getBoneByRotJointIndex(index_lower).length();

	if(con==CONSTRAINT_LEFT_HEEL || con==CONSTRAINT_RIGHT_HEEL)
	{
		goal=input_goal;
	}
	else
	{
		// Foot skate cleanup by kovar, page(2)
		dep_GetBoneFromCon(skeleton, con).getTranslation(currToe);
		skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
		goal=input_goal-currToe+currAnkle;
	}

	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(currHip);

	// �ٸ��� �ʹ� ������ ������ ���� �ʴ´�.
	if(goal.distance(currHip)> (l1+l2)*lengthGoal) return false;
	// �ʹ� �ָ�, �Ҽ� ����.
	if(currAnkle.distance(goal)> (l2)*distGoal) return false;
	return true;
}


void IKSolver::limbIK(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot)
{
	vector3 goal;		
	int index_upper, index_middle, index_lower;
	vector3 axis;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	index.setSize(4);
	
	index[1]=index_upper;
	index[2]=index_middle;
	index[3]=index_lower;

	vector3 l1, l2;
	skeleton.getBoneByRotJointIndex(index_middle).getOffset(l1);
	skeleton.getBoneByRotJointIndex(index_lower).getOffset(l2);

#define KNEE_DAMPING	
#ifdef KNEE_DAMPING	
	vector3 t1, t2, cv1, cv2;

	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(t1);
	skeleton.getBoneByRotJointIndex(index_middle).getTranslation(t2);
	cv1.difference(t1, t2);
	
	skeleton.getBoneByRotJointIndex(index_lower).getTranslation(t1);
	cv2.difference(t2, t1);
#endif

	quaterN origRot, rot;
	origRot.setSize(4);
	rot.setSize(4);
	for(int i=0; i<4; i++)
	{
		if(i==0)
			skeleton.getBoneByRotJointIndex(index[i+1]).parent()->getRotation(origRot[i]);
		else
			skeleton.getBoneByRotJointIndex(index[i]).getRotation(origRot[i]);
		rot[i]=origRot[i];
	}

	vector3 currToe;
	vector3 currAnkle;

	if(con==CONSTRAINT_LEFT_HEEL || con==CONSTRAINT_RIGHT_HEEL)
	{
		goal=input_goal;
	}
	else
	{
		// Foot skate cleanup by kovar, page(2)
		dep_GetBoneFromCon(skeleton, con).getTranslation(currToe);
		skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
		goal=input_goal-currToe+currAnkle;		
	}


	// limb ik solver
	// goal is the goal position of wrist (global)
	// sh is the shoulder position (global)
	// v1 is the vector from the shoulder to the elbow (in neutral pose : offset) 
	// v2 is the vector from the elbow to the wrist : offset
	// qq1 is the input and output shoulder joint angle (global)
	// qq2 is the input and output elbow joint angle (global)
	// ii is the importance value
	// axis is the elbow rotation axis
	vector3 sh;
	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(sh);

	const bool useOriginalAxisIfPossible=false;	// ���װ� �ִµ�.
	
	if(useOriginalAxisIfPossible)
	{
		vector3 axiso;
		m_real angleo;
		(rot[1].inverse()*rot[2]).toAxisAngle(axiso, angleo);
		if(axiso%axis<0)
		{
			axiso*=-1;
			angleo*=-1;
		}
		// angleo�� Ŀ������ original�� �������� axis�� ����ϵ��� �Ѵ�. �����ϸ� original detail����.
		axis.lerp(axis, axiso, CLAMP(angleo, 0.f, 1.f));

	}
#ifdef KNEE_DAMPING	
	//limbIK( goal, sh, l1, l2,	rot[1], rot[2], 1.f, axis, &cv1, &cv2);
	limbIK_new( goal, sh, l1, l2,	rot[1], rot[2], 1.f, axis, &cv1, &cv2);
#else
	limbIK( goal, sh, l1, l2,	rot[1], rot[2], 1.f, axis);
#endif

	delta_rot.setSize(4);
	delta_rot[0].identity();
	for(int i=3; i>=1; i--)
	{
		// Convert to local coordinate
		// hint: rot0*rot1*...*rotn=qqn
		// rot0=qq0
		// rot0*rot1=qq1
		// -> rot1=inv_qq0*qq1

		rot[i].leftMult(rot[i-1].inverse());
		origRot[i].leftMult(origRot[i-1].inverse());

		// calc displacement
		delta_rot[i].difference(origRot[i], rot[i]);
		delta_rot[i].align(quater(1,0,0,0));
	}
}

void IKSolver::limbIK_heu(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot, interval const& lengthGoal, interval const& distGoal)
{
	vector3 goal;		
	int index_upper, index_middle, index_lower;
	vector3 axis;
	setLimb(con, index_upper, index_middle, index_lower, axis, skeleton);

	index.setSize(4);

	index[1]=index_upper;
	index[2]=index_middle;
	index[3]=index_lower;

	vector3 l1, l2;
	skeleton.getBoneByRotJointIndex(index_middle).getOffset(l1);
	skeleton.getBoneByRotJointIndex(index_lower).getOffset(l2);

//#define KNEE_DAMPING	
#ifdef KNEE_DAMPING	
	vector3 t1, t2, cv1, cv2;

	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(t1);
	skeleton.getBoneByRotJointIndex(index_middle).getTranslation(t2);
	cv1.difference(t1, t2);
	
	skeleton.getBoneByRotJointIndex(index_lower).getTranslation(t1);
	cv2.difference(t2, t1);
#endif

	quaterN origRot, rot;
	origRot.setSize(4);
	rot.setSize(4);
	for(int i=0; i<4; i++)
	{
		if(i==0)
			skeleton.getBoneByVoca(index[i]).getRotation(origRot[i]);
		else
			skeleton.getBoneByRotJointIndex(index[i]).getRotation(origRot[i]);
		rot[i]=origRot[i];
	}

	vector3 currToe;
	vector3 currAnkle;

	if(con==CONSTRAINT_LEFT_HEEL || con==CONSTRAINT_RIGHT_HEEL)
	{
		goal=input_goal;
	}
	else
	{
		// Foot skate cleanup by kovar, page(2)
		dep_GetBoneFromCon(skeleton, con).getTranslation(currToe);
		skeleton.getBoneByRotJointIndex(index_lower).getTranslation(currAnkle);
		goal=input_goal-currToe+currAnkle;		
	}

	// limb ik solver
	// goal is the goal position of wrist (global)
	// sh is the shoulder position (global)
	// v1 is the vector from the shoulder to the elbow (in neutral pose : offset) 
	// v2 is the vector from the elbow to the wrist : offset
	// qq1 is the input and output shoulder joint angle (global)
	// qq2 is the input and output elbow joint angle (global)
	// ii is the importance value
	// axis is the elbow rotation axis
	vector3 sh;
	skeleton.getBoneByRotJointIndex(index_upper).getTranslation(sh);

	m_real ii1=1.0, ii2=1.0;
	
	// �ٸ��� �ʹ� ������ ������ ���� �ʴ´�.
	interval zeroToOne(0.0, 1.0);
	m_real distToHip=goal.distance(sh);
	m_real distToAnkle=currAnkle.distance(goal);
	m_real ll2=l2.length();
	if(distToHip> (l1.length()+ll2)*lengthGoal.start_pt())
		ii1=zeroToOne.project(1.0-lengthGoal.uninterpolate(distToHip));
	
	// �ʹ� �ָ�, �Ҽ� ����.
	if(distToAnkle> (ll2)*distGoal.start_pt())
		ii2=zeroToOne.project(1.0-distGoal.uninterpolate(distToAnkle));

	//test
	vector3 axiso;
	m_real angleo;
	(rot[1].inverse()*rot[2]).toAxisAngle(axiso, angleo);
	if(axiso%axis<0)
	{
		axiso*=-1;
		angleo*=-1;
	}
	// angleo�� Ŀ������ original�� �������� axis�� ����ϵ��� �Ѵ�. �����ϸ� original detail����.
	axis.lerp(axis, axiso, CLAMP(angleo, 0.f, 1.f));

#ifdef KNEE_DAMPING	
	limbIK( goal, sh, l1, l2,	rot[1], rot[2], ii1*ii2, axis, &cv1, &cv2);
#else
	limbIK( goal, sh, l1, l2,	rot[1], rot[2], ii1*ii2, axis);
#endif

	delta_rot.setSize(4);
	delta_rot[0].identity();
	for(int i=3; i>=1; i--)
	{
		// Convert to local coordinate
		// hint: rot0*rot1*...*rotn=qqn
		// rot0=qq0
		// rot0*rot1=qq1
		// -> rot1=inv_qq0*qq1

		rot[i].leftMult(rot[i-1].inverse());
		origRot[i].leftMult(origRot[i-1].inverse());

		// calc displacement
		delta_rot[i].difference(origRot[i], rot[i]);
		delta_rot[i].align(quater(1,0,0,0));
	}
}

void IKSolver::setLimb(int con, int& index_up, int& index_mid, int& index_low, vector3& axis, const MotionLoader& skeleton)
{
	if(con==CONSTRAINT_LEFT_TOE || con==CONSTRAINT_LEFT_HEEL)
		con=CONSTRAINT_LEFT_FOOT;

	if(con==CONSTRAINT_RIGHT_TOE || con==CONSTRAINT_RIGHT_HEEL)
		con=CONSTRAINT_RIGHT_FOOT;

	if(con==CONSTRAINT_LEFT_FINGERTIP)
		con=CONSTRAINT_LEFT_HAND;
	if(con==CONSTRAINT_RIGHT_FINGERTIP)
		con=CONSTRAINT_RIGHT_HAND;

	if(con==CONSTRAINT_LEFT_HAND || con==CONSTRAINT_RIGHT_HAND)
		axis.setValue(-1,0,0);
	else
		axis.setValue(1,0,0);

	int voca_up, voca_mid, voca_low;
	if(con == CONSTRAINT_LEFT_FOOT) 
	{
		voca_up=MotionLoader::LEFTHIP;
		voca_mid=MotionLoader::LEFTKNEE;
		voca_low=MotionLoader::LEFTANKLE;		
	}
	else if(con == CONSTRAINT_RIGHT_FOOT)
	{
		voca_up=MotionLoader::RIGHTHIP;
		voca_mid=MotionLoader::RIGHTKNEE;
		voca_low=MotionLoader::RIGHTANKLE;
	}
	else if(con ==  CONSTRAINT_LEFT_HAND)
	{
		voca_up=MotionLoader::LEFTSHOULDER;
		voca_mid=MotionLoader::LEFTELBOW;
		voca_low=MotionLoader::LEFTWRIST;
	}
	else if(con == CONSTRAINT_RIGHT_HAND)
	{
		voca_up=MotionLoader::RIGHTSHOULDER;
		voca_mid=MotionLoader::RIGHTELBOW;
		voca_low=MotionLoader::RIGHTWRIST;
	}
	else TRACE("error in contraint type!");

	index_up=skeleton.getRotJointIndexFromVoca(voca_up);		ASSERT(index_up!= -1);
	index_mid= skeleton.getRotJointIndexFromVoca(voca_mid);	ASSERT(index_mid!= -1);
	index_low= skeleton.getRotJointIndexFromVoca(voca_low);	ASSERT(index_low!= -1);	
}

