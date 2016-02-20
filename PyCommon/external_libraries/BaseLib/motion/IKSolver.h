#pragma once

class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;

namespace MotionUtil
{
	// LimbIK���� �����Ѵ�. ���� �������� IKSolver�� �ʿ��ϸ� FullbodyIK����� ��.
	class IKSolver
	{
	public:
		IKSolver(){}
		virtual~ IKSolver(){}
		// setPose�� �� skeleton�� �Է�. exact IK. �ٸ� ���� ���̱Ⱑ ���� ���� �ȵ�.
		virtual void limbIK(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot);
		// setPose�� �� skeleton�� �Է�. apporoximate IK. 
		
		// approximation 1: hip���� ������� �Ÿ��� �������Ÿ��� lengthGoal.start()�������� �ָ�, IK�� importance�� �پ���. lengthGoal.end()���� importance�� 0�̵ȴ�.
		// approximation 2: ankle���� ������� �Ÿ��� ����� ������ distGoal.start()�������� �ָ�, IK�� importance�� �پ���. 
		virtual void limbIK_heu(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot, interval const& lengthGoal, interval const& distGoal);
		static bool isIKpossible(const MotionLoader& skeleton, int con, const vector3& input_goal, m_real lengthGoal=0.95, m_real distGoal=0.3);
		
		// ���� �ٸ��� �����ִ� ������ ����Ѵ�. 1�� ������ �������¸� ���Ѵ�.
		static m_real calcIKlength(const MotionLoader& skeleton, int con);

		static m_real calcAnkleAngle(const MotionLoader& skeleton, int con);

		static void setLimb(int con, int& index_up, int& index_mid, int& index_low, vector3& axis, const MotionLoader& skeleton);
	protected:
		// solve the shoulder joint angle with the axial constraints
		// or is the original shoulder joint angle
		// ref is a solution
		// axis is the direction vector from shoulder to the goal position for wrist
		inline static quater findNearest( const quater& or, const quater& ref, const vector3& axis );
		inline static void angleRot(m_real& theta, vector3 const& v1, vector3 const& v2, quater* qk=NULL );

		inline static int limbIK( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2, 
			quater& qq1, quater& qq2, m_real ii , const vector3& axis =vector3(1,0,0), vector3* v3=NULL, vector3* v4=NULL);

		

		
	};
}