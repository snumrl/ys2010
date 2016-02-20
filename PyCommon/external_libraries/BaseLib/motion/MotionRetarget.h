#pragma once

class Motion;
class matrixn;
class vector3N;
class quaterN;
class bitvectorn;
class Bone;
class Path2D;

namespace MotionUtil
{
	class RetargetOnline2D
	{
		int mStart;
		int mRetargetQMethod;
		matrixn mCurve;
		Motion& mTarget;

		void getCurve(int start);
	public:
		enum {RETARGET_ROTY, RETARGET_DQ};

		// source�� smooth�� �������� �����Ѵ�. start�� �����Ͽ� �� ������ ���۸� �����Ѵ�.
		RetargetOnline2D(Motion& target, int start, int eRetargetQMethod=RETARGET_ROTY);
						
		// ����: time>start
		void adjust(int time, quater const& oriY, vector3 const& pos2D);
		void adjust(int time, quater const& oriY);
		void adjust(int time, vector3 const& pos2D);
		void adjust(int time, m_real deltarot);
		// ������ �������� orientation�� ��ȭ��Ű�� �ʴ´�. �� ���, time�� Ʋ������ŭ ���Ŀ� ���ƿ��� ȸ���� �߰��ȴ�.
		void adjustSafe(int time, m_real deltarot);
		// time�� time2�� �ǵ��� timewarping�Ѵ�. times�� ������ frame�� retarget�� ��� �����ӿ� �ش��ϴ����� ����Ѵ�.
		// times�� ���������� ���õǾ��ִٰ� �����Ѵ�.
		void adjust(int time, int time2, intvectorn& times);	

		// path2D�� ������ �����ӿ��� ��Ȯ�� ������ �����ϵ�, path2D�� ������ �ʹ� ũ�� �ʵ��� �߰��߰� ����Ʈ������ �߰���.
		void adjustToPath(Path2D & path, int frameThr=INT_MAX, m_real angleThr=FLT_MAX, m_real distThr=FLT_MAX);
	};
	

	//���� �ִ� stitch�Լ����� stitch�� concatŬ������� �޸�, ���� ���׸�Ʈ�� �������Ӿ� ��ġ�� �������� �����Ǿ���.
	// �� ����� �� �ֽ� �����. 

	// �������� prevSafe��ŭ�� �ٲ� �ȴ�. �������� afterSafe��ŭ�� �ٲ� �ȴ�.
	// front �� add�� �ε巴�� �����Ѵ�.
	void stitchUsingOpponent(int prevSafe, int afterSafe, Motion& front, Motion const& add);


	void stitchUsingRoot(int prevSafe, int afterSafe, Motion& front, Motion const& add);

	void exportBVH(Motion const& mot, const char* filename, int start=0, int end=INT_MAX);
}