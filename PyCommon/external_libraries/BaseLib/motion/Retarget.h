#pragma once

class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
namespace MotionUtil
{
	class Retarget
	{
	public:
		Retarget(Motion& mot):m_mot(mot){}
		virtual ~Retarget(void){}
		// ������ [startSafe, endSafe)�� �����ؼ� [startFrame, endFrame) ���̿��� ��Ȯ�ϰ� constraint�� �����ǵ��� �Ѵ�.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);
		Motion& m_mot;
	};


	// IK�� �� �����ϴ��� �׽�Ʈ �Ѵ�. (perframe IK only)
	class RetargetTest : public Retarget
	{
	public:
		RetargetTest(Motion& mot):Retarget(mot){}
		virtual ~RetargetTest(){}

		// ������ [startSafe, endSafe)�� �����ؼ� [startFrame, endFrame) ���̿��� ��Ȯ�ϰ� constraint�� �����ǵ��� �Ѵ�.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};


	class Retarget2: public Retarget
	{
	public:
		Retarget2(Motion& mot):Retarget(mot){}
		virtual ~Retarget2(){}

		// ������ [startSafe, endSafe)�� �����ؼ� [startFrame, endFrame) ���̿��� ��Ȯ�ϰ� constraint�� �����ǵ��� �Ѵ�.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};

}