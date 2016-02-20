#pragma once

class Motion;

namespace MotionUtil
{
	/// concat�� ���� ��ǰ� ���� ����� ���̴� ����� �Ѵ�. �� ������ �ڼ��� ������ �ʰ� ���� discontinuity�� �����ִ�.
	class Concat
	{
	public:
		Concat(){}
		virtual ~Concat(){}

		virtual void concat(Motion& front, const Motion& add) const { Msg::error("%s is not implemented", typeid(*this).name());}
	};

	class ReconstructConcat : public Concat
	{
	public:
		ReconstructConcat(){}
		virtual ~ReconstructConcat (){}
		
		virtual void concat(Motion& front, const Motion& add) const;
	};

	class ReconstructAdjustConcat : public Concat
	{
	public:
		ReconstructAdjustConcat(){adjustAngleY=0.f; adjustAngleZ=0.f; adjustAngleYFirst=0.f;}
		virtual ~ReconstructAdjustConcat (){}

		float adjustAngleYFirst;	
		float adjustAngleY;	// vertical axis
		float adjustAngleZ;	// forward axis
		virtual void concat(Motion& front, const Motion& add) const;
	};

	class ReconstructAdjustConcatTwoWay : public Concat
	{
	public:
		ReconstructAdjustConcatTwoWay(){adjustAngleY=0.f; }
		virtual ~ReconstructAdjustConcatTwoWay (){}

		float adjustAngleY;	// vertical axis
		int safe;
		virtual void concat(Motion& front, const Motion& add) const;
	};

	class RootTrajectoryConcat : public Concat
	{
	public:
		RootTrajectoryConcat(){}
		virtual ~RootTrajectoryConcat (){}

		virtual void concat(Motion& front, const Motion& add) const;
	};

	class RootOrientationConcat : public Concat
	{
	public:
		RootOrientationConcat (){}
		virtual ~RootOrientationConcat (){}

		virtual void concat(Motion& front, const Motion& add) const;
	};

	class FootPrint;

	class FootAdjustConcat : public Concat
	{
	public:
		FootAdjustConcat (const Concat& firstConcat, const FootPrint& footprint):mFirstConcat(firstConcat), mFootPrint(footprint){}
		virtual ~FootAdjustConcat (){}

		virtual void concat(Motion& front, const Motion& add) const;
		const Concat& mFirstConcat;
		const FootPrint& mFootPrint;
	};
}