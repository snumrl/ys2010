#pragma once
class Motion;


namespace MotionUtil
{
	//���� �ִ� stitch�Լ����� stitch�� concatŬ������� �޸�, ���� ���׸�Ʈ�� �������Ӿ� ��ġ�� �������� �����Ǿ���.

	// front.pose(frontLast)=src.pose(srcFirst)
	// front.pose(frontLast+1)=src.pose(srcFirst+1)
	// front.pose(frontLast+2)=src.pose(srcFirst+2)
	// ... 
	// front.pose(frontLast+srcLast-srcFirst)=src.pose(srcLast)
	// �̷����̵ǵ��� ��Ƽġ �ȴ�. 

	// ������ [firstSafe, lastSafe]�� �ε巴�� �����Ǿ� �ٴ´�.
	// [..] interval�� ����Ҷ��� first, last
	// [..) interval�� ����Ҷ��� start, end �� ǥ���Ѵ�. 
	void stitchGlobal(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX, bool two=true);
	void stitchGlobalC2(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX, bool two=true);

	// �⺻������ ���� ����������, ���� �ڿ� tail��ŭ �� �ٿ����´�. (������ ������ �� �ֵ���.)
	void stitchBlend(int firstSafe, int lastSafe, Motion& front, Motion const& src, int srcFirst, int srcLast, int frontLast, int tail);

	void nostitch(Motion& front, Motion const& src, int srcfirst, int srclast);

	// ����.
	// front�� ������ �������� src.Pose(transitionFrom)�� �ش��Ѵ�.  (������ ������ �ʿ�� ������, �ּ��� �ſ� �����Ͽ����Ѵ�.)
	//
	//                                transitionFrom
	//  front...............................|
	//  src                                 |..................|
	//                                 srcFirst              srcLast
	//  searchRange               |                  |
	//					      firstSafe				lastSafe
	void stitchTransition(int firstSafe, int lastSafe, Motion& front, int transitionFrom, Motion const& src, int srcFirst, int srcLast, int frontLast=INT_MAX);	



	void inpaint(int startInpaint, int endInpaint, Motion& front);
}


//////////////// deprecated. �Ʒ��� �ִ� Ŭ�������� ������� ����. 


#include <typeinfo.h>
namespace MotionUtil
{
	//���� �ִ� stitch�Լ����� stitch�� concatŬ������� �޸�, ���� ���׸�Ʈ�� �������Ӿ� ��ġ�� �������� �����Ǿ���.
	// �� ����� �� �ֽ� �����. 

	// �������� prevSafe��ŭ�� �ٲ� �ȴ�. �������� afterSafe��ŭ�� �ٲ� �ȴ�.
	// front �� add�� �ε巴�� �����Ѵ�.
	void stitchLocal(int prevSafe, int afterSafe, Motion& front, Motion const& add);
	void stitchGlobal(int prevSafe, int afterSafe, Motion& front, Motion const& add);
	void stitchOnline(Motion& front, Motion const& add);
	

	/// Stitch�� discontinuity�� ���ִ� ������� �̾���δ�.
	class Stitch
	{
	public:
		Stitch(){}
		virtual ~Stitch(){}

		  
//
//		������ ��������� propagate�ȴ�.
//		 * \param safe 
//		 �ֺ����� error�� propagate�ص� �Ǵ� �����̴�.
//		 front.length()+-safe�� �� valid index��� ������ �����Ƿ�, ����� Ŭ�������� ������, valid index�� �ƴϾ ������ ���� �ʵ���
//		 �����Ұ�.
//
		virtual void stitch(int safe, Motion& front, const Motion& add)	
			{ Msg::error("%s is not implemented", typeid( *this).name()); }

		
		//
		//������ �������θ� propagate�ȴ�.
		 //* \param safe 
		 //�������� error�� propagate�ص� �Ǵ� �����̴�.
		 //front.length()+-safe�� �� valid index��� ������ �����Ƿ�, ����� Ŭ�������� ������, valid index�� �ƴϾ ������ ���� �ʵ���
		 //�����Ұ�.
		 //
		virtual void stitchOnline(int safe, Motion& front, const Motion& add)	
			{ Msg::error("%s is not implemented", typeid( *this).name()); }
	};


	class Concat;
	// C1 continuous stitch- alwas bettern than C0stitch above.
	class C1stitch : public Stitch
	{
		const Concat& m_concator;
	public:
		C1stitch(const Concat& concator):m_concator(concator){}
		virtual ~C1stitch(){}
		
		virtual void stitch(int safe, Motion& front, const Motion& add);
		virtual void stitchOnline(int safe, Motion& front, const Motion& add);		
	};

	// C1 continuous root-special (root is specially stitched)
	class C1stitch2 : public Stitch
	{
		const Concat& m_concator;
	public:
		C1stitch2(const Concat& concator):m_concator(concator){}
		virtual ~C1stitch2(){}

		virtual void stitch(int safe, Motion& front, const Motion& add);
		static void stitchUtilRoot(int safe, Motion& afterConcat, int numFrameOld);
		static void stitchUtilJoint(int safe, Motion& afterConcat, int numFrameOld);
		static void stitchUtil(int safe, Motion& afterConcat, int numFrameOld);
	};

	namespace linstitch 
	{
		void stitchUtilRoot(int safe, Motion& afterConcat, int numFrameOld);
		void stitchUtilJoint(int safe, Motion& afterConcat, int numFrameOld);	
	}
	// C1 continuous stitch using stitching the displacementMap.
	class C1stitchReconstruct : public Stitch
	{
	public:
		C1stitchReconstruct (){}
		virtual ~C1stitchReconstruct (){}

        virtual void stitch(int safe, Motion& front, const Motion& add);		
	};

	class StitchRetarget: public Stitch
	{
	public:
		StitchRetarget(){}
		virtual ~StitchRetarget(){}
        virtual void stitch(int safe, Motion& front, const Motion& add);
	};

	class NoStitch: public Stitch
	{
		const Concat& m_concator;
	public:
		NoStitch(const Concat& concator):m_concator(concator){}
		virtual ~NoStitch(){}

		virtual void stitch(int safe, Motion& front, const Motion& add);
	};

	namespace C0stitch 
	{
		void stitchUtil(int safe, Motion& front, int numFrameOld);
	};

}