#pragma once

#include "quater.h"
#include "template_math.h"
class quaterNView;
class quaterN : public _tvectorn<quater, m_real ,4 >
{
	// ���� ī���ؼ� �޾ƿ´�.	
	quaterN(const _tvectorn<quater, m_real ,4 >& other)	{assert(0);}	
	quaterN(const quaterNView& other)	{assert(0);}

protected:
	quaterN(m_real* ptrr, int size, int stride):_tvectorn<quater, m_real ,4 >(ptrr,size,stride){}
public:
	quaterN();
	quaterN(int n);
	quaterN(quaterN const& other);
	~quaterN();


	// L-value�� ���ɼ� �ִ�, reference array�� ����� return �Ѵ�. 
	// ex) v.range(0,2).derivative(..)
	quaterNView	range(int start, int end, int step=1);

	//void setSize(int n);
	int rows() const					{ return size();}
	quater& row(int i) const		{ return value(i);}
	//quater& operator[](int i) const	{ return row(i);}

	vectornView		w() const						{ return _column<vectornView>(0);}
	vectornView		x() const						{ return _column<vectornView>(1);}
	vectornView		y() const						{ return _column<vectornView>(2);}
	vectornView		z() const						{ return _column<vectornView>(3);}
	vectornView		column(int i) const				{ return _column<vectornView>(i);}

	void assign(const quaterN& other);

	// void derivative(const quaterN& other);

	/**
	 * Online ���� stitch�� ����� displacementMap�� ����Ѵ�.
	 * 
	 * ____ src motion
	 *     ++++ add motion
	 *     **** make displacementmap
	 *
	 * \param sq1 srcmotion[numFrame-2]
	 * \param sq2 srcmotion[numFrame-1]
	 * \param aq1 addMotion[0]
	 * \param aq2 addMotion[1]
	 * \param duration length of displacementmap
	 */
	void displacementOnline(const quater& sq1, const quater& sq2, const quater& aq1, const quater& aq2, int duration);
	void displacement(const quater& sp1, const quater& sp2, const quater& ap2, const quater& ap3, int start, int end);
	/**
	 * ab�� cd�� �����ϴ� hermite curve�� �����. �߰��� duration���� �־�����.
	 *   ab
	 * ----
	 *    -- duration -- 
	 *                 +++++++
	 *                 cd
	 *    ************** hermite range (size=duration)
	 */
	void hermite(const quater& a, const quater& b, int duration, const quater& c, const quater& d, float interval=1.f);
	/// prefer identity. see source code for more detail.
	void hermite0(const quater& a, const quater& b, int duration, const quater& c, const quater& d, float interval=1.f);

	/// size=duration a-duration-b
	void transition(const quater& a, const quater& b, int duration);
	/// prefer identity. see source code for more detail.
	void transition0(const quater& a, const quater& b, int duration);

	void c0stitch(int discontinuity);
	void c1stitch(int discontinuity);	// use hermite curve();
	void hermite(int discontinuity);
	void decomposeStitch(int discontinuity);
	void linstitch(int discontinuity);

	// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
	void linstitch(quaterN const& a, quaterN const& b);
	void c0stitch(quaterN const& a, quaterN const& b);
	void c0stitchOnline(quaterN const& a, quaterN const& b);
	void c0stitchForward(quaterN const& a, quaterN const& b);
	void c1stitch(quaterN const& a, quaterN const& b);	// use hermite curve();

	void decompose(quaterN& rotY, quaterN& offset) const;
	void combine(const quaterN& rotY, const quaterN& offset);

	void align();

	void operator=(quaterN const& );
private:
//	void release();
//	quater* m_aQuater;
//	int m_nSize;
//	int m_nCapacity;
};

class quaterNView :public quaterN
{
public:
	// L-value�� ���ɼ� �ִ�, reference array�� �����. 
	quaterNView (m_real* ptrr, int size, int stride);	

	quaterNView ():quaterN()						{Msg::error("do not call this");}
	// ���� reference�� �޾ƿ´�.
	quaterNView(const _tvectorn<quater, m_real ,4 >& other)		{ assignRef(other);}	
	quaterNView(const quaterN& other)					{ assignRef(other);}	
	quaterNView(const quaterNView& other)				{ assignRef(other);}

	~quaterNView (){}

	// L-value�� ���Ǵ� ���, ���� copy�Ѵ�.
	quaterN& operator=(const quaterN & other)			{ assign(other);return *this;}
	quaterN& operator=(const _tvectorn<quater, m_real ,4 >& other){ _tvectorn<quater, m_real ,4 >::assign(other);return *this;}
	quaterN& operator=(const quaterNView& other)		{ assign(other);return *this;}
};
