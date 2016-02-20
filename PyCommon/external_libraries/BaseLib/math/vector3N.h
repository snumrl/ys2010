#pragma once

#include "vector.h"
class matrixn;
class vector3NView;

#pragma message("Compiling vector3N.h - this should happen just once per project.\n")

class vector3N : public _tvectorn<vector3, m_real, 3>
{
	// ���� ī���ؼ� �޾ƿ´�.	
	vector3N(const _tvectorn<vector3, m_real, 3>& other)	{assert(0);}		
	vector3N(const vector3NView& other)	{assert(0);}
protected:
	vector3N(m_real * ptrr, int size, int stride):_tvectorn<vector3, m_real, 3>(ptrr,size,stride){}
public:
	vector3N();
	vector3N(int n);
	vector3N(const vector3N& other):_tvectorn<vector3, m_real, 3>()	{assign(other);}
	~vector3N();
	
	int rows() const						{ return size();}
	vector3& row(int i) const			{ return value(i);}
	vector3& operator[](int i) const	{ return row(i);}
	void assign(const vector3N& other);
	void assign(const matrixn& other);

	// L-value�� ���ɼ� �ִ�, reference array�� ����� return �Ѵ�. 
	// ex) v.range(0,2).derivative(..)
	vector3NView	range(int start, int end, int step=1);

	// L-value�� ���ɼ� �ִ� reference vector �� return�Ѵ�.
	vectornView		x() const						{ return _column<vectornView>(0);}
	vectornView		y() const						{ return _column<vectornView>(1);}
	vectornView		z() const						{ return _column<vectornView>(2);}
	vectornView		column(int i) const				{ return _column<vectornView>(i);}

	void translate(const vector3& trans);
	void rotate(const quater& q);		 //!< Data ��ü�� Rotate�Ѵ�. root orientation quat�տ� �������� ���� ���߿�

	void rotate(const vector3& center, const quater&q)
	{
		translate(-center);
		rotate(q);
		translate(center);
	}
	// void derivative(const vector3N& other);

	/**
	 * Online ���� stitch�� ����� displacementMap�� ����Ѵ�.
	 * 
	 * ____ src motion
	 *     ++++ add motion
	 *     **** make displacementmap
	 *
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param duration length of displacementmap
	 */
	void displacementOnline(const vector3& sp1, const vector3& sp2, const vector3& ap1, const vector3& ap2, int duration);

	/**
	 * Offline ���� stitch�� ����� displacementMap�� ����Ѵ�.
	 * 
	 * ____ src motion
	 *     ++++ add motion
	 *   **** make displacementmap
	 *
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param start: For example, in the above picture, start = -2 
	 * \param end : ... end=2
	 */
	void displacement(const vector3& sp1, const vector3& sp2, const vector3& ap1, const vector3& ap2, int start, int end);

	/**
	 * Offline ���� stitch�� ����� displacementMap�� ����Ѵ�. Better stitching. 
	 * 
	 * _____ src motion
	 *      +++++ add motion
	 *  ******** make displacementmap
	 * (sps �� spe�� �߰��� speed�� ���� �� estimate�ϱ� ���� ����)
	 * \param sps srcmotion[numFrame+start]
	 * \param sp1 srcmotion[numFrame-2]
	 * \param sp2 srcmotion[numFrame-1]
	 * \param ap1 addMotion[0]
	 * \param ap2 addMotion[1]
	 * \param ape addMotion[end-1]
	 * \param start: For example, in the above picture, start = -4
	 * \param end : ... end=4
	 */
	void displacement(const vector3& sps, const vector3& sp1, const vector3& sp2, 
						const vector3& ap1, const vector3& ap2, const vector3& spe, int start, int end);

	/**
	 * ab�� cd�� �����ϴ� hermite curve�� �����. �߰��� duration���� �־�����.
	 *   ab
	 * ----
	 *    -- duration -- 
	 *                 +++++++
	 *                 cd
	 *    ************** hermite range (size=duration)
	 */
	void hermite(const vector3& a, const vector3& b, int duration, const vector3& c, const vector3& d);
	void transition(const vector3& a, const vector3& b, int duration);
	

	//////////////////////////////////////////////////////////////////////////
	// stitch �迭 �Լ���. void xxx(int discontinuity)
	// this �� discontinuity�ִ� array�� �����ϰ�, ����.
	//////////////////////////////////////////////////////////////////////////

	static void stitchTest(void (vector3N::*func)(int discontinuity), const char* filename);
	// stitch this at center. (even sized array has discontinuity at the center of this.)
	void c0stitch(int discontinuity);
	void c1stitch(int discontinuity);	// use hermite curve();
	// linear system. 
	void linstitch(int discontinuity);

	// ���� �ΰ����� �� �ΰ����� ���.
	void hermite(int discontinuity);

	// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
	void linstitch(vector3N const& a, vector3N const& b);
	void c0stitch(vector3N const& a, vector3N const& b);
	operator vector3*() const	{ return dataPtr();}

};


class vector3NView :public vector3N
{
public:
	// L-value�� ���ɼ� �ִ�, reference array�� �����. 
	vector3NView (m_real * ptrr, int size, int stride);	

	vector3NView ():vector3N()							{Msg::error("do not call this");}
	// ���� reference�� �޾ƿ´�.
	vector3NView(const _tvectorn<vector3, m_real, 3>& other)		{ assignRef(other);}	
	vector3NView(const vector3N& other)					{ assignRef(other);}	
	vector3NView(const vector3NView& other)				{ assignRef(other);}

	~vector3NView (){}

	// L-value�� ���Ǵ� ���, ���� copy�Ѵ�.
	vector3N& operator=(const vector3N & other)			{ assign(other);return *this;}
	vector3N& operator=(const _tvectorn<vector3, m_real, 3>& other){ _tvectorn<vector3, m_real, 3>::assign(other);return *this;}
	vector3N& operator=(const vector3NView& other)		{ assign(other);return *this;}
};
