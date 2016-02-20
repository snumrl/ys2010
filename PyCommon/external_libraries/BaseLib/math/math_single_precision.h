#pragma once
#include "template_math.h"

class vectornSPView;
class matrixnSPView ;

class vectornSP : public _tvectorn<float>
{	
protected:
	vectornSP(float* ptrr, int size, int stride):_tvectorn<float>(ptrr,size,stride){}
public:
	vectornSP();		

	// ���� ī���ؼ� �޾ƿ´�.	
	template <class VecType>
	vectornSP(const VecType& other)	{ assign(other);}

	explicit vectornSP( int x):_tvectorn<float>() { setSize(x);}

	// n dimensional vector	(ex) : vectornSP(3, 1.0, 2.0, 3.0);
	explicit vectornSP( int n, float x);		
	explicit vectornSP( int n, float x, float y);		
	explicit vectornSP( int n, float x, float y, float z);		
	explicit vectornSP( int n, float x, float y, float z, float w, ...);	// n dimensional vector	(ex) : vectornSP(3, 1.0, 2.0, 3.0);

	~vectornSP(){}

	matrixnSPView column() const;	// return n by 1 matrix, which can be used as L-value (reference matrix)
	matrixnSPView row() const;	// return 1 by n matrix, which can be used as L-value (reference matrix)
	
	// L-value�� ���ɼ� �ִ�, reference array�� ����� return �Ѵ�. 
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	vectornSPView range(int start, int end, int step=1);
	const vectornSPView range(int start, int end, int step=1) const	;

	template <class VecType>
	vectornSP& operator=(const VecType& other)	{ _tvectorn<float>::assign(other);return *this;}
};



class vectornSPView :public vectornSP
{
public:
	// L-value�� ���ɼ� �ִ�, reference array�� �����. 
	vectornSPView (float* ptrr, int size, int stride);	
	// ���� reference�� �޾ƿ´�.
	template <class VecType>
	vectornSPView(const VecType& other)	{ assignRef(other);}

	~vectornSPView (){}

	// L-value�� ���Ǵ� ���, ���� copy�Ѵ�.
	template <class VecType>
	vectornSPView& operator=(const VecType& other)	{ _tvectorn<float>::assign(other);return *this;}

};


class matrixnSP : public _tmat<float>
{
protected:
	matrixnSP(float* _ptr, int _n, int _m, int _stride) :_tmat<float>(_ptr, _n, _m, _stride){}
public:
	matrixnSP():_tmat<float>(){}
	matrixnSP ( int x, int y);

	template <class MatType>
	matrixnSP (const MatType& other):_tmat<float>()	{ _tmat<float>::assign(other);	}

	virtual ~matrixnSP();

	// reference�� value�� ī�ǵȴ�.
	template <class MatType>
	matrixnSP& operator=(const MatType& other)	{ __super::assign(other); return *this;}

	// L-value�� ���ɼ� �ִ� reference vector �� return�Ѵ�.
	vectornSPView		row(int i)const				{ return _row<vectornSPView>(i);}
	vectornSPView		column(int i)const			{ return _column<vectornSPView>(i);}

	matrixnSPView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX);
	inline const matrixnSPView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX) const;	

};

class matrixnSPView : public matrixnSP
{
public:
	// copy constructors : get reference
	template <class MatType>
	matrixnSPView(const MatType& other)		{ _assignRef(other);	}

	// L-value�� ����Ҷ��� copy
	template <class MatType>
	matrixnSPView& operator=(const MatType& other)	{ _tmat<float>::assign(other); return *this;}

	// L-value�� ���ɼ� �ִ�, reference matrix�� �����. 
	matrixnSPView (float* ptr, int nrow, int ncol, int stride);
	virtual ~matrixnSPView ();
};
