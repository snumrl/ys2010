#pragma once


#include "nr/nrtypes_nr.h"

class intmat3D
{
protected:
	int pages, nrows, columns;	
	int* data;
	std::vector<intmatrixnView*> m_pages;
public:
	intmat3D(void);
	intmat3D(int pages, int rows, int columns);
	intmat3D(const intmat3D& other);
	~intmat3D(void);

	int	page() const	{return pages; }
	int rows() const		{return nrows; }
	int cols() const	{return columns; }

	void setSize( int, int, int);  //!< ���� ����Ÿ ���� ���� ���� ����.
	void setSameSize(intmat3D const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	intmatrixnView& page(int index) const				{ return *m_pages[index];}
	intmatrixnView& operator[](int index) const		{ return page(index);}

	intmat3D&  assign(intmat3D const& other);
	intmat3D&  operator=(intmat3D const& other)	{ return assign(other);};	
};

//! 3D matrix Ŭ����
/*! conventions:
1. �� ��ü�� 3D ��Ʈ������ ǥ���ϰų�,
2. 2D matrix�� �ð��� ���� ���� (�� page�� 2D matrix, ��, �ð����� page ������ �ȴ�. )
3. multi-dimension signal ������. (�ð����� row������ �ȴ�.)
��� 2�� 3�� ������ ��Ȯ���� ������, ���� �������� matrix�� ���� 3�� convention�� ���ϴ� ���� ���� ���������� 
smoothing�ϰų� �׷����� �׸��� �־� ���� �ݸ�(ex: �޼� velocity signal, ������ velocity signal)
���� ������ ���� ���������� ���� signal�� matrix�� ������ 2�� convention�� ���ϴ� ���� ���ڴ�.

\ingroup group_math
*/

class hypermatrixn 
{
protected:
	int pages, nrows, columns;	
	m_real* data;
	std::vector<matrixnView*> m_pages;
public:
	hypermatrixn(void);
	hypermatrixn(int pages, int nrows, int columns);
	hypermatrixn(const hypermatrixn& other);
	~hypermatrixn(void);

	int	page() const	{return pages; }
	int rows() const		{return nrows; }
	int cols() const	{return columns; }

	void setSize( int, int, int);  //!< ���� ����Ÿ ���� ���� ���� ����.
	void setSameSize(hypermatrixn const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	matrixnView& page(int index) const				{ return *m_pages[index];}
	matrixnView& operator[](int index) const		{ return page(index);}

	hypermatrixn&  assign(hypermatrixn const& other);
	hypermatrixn&  operator=(hypermatrixn const& other)	{ return assign(other);};	

	void each(const m1::_op& op, const hypermatrixn& other);
};



/*

class matrixn;

#include "../utility/tarray.h"
#include <typeinfo.h>


namespace h2
{
	//enum { ADD, SUB};

	/// Operator�� ���� ������, ������� Operator(ADD) ��� �׳� ADD �̶�� ���� ��.
	struct Operator
	{
		Operator()				{ m_eOperator=-1;}
		Operator(int op)	{ m_eOperator=(int)op;}
		virtual ~Operator()		{}		
		virtual void calc(hypermatrixn& c, const hypermatrixn& a, const hypermatrixn& b) const
		{	Msg::error("h2::%s::calc(vv) not implemented!!!\n", typeid( *this).name()); ASSERT(0);	}
		int m_eOperator;
	};
}

namespace h1
{
	//enum { };
	
	/// Operator�� ���� ������, ������� Operator(ADD) ��� �׳� ADD �̶�� ���� ��.
	struct Operator
	{
		Operator()		{ m_eOperator=-1;}
		Operator(int op)	{ m_eOperator=(int)op;}
		virtual ~Operator()		{}		
		virtual void calc(hypermatrixn& c, const hypermatrixn& a) const
		{	Msg::error("h1::%s::calc(vv) not implemented!!!\n", typeid( *this).name()); ASSERT(0);}
		int m_eOperator;
	};
	
}

namespace sh1
{
	enum { LENGTH, MINIMUM, MAXIMUM, SUM, AVERAGE, RMS , SQUARESUM } ;
	
	/// Operator�� ���� ������, ������� Operator(LENGTH)��� �׳� LENGTH ��� ���� ��.
	struct Operator
	{		
		Operator()	{}
		Operator(int op);
		virtual ~Operator(){};
		virtual m_real calc(const hypermatrixn& c) const;
		int m_eOP;		
	};
};

class hypermatrixn :
	protected  CTArray<matrixn>
{
private:
	friend class matrixn;
	int pages, rows, columns;
	bool m_bDirty;	// �����߿� �ϳ����� resize�� ����Ȱ�쿡 m_bDirty�� set�ȴ�. �� ��� �������� ���� ũ�Ⱑ �ƴҼ� �ִٴ� ���� �ȴ�.
public:
	hypermatrixn(void);
	hypermatrixn(int pages, int rows, int columns);
	hypermatrixn(const hypermatrixn& other);
	~hypermatrixn(void);

	bool dirty();
	void clean();	// ���� ũ�Ⱑ �ٸ� ��� ������ �� ū ũ��� �ٲپ��ش�.

    int	page() const	{return pages; }
    int rows() const		{return rows; }
    int cols() const	{return columns; }
	
	void setSize( int, int, int);  //!< ���� ����Ÿ ���� ���� ���� ����.
	void resize(int, int, int);		//!< ���� ����Ÿ �ִ��� ����. ���ڸ��� 0����(����)
	void setSameSize(hypermatrixn const& other)	{ setSize(other.page(), other.rows(), other.cols());};
	matrixn& page(int index) const				{ return CTArray<matrixn>::operator [](index);};
	matrixn& operator[](int index) const		{ return page(index);};

	void fromMatrix(const matrixn& mat, int column);

	// unary operations
	m_real op1(const sh1::Operator& op) const;
	hypermatrixn&  op1(const s1::Operator&, m_real);
	hypermatrixn&  op1(const h1::Operator&, hypermatrixn const&);
	hypermatrixn&  operator=(hypermatrixn const& other)	{ return assign(other);};
	hypermatrixn&  assign(hypermatrixn const& other);

	// binary operations
	hypermatrixn&  op2(const h2::Operator&, hypermatrixn const&, hypermatrixn const&);
	
	void	  load(const char* filename, bool bLoadFromBinaryFile=false);
	void      save(const char* filename, bool bSaveIntoBinaryFile=false);

};
*/