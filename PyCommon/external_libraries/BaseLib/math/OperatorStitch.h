#pragma once

// analytic constrained SQP solver utility written by Taesoo.
class HessianQuadratic
{
public:
	matrixn H;	// Hessian	(1���� ���)
	vectorn R;	// Hessian residual	 (2���� ���)
	HessianQuadratic(int dim);
	
	// (3x+4y+5z+1)^2 �� �߰��ϰ� ������, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int n, m_real coef1, int index1, ...);

	// 4*(3x+4y+5z+1)^2 �� �߰��ϰ� ������, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int n, m_real coef1, int index1, ...);

	// (3x+4y+5z+1)^2 �� �߰��ϰ� ������, addSquared((0,1,2), (3,4,5,1))
	void addSquared(intvectorn const& index, vectorn const& value);
	void addSquaredH(intvectorn const& index, vectorn const& value);
	void addSquaredR(intvectorn const& index, vectorn const& value);
};

// numerical/analytic unconstrained SQP solver utility written by Taesoo.
class QuadraticFunction
{
public:	

	QuadraticFunction(){}
	virtual ~QuadraticFunction();
	
	// (3x+4y+5z+1)^2 �� �߰��ϰ� ������, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int n, m_real coef1, int index1, ...);
	// 4*(3x+4y+5z+1)^2 �� �߰��ϰ� ������, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int n, m_real coef1, int index1, ...);

	// ys
	// (3x+4y+5z+1)^2 -> addSquared((0,1,2), (3,4,5,1));
	void addSquared(intvectorn const& index, vectorn const& value);
	// ys
	// 4*(3x+4y+5z+1)^2 -> addSquared(4, (0,1,2), (3,4,5,1));
	void addSquaredWeighted(m_real weight, intvectorn const& index, vectorn const& value);
	// ys
	void clear();

	m_real func(vectorn const& x);
	void dfunc(vectorn const& x, vectorn& dx);
	void dfunc(vectorn const& x, vectorn& dx, m_real scale);
	void buildSystem(int nvar, matrixn & A, vectorn &b);

	struct SquaredTerm
	{
		intvectorn index;
		vectorn coef;

		m_real fSquared(vectorn const& x);
		void dfSquared(vectorn const& x, vectorn& dx);
		void dfSquared(vectorn const& x, vectorn& dx, m_real scale);
	};

	std::list<SquaredTerm*> mListSQTerms;
};

class QuadraticFunctionSoftCon : public QuadraticFunction
{
	int mNumVar;
	int mNumCon;
	m_real mCoef;

public:
	QuadraticFunctionSoftCon (int numVar, int numCon, m_real conCoef=1000):mNumVar(numVar), mNumCon(numCon) { mCoef=sqrt(conCoef);}
	virtual ~QuadraticFunctionSoftCon (){}

	// (3x+4y+5z+1==0) �� �߰� �ϰ� ������, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);
	void addCon(int n, m_real coef1, int index1, ...);
};


// analytic constrained SQP solver.
// numerical optimization(func/dfunc)�� ����� ���� ����. - Lagrange multiplier�˰��� ������ ��.
class QuadraticFunctionHardCon : public QuadraticFunction
{
	int mNumVar;
	int mNumCon;
public:

	typedef matrixn MAT_TYPE;	// traits.
	struct Con
	{
		intvectorn index;
		vectorn coef;
		m_real func(vectorn const& x, int numVar_plus_conIndex);
		void dfunc(vectorn const& x, vectorn& dx, int numVar_plus_conIndex);
	};

	std::list<Con*> mListConTerms;

	QuadraticFunctionHardCon (int numVar, int numCon):mNumVar(numVar), mNumCon(numCon){}
	virtual ~QuadraticFunctionHardCon ();

	void addCon(int n, m_real coef1, int index1, ...);

	m_real func(vectorn const& x);
	void dfunc(vectorn const& x, vectorn& dx);
	void buildSystem(matrixn & A, vectorn &b);
};

class AnalyticGradient
{
public:
	struct Base
	{
		Base(){}
		virtual ~Base(){}
		virtual m_real func(vectorn const& x)								{ Msg::error("func should be reimplemented");return 0.0;}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0)	{ Msg::error("dfunc should be reimplemented");}
		virtual Base* copy () const											{ Msg::error("copy has not been reimplemented"); return NULL;}
	};

	struct QuadraticFunction : Base
	{
		::QuadraticFunction f;

		virtual m_real func(vectorn const& x){return f.func(x);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0){f.dfunc(x,dx, scale);}
	};

	struct LinearTerm : Base
	{
		intvectorn index;
		vectorn coef;
		// (3x+4y+5z+1) -> LinearTerm(3,  3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
		LinearTerm(int n, m_real coef1, int index1, ...);

		virtual m_real func(vectorn const& x);
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new LinearTerm(*this); }
	};

	struct MultiplyTwo : Base
	{
		Base* f, * g;
		MultiplyTwo(Base* func1, Base* func2):f(func1), g(func2){}
		virtual ~MultiplyTwo()	{ delete f; delete g;}

		virtual m_real func(vectorn const& x){return f->func(x)*g->func(x);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new MultiplyTwo(f->copy(), g->copy()); }
	};

	struct SQRT : Base
	{
		Base* f;
		SQRT(Base* func):f(func){}
		virtual ~SQRT(){delete f;}
		virtual m_real func(vectorn const& x){return sqrt(f->func(x));}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);		
		virtual Base* copy () const	{ return (Base*)new SQRT(f->copy());}
	};
	
	struct Squared : Base
	{
		Base* f;
		Squared(Base* func):f(func){}
		virtual ~Squared(){delete f;}
		virtual m_real func(vectorn const& x){m_real v=f->func(x); return v*v;}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);		
		virtual Base* copy () const	{ return (Base*)new Squared(f->copy());}
	};

	struct Pow : Base
	{
		Base* f;
		m_real exponent;
		Pow(Base* func, m_real exp):f(func), exponent(exp){}
		virtual ~Pow(){delete f;}
		virtual m_real func(vectorn const& x){m_real v=f->func(x); return pow(v, exponent);}
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);
		virtual Base* copy () const	{ return (Base*)new Pow(f->copy(), exponent);}
	};

	struct LinearCombination : Base
	{
		LinearCombination() :Base(){mCoef=0.0;}
		struct Term
		{
			Term(m_real coef):mTerm(NULL), mCoef(coef){}
			~Term(){delete mTerm;}
			m_real mCoef;
			Base* mTerm;
		};

		void add(Base* pTerm, m_real coef=1.0);
		void addSqrt(Base* pTerm, m_real coef=1.0);
		void addSQR(Base* pTerm, m_real coef=1.0);
		void add(m_real coef){ mCoef+=coef;}
		
		virtual m_real func(vectorn const& x);
		virtual void dfunc(vectorn const& x, vectorn& dx, m_real scale=1.0);

		std::list<Term> mListTerms;
		m_real mCoef;

		virtual Base* copy() const ;		
	};

	void add(Base* pTerm, m_real coef=1.0)		{ t.add(pTerm, coef);}
	void addSqrt(Base* pTerm, m_real coef=1.0)	{ t.addSqrt(pTerm, coef);}
	void addSQR(Base* pTerm, m_real coef=1.0)	{ t.addSQR(pTerm, coef);}

	
	m_real func(vectorn const& x)	{ return t.func(x);}
	
	inline void dfunc(vectorn const& x, vectorn &dx)
	{
		dx.setSize(x.size());
		dx.setAllValue(0.0);

		dfunc_noinit(x,dx);
	}

	// assumes that dx are already initizialized to zero vector
	void dfunc_noinit(vectorn const& x, vectorn& dx) { t.dfunc(x,dx);}
	

private:
	LinearCombination t;

};

#include "math_macro.h"

namespace m0
{
	struct c0stitch: public _op
	{
		int mDiscontinuity;
		m_real mStrength;
		c0stitch(int discontinuity, m_real strength=2.0):mDiscontinuity(discontinuity), mStrength(strength){}
		virtual void calc(matrixn& c) const; 
	};
}

namespace m2
{
	// for multi dimensional signal. (each DOF is dependent)
	struct linstitchMulti: public _op
	{
		m_real mStrength;
		bool mbConMid;
		// strength�� Ŭ���� c0stitch�� ���������.(overshootingȮ���� ��������.) 0���� ū���� �����ؾ��Ѵ�.
		linstitchMulti(m_real strength=5, bool bConMid=true):mStrength(strength), mbConMid(bConMid){}	
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c1stitchPreprocess: public _op
	{
		int mArow;
		int mBrow;
		bool mbConMid;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;
		
		// working space (temporary)
		mutable vectorn cc;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;

		m_real mStrength;
		c1stitchPreprocess(int arow, int brow, m_real strength=2.0, bool bConMid=true);
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};


	struct linstitchPreprocessInc: public _op
	{
		int mArow;
		int mBrow;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;
		
		// working space (temporary)
		mutable vectorn cc,dd;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;
		mutable matrixn aa,bb;


		int mMaxIter;
		int mNormalize;
		// strength�� Ŭ���� c0stitch�� ���������.(overshootingȮ���� ��������.) 0���� ū���� �����ؾ��Ѵ�.
		linstitchPreprocessInc(int arow, int brow, int nIter, int normalize=0,m_real strength=5);
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};


	struct linstitchPreprocess: public _op
	{
		int mArow;
		int mBrow;
		bool mbConMid;
		matrixn Augmented;
		matrixn invAugmented;
		vectorn weight;
		
		// working space (temporary)
		mutable vectorn cc;
		mutable HessianQuadratic h;
		mutable vectorn d;
		mutable intvectorn index;
		mutable vectorn coef;
		mutable vectorn x;


		// strength�� Ŭ���� c0stitch�� ���������.(overshootingȮ���� ��������.) 0���� ū���� �����ؾ��Ѵ�.
		linstitchPreprocess(int arow, int brow, m_real strength=5, bool bConMid=true);
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// ������ �������� ��ġ, �ӵ��� constraint�� ���ӵ� ���̸� minimize.
	struct linstitch2: public _op
	{
		linstitch2(){}
		// a�� ������ �� �������� b�� ù �������Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-2�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// ������ �������� ��ġ, �ӵ��� constraint�� ���ӵ� ���̸� minimize.
	struct linstitchOnline: public _op
	{
		m_real mStrength;
		linstitchOnline(m_real strength=5.0):mStrength(strength){}
		// a�� ������ �������� b�� ù �����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		// ��, a�� ���� �ٲ��� �ʴ´�. �� b�� �����ؼ� stitch
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// ������ �������� ��ġ, �ӵ��� constraint�� ���ӵ� ���̸� minimize.
	struct linstitchForward: public _op
	{
		linstitchForward(){}
		// a�� ������ �������� b�� ù �����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		// ��, b�� ���� �ٲ��� �ʴ´�. �� b�� �����ؼ� stitch
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0concat: public _op
	{
		c0concat(){}
		// a�� ������ �������� b�� ù�����Ӱ� ������������ concat. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0stitch: public _op
	{
		c0stitch(){}
		// a�� ù�����Ӱ� b�� ������ �������� ������ �ʴ´�.
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	struct c0stitchPreserve2: public _op
	{
		c0stitchPreserve2(){}
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		// a�� ù�������Ӱ� b�� ������ ���������� ������ �ʴ´�.
		// linstitch�� ù ��������, ������ ���������� ������ �ʱ� ���� �ϰ����� ���� ����.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};

	// quaterN�� n�� �پ��ִ� matrix�� ��ƼĪ. (c�� a.rows()+b.rows()-1 by 4n matrix�� �ȴ�.)
	struct stitchQuaterNN : public _op
	{
		int mPreserveAmount;
		void (quaterN::*mFunc)(quaterN const&, quaterN const& );
		stitchQuaterNN (void (quaterN::*func)(quaterN const&, quaterN const& ), int preserveAmount=0)
			: mFunc(func),mPreserveAmount(preserveAmount){}
		virtual void calc(matrixn& c, const matrixn& a, const matrixn&b )const;	
	};

	struct c1stitch: public _op
	{
		m_real mStrength;
		c1stitch(m_real strength=2.0):mStrength(strength){}
		// a�� ������ �������� b�� ù�����Ӱ� ������������ stitch. ���� ���̴� a.size()+b.size()-1�� �ȴ�.
		virtual void calc(matrixn& c, const matrixn& a, const matrixn& b) const;
	};	
}

void quater_linstitch(m2::_op const& op, quaterN& c, quaterN const& a, quaterN const& b);
// �������� quaterN�� ��Ƴ� matrix�� stitch. (fast) 
// quaternion align�ϴ��� a�� b�� �ٲ�� ����.
void quaterNN_linstitch(m2::_op const& op, matrixn& c, matrixn & a, matrixn & b);
