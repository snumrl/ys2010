#include "stdafx.h"
#include "mathclass.h"
#include "operator.h"
#include "filter.h"
#include "bspline.h"
#include "nr/nr.h"
#include "../utility/operatorstring.h"
#include "matrixnTransposedView.h"
namespace sop
{
	int interpolateInt(m_real t, int s, int e)
	{
		m_real f=m_real(s)*(1.0-t)+m_real(e)*t;
		return ROUND(f);
	}

	m_real interpolate(m_real t, m_real s, m_real e)
	{
		m_real f=s*(1.0-t)+e*t;
		return f;
	}

	m_real map(m_real t, m_real min, m_real max, m_real v1, m_real v2)
	{
		t=(t-min)/(max-min);
		return interpolate(t, v1, v2);
	}

	m_real clampMap(m_real t, m_real min, m_real max, m_real v1, m_real v2)
	{
		if(t<min) t=min;
		if(t>max) t=max;

		t=(t-min)/(max-min);
		return interpolate(t, v1, v2);
	}

	m_real smoothTransition(m_real a)
	{ return ((m_real)-2.0)*a*a*a+((m_real)3.0)*a*a;} // y=-2x^3+3x^2
}

index2 m::argMin(matrixn const& a)
{
	index2 argMinV;
	m_real minV=DBL_MAX;
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<a.cols(); j++)
		{
			if(a(i,j)<minV)
			{
				minV=a(i,j);
				argMinV=index2(i,j);
			}
		}
	}
	return argMinV;
}

int v::argMinRand(vectorn const& a, m_real thr, int start, int end)
{
	if (end>a.size()) 
		end=a.size();
	int argMinV=a.argMin();

	intvectorn indexes;

	m_real minV=a(argMinV)*thr;
	for(int i=start; i<end; i++)
	{
		if(a[i]<minV)
			indexes.push_back(i);
	}

	return indexes[rand()%indexes.size()];
}
index2 m::argMinRand(matrixn const& a, m_real thr)
{
	index2 argMinV=argMin(a);

	std::vector<index2> indexes;

	m_real minV=a(argMinV(0), argMinV(1))*thr;
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<a.cols(); j++)
		{
			if(a(i,j)<minV)
			{
				indexes.push_back(index2(i,j));
			}
		}
	}

	return indexes[rand()%indexes.size()];
}

namespace m
{
	void multAB(matrixn& out, matrixn const& a, matrixn const& b, bool transposeA, bool transposeB)
	{
		if(transposeA)
		{
			if(transposeB)
				out.multAtBt(a,b);
			else
				out.multAtB(a,b);
		}
		else
		{
			if(transposeB)
				out.multABt(a,b);
			else
				out.mult(a,b);
		}
	}

	void multABC(matrixn& out, matrixn const& a, matrixn const& b, matrixn const& c, bool transposeA, bool transposeB, bool transposeC)
	{
		int rowA=a.rows();
		int colA=a.cols();
		int rowB=b.rows();
		int colB=b.cols();
		int rowC=c.rows();
		int colC=c.cols();

		if(transposeA)	SWAP<int>(rowA, colA);
		if(transposeB)	SWAP<int>(rowB, colB);
		if(transposeC)	SWAP<int>(rowC, colC);

		ASSERT(colA==rowB);
		ASSERT(colB==rowC);
		
		int aa=rowA;
		int bb=rowB;
		int cc=rowC;
		int dd=colC;

		if(aa*bb*cc+aa*cc*dd < bb*cc*dd+aa*bb*dd)
		{
			// case 1: (ab)c is efficient than a(bc)
			matrixn ab;
			m::multAB(ab, a, b, transposeA, transposeB);
			m::multAB(out, ab, c, false, transposeC);
		}
		else
		{
			// case 1: a(bc) is efficient than (ab)c
			matrixn bc;
			m::multAB(bc, b, c, transposeB, transposeC);
			m::multAB(out, a, bc, transposeA, false);
		}
	}

	void multAtB(vectorn& out, matrixn const& A, vectorn const& b)
	{
		ASSERT(A.cols()==b.size());
		// out.setSize(A.rows());
		// out.column().multAtB(A, b.column()); -> slow
		out.multmat(b,A);
	}

	void LUsolve(matrixn const & A, vectorn const& b, vectorn& x)
	{
		DP p;
		matrixn LU(A);
		Vec_INT indx(A.rows());
		NR::ludcmp(LU,indx,p);
		x=b;
		NR::lubksb(LU,indx,x);
	}

	// covariance * numData: usually called the matrix S. 
	void covarianceN(vectorn& mean, matrixn& c, const matrixn& a) 
	{
		mean.aggregateColumn(CAggregate::AVG, a);

		matrixn zeroMeaned;
		zeroMeaned.each2(&vectorn::sub, a, mean);

		int dimension=a.cols();
		m_real numData=a.rows();

		c.setSize(dimension, dimension);

		for(int i=0; i<dimension; i++)
		{
			for(int j=0; j<dimension; j++)
			{
				// ���Ѱ��� average
				c[i][j]=(zeroMeaned.column(i)%zeroMeaned.column(j));///(numData);
			}
		}
	}

	matrixn op1(m1::_op const& op, matrixn const& A)	
	{
		matrixn c; c.op1(op, A); return c;
	}

	matrixn diag(vectorn const& a)
	{
		matrixn c;
		c.setSize(a.size(),a.size());
		c.setAllValue(0.0);
		c.setDiagonal(a);
		return c;
	}

	void LUinvert(matrixn& F, const matrixn& E, m_real& log_det)
	{
		/* transfer matrix for inversion into F */

		ASSERT(E.rows()==E.cols());
		F.setSameSize(E);

		static matrixn C_temp;
		static vectorn lu_col;
		C_temp=E;
		lu_col.setSize(E.rows());
		log_det = 0.0;

		/* Use LU decomposition to invert E[][] and find det(E) */
		Vec_INT indx(E.rows());
		m_real d;
		NR::ludcmp(C_temp, indx, d);
		int n=E.rows();

		for(int i=0; i < n; ++i)
		{
			log_det += log(fabs(C_temp[i][i]));
			d       *= ( C_temp[i][i] > 0.0 ) ? 1.0 : - 1.0 ; 
		}
		if( d < 0.0 ) printf("WARNING - negative eigenvalues\n");

		for(int j=0;j<n;j++)
		{
			lu_col.setAllValue(0);
			lu_col[j]=1.0;
			NR::lubksb(C_temp,indx,lu_col);
			for(int i=0;i<n;i++) F[i][j] = lu_col[i];
		}
	}

	void Diaginvert(vectorn& out, const vectorn& in, m_real& log_det)
	{
		out.each1(s1::LOG, in);
		log_det=out.sum();
		out.each1(s1::INVERSE, in);
	}

	void LUinvert(matrixn& F, const matrixn& E, m_real & d_man, int& d_exp)
	{
		ASSERT(E.rows()==E.cols());
		F.setSameSize(E);

		static matrixn C_temp;		
		static vectorn col;
		C_temp=E;
		
		int n=E.rows();

		col.setSize(n);

		
		/* Use LU decomposition to invert E[][] and find det(E) */
		Vec_INT indx(n);
		
		
		d_exp = 0;
		NR::ludcmp(C_temp,indx,d_man);


		for(int j=0; j<n; j++) 
		{
			d_man *= C_temp[j][j];
			while( fabs(d_man)>10 ) 
			{
			  d_man = d_man/10;
			  d_exp++;
			}
			while( (fabs(d_man)<0.1)&&(fabs(d_man)>0) ) {
			  d_man = d_man*10;
			  d_exp--;
			}
		}

		for(int j=0; j<n; j++) 
		{
			col.setAllValue(0);
			col[j]=1.0;
			NR::lubksb(C_temp, indx,col);
			for(int i=0; i<n; i++) F[i][j] = col[i];
		}
	}

}

void v0::colon::calc(vectorn& c) const
{
	int count=0;
	for(int i=mStart; i<mEnd; i+=mStepSize)
		count++;

	c.setSize(count);
	count=0;
	for(int i=mStart; i<mEnd; i+=mStepSize)
		c[count++]=i;
}
		
void v0::pow::calc(vectorn &c) const	
{
	for(int i=0; i<c.size(); i++) c[i]=::pow(c[i], mK);
}
void v0::useUnary::calc(vectorn& c) const
{
	vectorn temp(c);
	m_cOperator.calc(c,temp);
}

void v0::transition::calc(vectorn& c) const
{
	c.setSize(mnSize);

	m_real totalTime=mnSize-1;
	m_real currTime;
	for(int i=0; i<mnSize; i++)
	{
		currTime=(float)i/totalTime;
		float t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
		c[i]=sop::interpolate(t, mStart, mEnd);
	}
}

void v0::decay::operator()(vectorn& c) const
{
	c.setSize(mnSize);

	m_real totalTime=mnSize-1;
	m_real currTime;
	for(int i=0; i<mnSize; i++)
	{
		currTime=(m_real)i/totalTime;
		m_real t;
		switch(mType)
		{
		case TRANSITION:
			t=1.0-(-2.f*CUBIC(currTime)+3.f*SQR(currTime));
			break;
		case COS:
			t=cos(currTime*M_PI*0.5);
			break;
		case LINEAR:
			t=1.0-currTime;
			break;
		}
		c[i]=t*mStart;
	}
}
void v0::decay::calc(vectorn& c) const 
{
	this->operator()(c);
}

void v0::zeroToOne::operator()(vectorn& c) const
{
	if(mType==LINEAR)
	{
		v0::decay(1.0, mnSize, v0::decay::LINEAR)(c);
		c.sub(1.0, c);
	}
	else if(mType==SLOW_START)
	{
		v0::decay(1.0, mnSize, v0::decay::COS)(c);
		c.sub(1.0, c);
	}
	else if(mType==SLOW_END)
	{
		vectorn temp;
		v0::decay(1.0, mnSize, v0::decay::COS)(temp);
		v1::reverse()(c, temp);
	}
	else
	{
		v0::decay(1.0, mnSize, v0::decay::TRANSITION)(c);
		c.sub(1.0, c);
	}
}

void v0::oneToZero::operator()(vectorn& c) const
{
	if(mType==LINEAR)
	{
		v0::decay(1.0, mnSize, v0::decay::LINEAR)(c);
	}
	else if(mType==SLOW_START)
	{		
		v0::decay(1.0, mnSize, v0::decay::COS)(c);
	}
	else if(mType==SLOW_END)
	{
		vectorn temp;
		v0::decay(1.0, mnSize, v0::decay::COS)(temp);
		temp.sub(1.0, temp);
		v1::reverse()(c, temp);		
	}
	else
	{
		v0::decay(1.0, mnSize, v0::decay::TRANSITION)(c);
	}
}
/*
void v0::each::calc(vectorn& c) const
{
	for(int i=0; i<c.size(); i++)
		m_cOperator.Calc(c[i], c[i]);
}


void v0::useUnary::calcInt(intvectorn& c) const
{
	intvectorn temp(c);
	m_cOperator.calcInt(c,temp);
}

v0::domain::domain(const bitvectorn& abIndex, const v0::Operator& op)
:m_op(op)
{
	m_aIndex.findIndex(abIndex,true);
}

v0::domain::domain(int start, int end, int stepSize, const v0::Operator& op)
:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void v0::domain::calc(vectorn& c) const
{
	vectorn temp;
	temp.op1(v1::extract(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.size()!=m_aIndex.size())
		printf("Error in v0::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aIndex), temp);
}

void v0::domain::calcInt(intvectorn& c) const
{
	intvectorn temp;
	temp.op1(v1::extract(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.size()!=m_aIndex.size())
		printf("Error in v0::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aIndex), temp);
}

v1::domainRange::domainRange(const bitvectorn& abIndex, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.findIndex(abIndex,true);
	m_aOutIndex=m_aInIndex;

}

v1::domainRange::domainRange(int start, int end, int stepSize, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end, stepSize);
	m_aOutIndex=m_aInIndex;
}

void v1::domainRange::calc(vectorn& c, const vectorn& a) const
{
	vectorn tempc;
	vectorn tempa;
	
	tempc.op1(v1::extract(m_aOutIndex),c);
	tempa.op1(v1::extract(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), tempc);
}

v1::domain::domain(int start, int end, const v1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end);
}
void v1::domain::calc(vectorn& c, const vectorn& a) const
{
	vectorn tempa;

	tempa.op1(v1::extract(m_aInIndex),a);
	c.op1(m_op, tempa);
}

void v1::domainRange::calcInt(intvectorn& c, const intvectorn& a) const
{
	intvectorn tempc;
	intvectorn tempa;
	
	tempc.op1(v1::extract(m_aOutIndex),c);
	tempa.op1(v1::extract(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), tempc);
}

void v1::domainRange::calc(vectorn& c, m_real a) const
{
	vectorn temp;
	temp.op1(v1::extract(m_aOutIndex), c);
	temp.op1(m_op, a);

	if(temp.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), temp);
}

void v1::domainRange::calcInt(intvectorn& c, int a) const
{
	intvectorn temp;
	temp.op1(v1::extract(m_aOutIndex), c);
	temp.op1(m_op, a);

	if(temp.size()!=m_aOutIndex.size())
		printf("Error in v1::domain::calcInt\n");
	else
		c.op1(v1::assign(m_aOutIndex), temp);
}*/

v1::filter::filter(const vectorn& kernel, int numIter)
{
	m_vKernel.assign(kernel);
	m_nNumIter=numIter;
}

v1::filter::filter(int kernelSize, int numIter)
{
	Filter::GetGaussFilter(kernelSize, m_vKernel);	
	m_nNumIter=numIter;
}

void v1::filter::calc(vectorn& c, const vectorn& a) const
{
	Filter::LTIFilter(m_nNumIter, m_vKernel, a, c);
}

	// c.size()=a.size()-1. c[i]=a[i+1]-a[i].
void v1::delta::calc(vectorn& c, const vectorn& a) const
{
	operator()(c,a);
}

void v1::delta::operator()(vectorn& c, const vectorn& a) const
{
	c.setSize(a.size()-1);
	for(int i=0; i<c.size(); i++)
		c[i]=a[i+1]-a[i];
}


void v1::cumulate::calc(vectorn& c, const vectorn& a) const
{
	c.setSize(a.size()+1);
	c[0]=0.0;

	for(int i=0; i<a.size(); i++)
		c[i+1]=c[i]+a[i];
}

void v1::secondDerivative::calc(vectorn& c, const vectorn& a) const
{
	matrixn temp, temp2;
	temp.setSize(a.size(), 1);
	temp.setColumn(0, a);

	UniformSpline spl(temp);
	spl.getSecondDeriv(temp2);

	temp2.getColumn(0, c);
/*
	c.setSize(a.size());
	ASSERT(c.size()>2);
	for(int i=1; i<a.size()-1; i++)
	{
		c[i]= a[i+1] + a[i-1] -2.0*a[i];
	}
	c[0]=c[1];
	c[a.size()-1]=c[a.size()-2];*/
}

/*
v1::extract::extract(const intvectorn& index)
{
	m_vIndex.assign(index);
}

v1::extract::extract(int start, int end, int stepSize)
{
	m_vIndex.colon(start, end, stepSize);
}

void v1::extract::calc(vectorn& c, const vectorn& a) const
{
	c.setSize(m_vIndex.size());

	for(int j=0; j<m_vIndex.size(); j++)
		c[j]=a[m_vIndex[j]];
}

void v1::extract::calcInt(intvectorn& c, const intvectorn& a) const
{
	c.setSize(m_vIndex.size());

	for(int j=0; j<m_vIndex.size(); j++)
		c[j]=a[m_vIndex[j]];
}*/

v1::assign::assign(const intvectorn& index)
{
	m_vIndex.assign(index);
}

v1::assign::assign(int start, int end, int stepSize)
{
	m_vIndex.colon(start, end, stepSize);
}

void v1::assign::calc(vectorn& c, const vectorn& a) const
{
	if(m_vIndex.size())
	{
		for(int j=0; j<m_vIndex.size(); j++)
			c[m_vIndex[j]]=a[j];
	}
	else
	{
		c.assign(a);
	}
}
/*
void v1::assign::calcInt(intvectorn& c, const intvectorn& a) const
{
	for(int j=0; j<m_vIndex.size(); j++)
		c[m_vIndex[j]]=a[j];
}*/

void v1::sort::calc(vectorn& c, const vectorn& a) const
{
	intvectorn sortedIndex;
	c.sort(a, sortedIndex);
}

/*v2::domain::domain(int start, int end, int stepSize, const v2::Operator& op)
:m_op(op)
{
	m_cIndex.colon(start, end, stepSize);
	m_aIndex=m_cIndex;
	m_bIndex=m_cIndex;
}

void v2::domain::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	vectorn tempc;
	vectorn tempa;
	vectorn tempb;
	
	tempc.op1(v1::extract(m_cIndex),c);
	tempa.op1(v1::extract(m_aIndex),a);
	tempb.op1(v1::extract(m_bIndex),b);
	tempc.op2(m_op, tempa, tempb);
	
	if(m_aIndex.size()!=m_cIndex.size())
		printf("Error in v2::domain::calc\n");
	else
		c.op1(v1::assign(m_cIndex), tempc);
}

void v2::domain::calcInt(intvectorn& c, const intvectorn& a, const intvectorn& b) const
{
	intvectorn tempc;
	intvectorn tempa;
	intvectorn tempb;
	
	tempc.op1(v1::extract(m_cIndex),c);
	tempa.op1(v1::extract(m_aIndex),a);
	tempb.op1(v1::extract(m_bIndex),b);
	tempc.op2(m_op, tempa, tempb);
	
	if(m_aIndex.size()!=m_cIndex.size())
		printf("Error in v2::domain::calc\n");
	else
		c.op1(v1::assign(m_cIndex), tempc);
}
*/
#include "filter.h"
void v2::filter::calc(vectorn& c, const vectorn& source, const vectorn& kernelSize) const
{
	Filter::LTIFilter(1, m_fFrameTime, kernelSize, source, c);
}

/*
void v2::each::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.Calc(a[i],b[i]);
}	

void v2::each::calc(vectorn& c, const vectorn& a, m_real b) const
{
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.Calc(a[i],b);
}	

void v2::each::calcInt(intvectorn& c, const intvectorn& a, const intvectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.CalcInt(a[i],b[i]);
}

void v2::each::calcInt(intvectorn& c, const intvectorn& a, int b) const
{
	c.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		c[i]=m_cOperator.CalcInt(a[i],b);
}

*/

void v2::multAdd::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(c.size()==a.size() && c.size()==b.size());

	for(int i=0; i<c.size(); i++)
	{
		c[i]+=a[i]*b[i];
	}
}

void v2::cross::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	vector3 vc;
	vc.cross(a.toVector3(), b.toVector3());
	c.assign(vc);
}

void v2::minimum::calc(vectorn& c, const vectorn& a, const vectorn& b) const 
{
	c.each2(s2::MINIMUM, a,b);	
}

void v2::maximum::calc(vectorn& c, const vectorn& a, const vectorn& b) const 
{
	c.each2(s2::MAXIMUM, a,b);	
}

void v2::add::calc( vectorn& c, const vectorn & a, const vectorn & b ) const
{
    assert( a.size()==b.size() );
    c.setSize( a.size() );

    for( int i=0; i<a.size(); i++ )
        c[i] = a[i] + b[i];
}


void v2::sub::calc( vectorn& c, const vectorn & a, const vectorn & b ) const
{
    assert( a.size()==b.size() );
    c.setSize( a.size() );

    for( int i=0; i<a.size(); i++ )
        c[i] = a[i] - b[i];
}


void v2::mid::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.add(a,b);
	c/=2.0;
}


void v2::interpolate::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(a.size()==b.size());
	c.setSize(a.size());
	for(int i=0; i<c.size(); i++)
	{
		c[i]=(1.0-m_fT)*a[i]+m_fT*b[i];
	}
}

/*
/// c=ADD(c,d) ó�� binaryop�� ����Ѵ�. eg) matrix1.op1(m1::useBinary(m2::...()))..

m0::domain::domain(int start, int end, int stepSize, const m0::Operator& op)
:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void m0::domain::calc(matrixn& c) const
{
	matrixn temp;
	temp.op1(m1::extractRows(m_aIndex), c);
	temp.op0(m_op);
	
	if(temp.rows()!=m_aIndex.size())
		Msg::error("Error in m0::domain::calc\n");
	else
		c.op1(m1::assignRows(m_aIndex), temp);
}

m1::assignRows::assignRows(const intvectorn& index)
{
	m_vIndex.assign(index);
}

m1::assignRows::assignRows(int start, int end, int stepSize)
{
	m_vIndex.colon(start, end, stepSize);
}

void m1::assignRows::calc(matrixn& c, const matrixn& a) const
{
	for(int j=0; j<m_vIndex.size(); j++)
		c[m_vIndex[j]]=a[j];
}

m1::domainRange::domainRange(const bitvectorn& abIndex, const m1::Operator& op)
:m_op(op)
{
	m_aInIndex.findIndex(abIndex,true);
	m_aOutIndex=m_aInIndex;

}

m1::domainRange::domainRange(int start, int end, int stepSize, const m1::Operator& op)
:m_op(op)
{
	m_aInIndex.colon(start, end, stepSize);
	m_aOutIndex=m_aInIndex;
}

void m1::domainRange::calc(matrixn& c, const matrixn& a) const
{
	matrixn tempc;
	matrixn tempa;
	
	// unary operator ���� c�� ������� �ʴ� ��찡 �ִ�. �̰�� ����ڰ� c�� ũ�⸦ ������ �������� �ʴ°�찡 �ִ�.
	// �̰�� ���� ũ�Ⱑ �ٸ��� ������ ���� �����Ƿ� �Ʒ��� �߰��Ͽ���. c�� a�� ũ�Ⱑ ���� �Ϲ����� ��� �������.
	c.setSameSize(a);

	tempc.op1(m1::extractRows(m_aOutIndex),c);
	tempa.op1(m1::extractRows(m_aInIndex),a);
	tempc.op1(m_op, tempa);
	
	if(m_aInIndex.size()!=m_aOutIndex.size())
		Msg::error("Error in m1::domain::calcInt (in, out size are different!)");
	else
		c.op1(m1::assignRows(m_aOutIndex), tempc);
}
*/

void m0::diagonalize::calc(matrixn& c) const
{
	ASSERT(c.rows()==c.cols());
	int nbands=c.rows();
    int b1,b2;

    for(b1=0; b1<nbands; b1++)
    for(b2=0; b2<nbands; b2++)
      if(b1!=b2) c[b1][b2] = 0;

}

void m1::LUinvert::calc(matrixn& F, const matrixn& E) const
{
	/* transfer matrix for inversion into F */

	ASSERT(E.rows()==E.cols());
	F.setSameSize(E);

	static matrixn C_temp;
	static vectorn lu_col;
	C_temp=E;
	lu_col.setSize(E.rows());

	/* Use LU decomposition to invert E[][] and find det(E) */
	Vec_INT indx(E.rows());
	m_real d;
	NR::ludcmp(C_temp, indx, d);
	int n=E.rows();

	for(int j=0;j<n;j++)
	{
		lu_col.setAllValue(0);
		lu_col[j]=1.0;
		NR::lubksb(C_temp,indx,lu_col);
		for(int i=0;i<n;i++) F[i][j] = lu_col[i];
	}
}

void m1::splineFit::calc(matrixn& c, const matrixn& a) const
{
	BSpline sp(a, m_nDegree);
	c.setSameSize(a);
	for(int i=0; i<a.rows(); i++)
	{
		sp.GetPosition((float)i/(float)(a.rows()-1), c.row(i));
	}
}

void v1::useBinary::calc(vectorn& c, const vectorn& a) const
{
	vectorn temp(c);
	m_op.calc(c, temp, a);
}

void m0::useUnary::calc(matrixn& c) const
{
	matrixn temp(c);
	m_cOperator.calc(c, temp);
}
/*

void v1::useBinary::calcInt(intvectorn& c, const intvectorn& a) const
{
	intvectorn temp(c);
	m_op.calcInt(c, temp, a);
}

void v1::each::calc(vectorn& c, const vectorn& a) const
{
	
}

void v1::each::calc(vectorn& c, m_real a) const
{
	for(int i=0; i<c.size(); i++)
		m_cOperator.Calc(c[i], a);
}*/



/*
void v1::each::calcInt(intvectorn& c, const intvectorn& a) const
{
	for(int i=0; i<c.size(); i++)
		m_cOperator.CalcInt(c[i], a[i]);
}

void v1::each::calcInt(intvectorn& c, int a) const
{
	for(int i=0; i<c.size(); i++)
		m_cOperator.CalcInt(c[i], a);
}
void v1::concat::calc(vectorn& c, const vectorn& a) const
{
	int prev_size=c.size();
	c.resize(c.size()+a.size());
	c.setValue(prev_size, prev_size+a.size(), a);
}

void v1::concat::calcInt(intvectorn& c, const intvectorn& a) const
{
	int prev_size=c.size();
	c.resize(c.size()+a.size());
	c.setValue(prev_size, prev_size+a.size(), a);
}

void v1::derivative::calc(vectorn& c, const vectorn& a) const
{
	c.setSize(a.size());
	ASSERT(c.size()>2);
	for(int i=1; i<a.size()-1; i++)
	{
		c[i]= (a[i+1]- a[i-1])/2.f;
	}
	c[0]=c[1];
	c[a.size()-1]=c[a.size()-2];
}



void m0::each::calc(matrixn& c) const
{
	int prev_column=c.cols();

    // output size determination
	vectorn out(prev_column);

	for(int i=0; i<c.rows(); i++)
	{
		out.setSize(prev_column);
		out.setValue(0, prev_column, c[i]);
		m_cOperator.calc(out);
		c[i].assign(out);
	}
}*/
/*
void m0::align::calc(matrixn& c) const
{
	for(int i=1; i<c.rows(); i++)
	{
		if(c[i-1]%c[i]<0)
			c[i]*=-1.f;
	}
}*/
#include "../image/imageclass.h"

m0::drawSignals::drawSignals(const char* filename, bool useTheSameMinMax, const intvectorn& vXplot)
: m_strFilename(filename), m_fMin(0.f), m_fMax(0.f), m_bMultiDimensional(true), m_vXplot(vXplot),m_bUseTheSameMinMax(useTheSameMinMax)
{
}

void m0::drawSignals::calc(matrixn& c) const
{
	CImage* pImage;
	if(m_bMultiDimensional)
	{
		matrixn t;
		t.transpose(c);

		if(m_bUseTheSameMinMax)
			pImage=CImageProcessor::DrawChart(t, CImageProcessor::LINE_CHART, m_fMin, m_fMax);
		else
		{
			vectorn aMin, aMax;
			aMin.minimum(c);
			aMax.maximum(c);
			aMax.each1(s1::RADD, 0.000001); 
			pImage=Imp::DrawChart(t, CImageProcessor::LINE_CHART, aMin, aMax);
		}
	}
	else
		pImage=CImageProcessor::DrawChart(c, CImageProcessor::LINE_CHART, m_fMin, m_fMax);

	CImagePixel cip(pImage);
	
	for(int i=0; i<m_vXplot.size(); i++)
	{
		cip.DrawVertLine(m_vXplot[i], 0, cip.Height(), RGB(0,0,0),true);
	}

	CImageProcessor::SafeDelete(pImage, m_strFilename);

}

void m0::draw::calc(matrixn& c) const
{
	m_real min, max;
	min=m_fMin;
	max=m_fMax;

	if(min==max)
	{
		min=c.minimum();
		max=c.maximum();
	}

	CImage* pImage;
	pImage=CImageProcessor::DrawMatrix2(c, min, max);

	CImagePixel cip(pImage);
	cip.DrawText(0, 0, sz1::format("[%f,%f]", min, max));

	CImageProcessor::SafeDelete(pImage, m_strFilename);
}

void m0::plotScatteredData::calc(matrixn& c) const
{
	CImage* pImage;
	pImage=CImageProcessor::Plot(c, m_vMin, m_vMax);
	CImageProcessor::SafeDelete(pImage, TString("myplot/")+m_strFilename);
}
/*
void m0::linspace::calc(matrixn& c) const
{
	vectorn space;
	space.linspace(0.f, 1.f, m_n);
	c.setSize(m_n, m_vStart.size());
	for(int i=0; i<m_n; i++)
		c[i].interpolate(m_vStart, m_vEnd, space[i]);
}
*/
/*void m1::extractRows::calc(matrixn& c, const matrixn& a) const
{
	c.extractRows(a, m_aIndex);
}*/

m1::filter::filter(const vectorn& kernel, int numIter)
{
	m_vKernel.assign(kernel);
	m_nNumIter=numIter;
}

m1::filter::filter(int kernelSize, int numIter)
{
	Filter::GetGaussFilter(kernelSize, m_vKernel);	
	m_nNumIter=numIter;
}

void m1::filter::calc(matrixn& c, const matrixn& a) const
{
	Filter::LTIFilter(m_nNumIter, m_vKernel, a, c);
}
	
/*void m1::each::calc(matrixn& c, const matrixn& a) const
{
	ASSERT(c.rows()==a.rows());

	int prev_column=c.cols();

    // output size determination
	vectorn out(prev_column);

	for(int i=0; i<a.rows(); i++)
	{
		out.setSize(prev_column);
		out.setValue(0, prev_column, c[i]);
		m_cOperator.calc(out, a[i]);
		c[i].assign(out);
	}
}*/

void m1::derivative::calc(matrixn& c, const matrixn& a) const
{
	c.setSameSize(a);
	ASSERT(c.rows()>2);
	for(int i=1; i<a.rows()-1; i++)
	{
		c.row(i).sub(a.row(i+1), a.row(i-1));
		c.row(i)*=1/2.f;
	}
	c.setRow(0, c.row(1));
	c.setRow(a.rows()-1, c.row(a.rows()-2));
}

void m1::superSampling::calc(matrixn& c, const matrixn& a) const/// c.op(a)
{
	UniformSpline spl(a, m_nStart, m_nEnd);
	spl.getCurve(c, m_nXn);

	// duplicate final vectors so that the number of rows simply becomes (end-start)*m_nXn
	c.resize(c.rows()+m_nXn-1, c.cols());
	for(int i=0; i<m_nXn-1; i++)
	{
	
		c.row(c.rows()-m_nXn+1+i)=c.row(c.rows()-m_nXn);
	}
}

void m1::downSampling::calc(matrixn& c, const matrixn& a) const/// c.op(a)
{
	int start=m_nStart;
	int end=m_nEnd;

	if(start<0) start=0;
	if(end>a.rows()) end=a.rows();
	ASSERT((end-start)%m_nXn==0);

	c.setSize((end-start)/m_nXn, a.cols());

	c.setAllValue(0);
	for(int i=0; i<c.rows(); i++)
	{
		for(int j=0; j<m_nXn; j++)
		{
			c.row(i)+=a.row(i*m_nXn+j);
		}
		c.row(i)/=(m_real)m_nXn;
	}
}

void m1::covariance::calc(matrixn& c, const matrixn& a) const
{
	vectorn mean;
	mean.aggregateColumn(CAggregate::AVG, a);

	matrixn zeroMeaned;
	zeroMeaned.each2(&vectorn::sub, a, mean);
	matrixn zeroMeaned_Columns;
	zeroMeaned_Columns.transpose(zeroMeaned);

	int dimension=a.cols();
	int numData=a.rows();

	c.setSize(dimension, dimension);

	for(int i=0; i<dimension; i++)
	{
		for(int j=0; j<dimension; j++)
		{
			c[i][j]=(zeroMeaned_Columns.row(i)%zeroMeaned_Columns.row(j))/(numData-1);
		}
	}
}
/*
void m1::NR_OLD::SVDInverse::calc(matrixn& c, const matrixn& a) const
{
	c.inverse(a);
}

void m1::NR_OLD::PseudoInverse::calc(matrixn& c, const matrixn& a) const
{
	c.pseudoInverse(a);
}

void m2::each::calc(matrixn& c, const matrixn& a, const matrixn& b) const
{
	// component wise
	ASSERT(a.rows()==b.rows());
	
	// output size detection
	vectorn outfirst;
	m_cOperator.calc(outfirst, a[0], b[0]);
	c.setSize(a.rows(), outfirst.size());
	c[0].assign(outfirst);
	
	for(int i=1; i<a.rows(); i++)
		m_cOperator.calc(c[i], a[i], b[i]);
}
*/

void v0::uniformSampling::calc(vectorn& c) const
{
	if(nSize!=-1)
		c.setSize(nSize);

	// simple sampling
	m_real len=x2-x1;
	m_real factor=1.f/((m_real)c.size());

	for(int i=0; i<c.size(); i++)
	{
		m_real position=((m_real)(i+(i+1)))*factor/2.f;
		c[i]=position*len+x1;
	}
}


void v0::drawSignals::calc(vectorn& c) const
{
	CImage* pImage;
	pImage=CImageProcessor::DrawChart(c, CImageProcessor::LINE_CHART, m_fMin, m_fMax);

	CImagePixel cip(pImage);

	for(int i=0; i<m_vXplot.size(); i++)
	{
		cip.DrawVertLine(m_vXplot[i], 0, cip.Height(), RGB(0,0,0),true);
	}
	if(m_strCommentname!=NULL)cip.DrawText(0,15,m_strCommentname);
	CImageProcessor::SafeDelete(pImage, m_strFilename);
}

/*
void h1::each::calc(hypermatrixn& c, const hypermatrixn& a) const
{
	// matrix�� �� row vector���� aggregate�Ѵ�. (��� vector dim�� rows())
	c.setSize(a.page(), a.rows(), a.cols());
	for(int i=0; i<a.page(); i++)
		m_cOperator.calc(c[i], a[i]);
}*/

/*void v0::colon::calcInt(intvectorn& c) const
{
	c.colon(m_nStart, m_nEnd, m_nStepSize);
}*/

void v1::freq::calc(vectorn& c, const vectorn& a) const
{
	matrixn fff;
	fff.spectrum(a, m_nWindowSize);
	c.setSize(a.size());

	vectorn index(m_nWindowSize/2);
	index.colon(0,1);
	vectorn temp;
	intvectorn sort;

	for(int i=0; i<c.size(); i++)
	{
		temp=fff.row(i).range(0, m_nWindowSize/2);
		c[i]=temp.argMax();	// argMAx -> nice but noisy?
		//c[i]=(index%temp)/temp.sum(); // weighted average->best accuracy but a bit noisy.-> no meaning.
/*
		// avg of 5 maximum
		sort.sortedOrder(temp);
		m_real wsum=0;
		m_real sum=0;
		for(int j=0; j<5; j++)
		{
//			printf("%d_%f_",sort[sort.size()-j-1], temp[sort[sort.size()-j-1]]);
			wsum+=(m_real)(sort[sort.size()-j-1])*temp[sort[sort.size()-j-1]];
			sum+=temp[sort[sort.size()-j-1]];
		}

//		printf(" : %d %f, %f, %f \n",i, wsum, sum, wsum/sum);
		c[i]=wsum/sum;*/
	}
}

void v1::downSampling::calc(vectorn& c, const vectorn& a) const/// c.op(a)
{
	int start=m_nStart;
	int end=m_nEnd;

	if(start<0) start=0;
	if(end>a.size()) end=a.size();
	ASSERT((end-start)%m_nXn==0);

	c.setSize((end-start)/m_nXn);


	c.setAllValue(0);
#ifdef USE_BOX_FILTER
	for(int i=0; i<c.size(); i++)
	{
		for(int j=0; j<m_nXn; j++)
		{
			c[i]+=a[i*m_nXn+j];
		}
		c[i]/=(m_real)m_nXn;
	}
#else
	// USE_GAUSSIAN_FILTER

	vectorn kernel;
	Filter::GetGaussFilter(m_nXn*2, kernel);

	for(int i=0; i<c.size(); i++)
	{
		for(int j=0; j<m_nXn*2; j++)
		{
			int jj=j-m_nXn/2;	// range -> [-X/2, X+ -X/2]
			c[i]+=a[CLAMP(i*m_nXn+jj, 0, a.size()-1)]*kernel[j];
		}
	}

#endif
}


void m::multA_diagB(matrixn& c, matrixn const& a, vectorn const& b)
{
	c.setSameSize(a);
	for(int i=0; i<c.cols(); i++)
		c.column(i).mult(a.column(i), b[i]);
}


m_real m::vMv(vectorn const& v, matrixn const& M)
{
	/* ����� ����
	static vectorn vM;
	vM.multmat(v, M);
	return vM%v;*/

	// ���� ����.
	int b1,b2;
    m_real diff1,diff2;
    double sum;

    sum = 0;
	int nbands=v.size();
    for(b1=0; b1<nbands; b1++) 
    for(b2=0; b2<nbands; b2++)
    {
      diff1 = v[b1];
      diff2 = v[b2];
      sum += diff1*diff2*M[b1][b2];
    }
	return sum;
}

m_real m::sMs(vectorn const& a, vectorn const& b, matrixn const& M)
{
	/* ����� ����
	static vectorn s;
	s.sub(a,b);
	return m::vMv(s,M);*/

	// ���� ����. (�ӵ� ���� ���� ��)
	int b1,b2;
    m_real diff1,diff2;
    double sum;

    sum = 0;
	int nbands=a.size();
    for(b1=0; b1<nbands; b1++) 
    for(b2=0; b2<nbands; b2++)
    {
      diff1 = a[b1]-b[b1];
      diff2 = a[b2]-b[b2];
      sum += diff1*diff2*M[b1][b2];
    }
	return sum;

}

m_real m::ss(vectorn const& a, vectorn const& b)
{
	m_real ss=0;
	for(int i=0; i<a.size(); i++)
	{
		ss+=SQR(a[i]-b[i]);
	}
	return ss;
}
m_real m::vDv(vectorn const& v, vectorn const& diagM)
{
    int b1;
    double diff1,diff2;
    double sum;

    sum = 0;
	int nbands=v.size();
    for(b1=0; b1<nbands; b1++) 
    {
      diff1 = v[b1];      
      sum += diff1*diff1*diagM[b1];
    }
	return sum;
}

m_real m::sDs(vectorn const& a, vectorn const& b, vectorn const& diagM)
{
    int b1;
    double diff1,diff2;
    double sum;

    sum = 0;
	int nbands=a.size();
    for(b1=0; b1<nbands; b1++) 
    {
      diff1 = a[b1]-b[b1];
      
      sum += diff1*diff1*diagM[b1];
    }

	return sum;
}

void m::eigenDecomposition(matrixn const& cov, vectorn & d, matrixn & v, int method)
{
	const bool sort=true;

	switch(method)
	{
	case 0:
		{
			int nrot;
			matrixn a=cov;
			NR::jacobi(a, d,v, nrot);
		}
		break;
	case 1:
		{
			v=cov;
			vectorn e;
			d.setSize(v.rows());
			e.setSize(v.rows());
			NR::tred2(v, d, e);
			NR::tqli(d, e, v);
		}
		break;
	}
	if(sort)
		NR::eigsrt(d,v);
}

m_real m::determinant(const matrixn& E)
{
	/* transfer matrix for inversion into F */
	ASSERT(E.rows()==E.cols());

	if(E.rows()==1)
		return E[0][0];
	static matrixn C_temp;
	static vectorn lu_col;
	C_temp=E;
	lu_col.setSize(E.rows());
	m_real det=1.0;

	/* Use LU decomposition to invert E[][] and find det(E) */
	Vec_INT indx(E.rows());
	m_real d=DBL_MAX;
	NR::ludcmp(C_temp, indx, d);
	int n=E.rows();

	if(d==DBL_MAX)	// singular matrix
		return 0.0;
	
	for(int i=0; i < n; ++i)
	{
		det*= C_temp[i][i];
	}
	return det*d;
}

void m1::inverseDiagonal ::calc(matrixn& c, matrixn const& a) const
{
	c.setSameSize(a);
	c.setAllValue(0.0);
	for(int i=0; i<a.rows(); i++)
		c[i][i]=1.0/a[i][i];
}

void m1::cofactor::calc(matrixn& c, const matrixn& a) const
{
	c.setSameSize(a);

	matrixn det(a.rows()-1, a.rows()-1);
	for(int i=0; i<a.rows(); i++)
	{
		for(int j=0; j<a.cols(); j++)
		{
			for(int k=0; k<i; k++)
			{
				for(int l=0; l<j; l++)
					det[k][l]=a[k][l];
				for(int l=j+1; l<a.cols(); l++)
					det[k][l-1]=a[k][l];
			}
			for(int k=i+1; k<a.rows(); k++)
			{
				for(int l=0; l<j; l++)
					det[k-1][l]=a[k][l];
				for(int l=j+1; l<a.cols(); l++)
					det[k-1][l-1]=a[k][l];
			}

			c[i][j]=m::determinant(det);
			if((i+j)%2==1)
				c[i][j]*=-1.0;
		}
	}
}


void m1::add::calc(matrixn& c, matrixn const& a) const
{
	for(int i=0; i<c.rows(); i++)
		for(int j=0; j<c.cols(); j++)
			c[i][j]+=a[i][j];
}
void m1::multAdd::calc(matrixn& c, matrixn const& a) const
{
	for(int i=0; i<c.rows(); i++)
		for(int j=0; j<c.cols(); j++)
			c[i][j]+=a[i][j]*mMult;
}

void m2::multAtBA::calc(matrixn& c, matrixn const& a, matrixn const& b) const
{
	c.multAtB(a, b*a);
}

void m2::multABAt::calc(matrixn& c, matrixn const& a, matrixn const& b) const
{
	c.multABt(a*b, a);
}

void m2::multABt::calc(matrixn& c, matrixn const& a, matrixn const& b) const
{
	if(&a==&c)
	{
		matrixn aa(a);
		calc(c, aa,b);
	}

	if(&b==&c)
	{
		matrixn bb(b);
		calc(c, a,bb);
	}

    assert( a.cols()==b.cols() );
    c.setSize( a.rows(), b.rows() );

    for( int i=0; i<a.rows(); i++ )
    for( int j=0; j<b.rows(); j++ )
    {
        c[i][j] = 0;
        for( int k=0; k<a.cols(); k++ )
            c[i][j] += a[i][k] * b[j][k];
    }
    
}
/*

void vm1::curvature::calc(vectorn& c, const matrixn& pos) const
{
	//http://mathworld.wolfram.com/Curvature.html

	//k = 	(| r.^ x r^..|)/(| r^. |^3)

	matrixn vel;
	matrixn acc;

	vel.derivative(pos);
	acc.derivative(vel);

	c.setSize(pos.rows());

	for(int i=0; i<c.size(); i++)
	{
		c[i]= vel[i].toVector3().cross(acc[i].toVector3()).length();
		c[i]/=CUBIC(vel[i].length());
	}
}

void vm1::_calcUtility(const sv1::Operator& cOP, vectorn& c, const matrixn& a)
{
	// matrix�� �� column vector���� aggregate�Ѵ�. (��� vector dim�� cols())
	vectorn column;
	c.setSize(a.cols());
	
	for(int i=0; i<a.cols();i++)
	{
		a.getColumn(i, column);
		c[i]=cOP.calc(column);
	}
}

void vm1::each::calc(vectorn& c, const matrixn& a) const
{
	// matrix�� �� row vector���� aggregate�Ѵ�. (��� vector dim�� rows())
	const matrixn& mat=a;
	c.setSize(mat.rows());
	for(int i=0; i<mat.rows(); i++)
		c[i]=m_cOperator.calc(mat[i]);
}

// a���� �Ϻθ� �̾Ƽ� ���.
vm1::domain::domain(int start, int end, int stepSize, const vm1::Operator& op)
	:m_op(op)
{
	m_aIndex.colon(start, end, stepSize);
}

void vm1::domain::calc(vectorn& c, const matrixn& a) const
{
	matrixn temp;
	temp.extractRows(a, m_aIndex);
	m_op.calc(c, temp);
}

vm1::domainRange::domainRange(int start, int end, int stepSize, const vm1::Operator& op)
	:m_op(op)
{
	m_aInOutIndex.colon(start, end, stepSize);
}

void vm1::domainRange::calc(vectorn& c, const matrixn& a) const
{
	// unary operator ���� c�� ������� �ʴ� ��찡 �ִ�. �̰�� ����ڰ� c�� ũ�⸦ ������ �������� �ʴ°�찡 �ִ�.
	// �̰�� ���� ũ�Ⱑ �ٸ��� ������ ���� �����Ƿ� �Ʒ��� �߰��Ͽ���. c�� a�� ũ�Ⱑ ���� �Ϲ����� ��� �������.
	c.setSize(a.rows());

	matrixn tempa;
	vectorn tempc;
	tempa.extractRows(a, m_aInOutIndex);
	tempc.extract(c, m_aInOutIndex);
	m_op.calc(tempc, tempa);

	c.op1(v1::assign(m_aInOutIndex), tempc);
}

m_real sv1::domain::calc(const vectorn& c) const
{
	vectorn temp;
	temp.extract(c, m_aIndex);
	return temp.op1(m_op);
}

m_real sv1::_calcUtil(CAggregate::aggregateOP eOP, const vectorn& c) 
{
	CAggregate cOP(eOP);
	m_real cur=cOP.Init();

	for( int i=0; i<c.size(); i++ )
		cOP.Update(cur, c[i]);
	
	return cOP.Final(cur, c.size());
}

void NR_OLD::tridag(const vectorn& a, const vectorn& b, const vectorn& c, const vectorn& r, vectorn& u)
{
	int j;
	m_real bet;

	int n=a.size();
	u.setSize(n);

	vectorn gam(n);	// one vector of workspace
	if(b[0]==0.0) ASSERT(0);
	// if this happens then you should rewrite your equations as a set of order N-1, with u1 trivially eliminated
	u[0]=r[0]/(bet=b[0]);

	// decomposition and forward substitution
	for(j=1; j<n; j++)
	{
		gam[j]=c[j-1]/bet;
		bet=b[j]-a[j]*gam[j];
		if(bet==0.0) ASSERT(0);	// algorithm fails
		u[j]=(r[j]-a[j]*u[j-1])/bet;
	}

	// backsubstitution
	for(j=n-2; j>=0; j--)
		u[j]-=gam[j+1]*u[j+1];
}


void NR_OLD::SVdecompose( matrixn &a , vectorn& w, matrixn& v )
{
	int m = a.rows();
	int n = a.cols();

	w.setSize( n );
	v.setSize( n, n );

	int flag, i, its, j, jj, k, l, nm;
	m_real anorm, c, f, g, h, s, scale, x, y, z;

  	static vectorn rv1; rv1.setSize( n );
	g = scale = anorm = 0.0;

	for( i=0; i<n; i++ )
	{
		l = i + 1;
		rv1[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m )
		{
			for ( k=i; k<m; k++ )
				scale += fabs(a[k][i]);

			if ( scale )
			{
				for ( k=i; k<m; k++ )
				{
					a[k][i] /= scale;
					s += a[k][i] * a[k][i];
				}

				f = a[i][i];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][i] = f - g;

				for( j=l; j<n; j++ )
				{
					for ( s=0.0, k=i; k<m; k++ )
						s += a[k][i] * a[k][j];
					f = s / h;

					for ( k=i; k<m; k++ )
						a[k][j] += f * a[k][i];
				}

				for( k=i; k<m; k++ )
					a[k][i] *= scale;
			}
		}

		w[i] = scale * g;
		g = s = scale = 0.0;

		if ( i<m && i != n-1)
		{
			for( k=l; k<n; k++)
				scale += fabs(a[i][k]);

			if ( scale )
			{
				for( k=l; k<n; k++ )
				{
					a[i][k] /= scale;
					s += a[i][k] * a[i][k];
				}

				f = a[i][l];
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				a[i][l] = f - g;

				for ( k=l; k<n; k++ )
					rv1[k] = a[i][k] / h;

				for( j=l; j<m; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[j][k] * a[i][k];

					for( k=l; k<n; k++ )
						a[j][k] += s * rv1[k];
				}

				for( k=l; k<n; k++ )
					a[i][k] *= scale;
			}
		}

		anorm = MAX(anorm, (fabs(w[i]) + fabs(rv1[i])));
	}

	for( i=n-1; i>=0; i-- )
	{
		if ( i<n-1 )
		{
			if ( g )
			{
				for( j=l; j<n; j++ )
					v[j][i] = (a[i][j] / a[i][l]) / g;

				for ( j=l; j<n; j++ )
				{
					for( s=0.0, k=l; k<n; k++ )
						s += a[i][k] * v[k][j];

					for( k=l; k<n; k++ )
						v[k][j] += s * v[k][i];
				}
			}

			for( j=l; j<n; j++ )
				v[i][j] = v[j][i] = 0.0;
		}

		v[i][i] = 1.0;
		g = rv1[i];
		l = i;
	}

	for( i=MIN(m, n)-1; i>=0; i-- )
	{
		l = i + 1;
		g = w[i];
		for( j=l; j<n; j++ )
		a[i][j] = 0.0;

		if ( g )
		{
			g = (m_real)1.0 / g;
			for( j=l; j<n; j++ )
			{
				for ( s=0.0, k=l; k<m; k++ )
					s += a[k][i] * a[k][j];

				f = (s / a[i][i]) * g;

				for( k=i; k<m; k++ )
					a[k][j] += f * a[k][i];
			}

			for( j=i; j<m; j++ )
				a[j][i] *= g;
		}
		else
			for( j=i; j<m; j++ )
				a[j][i] = 0.0;

		++a[i][i];
	}

	for( k=n-1; k>=0; k-- )
	{
		for( its=1; its<30; its++ )
		{
			flag = 1;
			for( l=k; l>=0; l-- )
			{
				nm = l - 1;
				if ((m_real) (fabs(rv1[l]) + anorm) == anorm)
				{
					flag = 0;
					break;
				}

				if ((m_real) (fabs(w[nm]) + anorm) == anorm) break;
			}

			if ( flag )
			{
				c = 0.0;
				s = 1.0;

				for( i=l; i<= k; i++ )
				{
					f = s * rv1[i];
					rv1[i] = c * rv1[i];

					if ((m_real) (fabs(f) + anorm) == anorm) break;

					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0f / h;
					c = g * h;
					s = -f * h;

					for( j=0; j<m; j++ )
					{
						y = a[j][nm];
						z = a[j][i];
						a[j][nm] = y * c + z * s;
						a[j][i] = z * c - y * s;
					}
				}
			}

			z = w[k];
			if ( l == k )
			{
				if ( z < 0.0 )
				{
					w[k] = -z;
					for( j=0; j<n; j++ )
						v[j][k] = -v[j][k];
				}
				break;
			}

			if (its == 29)
				error("no convergence in 30 svdcmp iterations");

			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0f * h * y);
			g = pythag(f, 1.0f);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0f;

			for( j=l; j<=nm; j++ )
			{
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;

				for( jj=0; jj<n; jj++ )
				{
					x = v[jj][j];
					z = v[jj][i];
					v[jj][j] = x * c + z * s;
					v[jj][i] = z * c - x * s;
				}

				z = pythag(f, h);
				w[j] = z;

				if ( z )
				{
					z = 1.0f / z;
					c = f * z;
					s = h * z;
				}

				f = c * g + s * y;
				x = c * y - s * g;

				for( jj=0; jj<m; jj++ )
				{
					y = a[jj][j];
					z = a[jj][i];
					a[jj][j] = y * c + z * s;
					a[jj][i] = z * c - y * s;
				}
			}

			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
	}
}

void NR_OLD::SVsubstitute( const matrixn& u, vectorn const& w, matrixn const& v,
            const vectorn& b, vectorn &x ) 
{
    assert( u.cols() == w.getSize() );
    assert( u.cols() == v.cols() );
    assert( u.cols() == v.rows() );
    assert( u.rows() == b.getSize() );
    assert( u.cols() == x.getSize() );

    int m = u.rows();
    int n = u.cols();

    int jj,j,i;
    m_real s;
    static vectorn tmp; tmp.setSize(n);

    for (j=0;j<n;j++) {
        s=0.0;
        if (w[j]>EPS) {
            for (i=0;i<m;i++) s += u[i][j]*b[i];
            s /= w[j];
        }
        tmp[j]=s;
    }
    for (j=0;j<n;j++) {
        s=0.0;
        for (jj=0;jj<n;jj++) s += v[j][jj]*tmp[jj];
        x[j]=s;
    }
}

void NR_OLD::SVinverse( matrixn& in_u, matrixn &mat ) 
{
    int m = in_u.rows();
    int n = in_u.cols();

    static matrixn V; V.setSize( n, n );
    static vectorn w; w.setSize( n );

    static vectorn b; b.setSize( m );
    static vectorn x; x.setSize( n );

	mat.setSize( n, m );

    SVdecompose( in_u, w, V );
    for( int j=0; j<m; j++ )
    {
        for( int i=0; i<m; i++ ) b[i] = 0;
        b[j] = 1.0;

        SVsubstitute(in_u, w, V, b, x );

        for( i=0; i<n; i++ )
            mat[i][j] = x[i];
    }
}

m_real NR_OLD::pythag( m_real a, m_real b )
{
	m_real pa = fabs( a );
	m_real pb = fabs( b );

	if ( pa > pb ) return pa * sqrt( 1.0f + SQR(pb / pa) );
	else return (pb==0.0f ? 0.0f : pb * sqrt(1.0f + SQR(pa / pb)));
}

void NR_OLD::tred2(matrixn& a, vectorn& d, vectorn& e)
{
	int l,k,j,i;
	double scale,hh,h,g,f;

	ASSERT(a.rows()==a.cols());

	int n=a.rows();
	d.setSize(n);
	e.setSize(n);

	for (i=n-1;i>0;i--) {
		l=i-1;
		h=scale=0.0;
		if (l > 0) {
			for (k=0;k<l+1;k++)
				scale += fabs(a[i][k]);
			if (scale == 0.0)
				e[i]=a[i][l];
			else {
				for (k=0;k<l+1;k++) {
					a[i][k] /= scale;
					h += a[i][k]*a[i][k];
				}
				f=a[i][l];
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e[i]=scale*g;
				h -= f*g;
				a[i][l]=f-g;
				f=0.0;
				for (j=0;j<l+1;j++) {
				// Next statement can be omitted if eigenvectors not wanted
					a[j][i]=a[i][j]/h;
					g=0.0;
					for (k=0;k<j+1;k++)
						g += a[j][k]*a[i][k];
					for (k=j+1;k<l+1;k++)
						g += a[k][j]*a[i][k];
					e[j]=g/h;
					f += e[j]*a[i][j];
				}
				hh=f/(h+h);
				for (j=0;j<l+1;j++) {
					f=a[i][j];
					e[j]=g=e[j]-hh*f;
					for (k=0;k<j+1;k++)
						a[j][k] -= (f*e[k]+g*a[i][k]);
				}
			}
		} else
			e[i]=a[i][l];
		d[i]=h;
	}
	// Next statement can be omitted if eigenvectors not wanted
	d[0]=0.0;
	e[0]=0.0;
	// Contents of this loop can be omitted if eigenvectors not
	//	wanted except for statement d[i]=a[i][i];
	for (i=0;i<n;i++) {
		l=i;
		if (d[i] != 0.0) {
			for (j=0;j<l;j++) {
				g=0.0;
				for (k=0;k<l;k++)
					g += a[i][k]*a[k][j];
				for (k=0;k<l;k++)
					a[k][j] -= g*a[k][i];
			}
		}
		d[i]=a[i][i];
		a[i][i]=1.0;
		for (j=0;j<l;j++) a[j][i]=a[i][j]=0.0;
	}
}


void NR_OLD::tqli(matrixn& z, vectorn& d, vectorn& e)
{
	int size=d.size();

	int m,l,iter,i,k;
	double s,r,p,g,f,dd,c,b;

	int n=size;
	for (i=1;i<n;i++) e[i-1]=e[i];
	e[n-1]=0.0;
	for (l=0;l<n;l++) {
		iter=0;
		do {
			for (m=l;m<n-1;m++) {
				dd=fabs(d[m])+fabs(d[m+1]);
				if (fabs(e[m])+dd == dd) break;
			}
			if (m != l) {
				if (iter++ == 30) {
//					MESSAGE("Too many iterations in tqli");
					return;
				}
				g=(d[l+1]-d[l])/(2.0*e[l]);
				r=pythag(g,1.0);
				g=d[m]-d[l]+e[l]/(g+SIGN(r,g));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e[i];
					b=c*e[i];
					e[i+1]=(r=pythag(f,g));
					if (r == 0.0) {
						d[i+1] -= p;
						e[m]=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d[i+1]-p;
					r=(d[i]-g)*s+2.0*c*b;
					d[i+1]=g+(p=s*r);
					g=c*r-b;
					// Next loop can be omitted if eigenvectors not wanted
					for (k=0;k<n;k++) {
						f=z[k][i+1];
						z[k][i+1]=s*z[k][i]+c*f;
						z[k][i]=c*z[k][i]-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d[l] -= p;
				e[l]=g;
				e[m]=0.0;
			}
		} while (m != l);
	}
}*/



void s1::COS(m_real&b,m_real a)  {b= (m_real)cos(a);}
void s1::SIN(m_real&b,m_real a)  {b= (m_real)sin(a);}
void s1::EXP(m_real&b,m_real a)  {b= (m_real)exp(a);}
void s1::NEG(m_real&b,m_real a)  {b= -1*a;}
void s1::SQRT(m_real&b,m_real a)  { b= sqrt(a);}
void s1::SQUARE(m_real&b,m_real a)  { b= a*a;}
void s1::ASSIGN(m_real&b,m_real a)  { b= a;}
void s1::LOG(m_real&b,m_real a)  { b= log(a);}
void s1::abs(m_real&b,m_real a)  { b= (m_real)ABS(a);}
void s1::SMOOTH_TRANSITION(m_real&b,m_real a)  { b= ((m_real)-2.0)*a*a*a+((m_real)3.0)*a*a;} // y=-2x^3+3x^2
void s1::RADD(m_real&b,m_real a)  { b+=a;}
void s1::RDIV(m_real&b,m_real a)  { b/=a;}
void s1::RSUB(m_real&b,m_real a)  { b-=a;}
void s1::RMULT(m_real&b,m_real a)  { b*=a;}
void s1::BOUND(m_real&b, m_real a)  { b=CLAMP(b, -1*a, a);}
void s1::INVERSE(m_real&b, m_real a) { b=1.0/a;	}


m_real s2::ADD(m_real a, m_real b)  {return a+b;}
m_real s2::SUB(m_real a, m_real b)  {return a-b;}
m_real s2::MULT(m_real a, m_real b)  {return a*b;}
m_real s2::DIV(m_real a, m_real b)  {return a/b;}
m_real s2::POW(m_real a, m_real b)  {return pow(a,b);}
m_real s2::MINIMUM(m_real a, m_real b)  {return MIN(a,b);}
m_real s2::MAXIMUM(m_real a, m_real b)  {return MAX(a,b);}
m_real s2::GREATER(m_real a, m_real b)  { return (m_real) a>b;}
m_real s2::GREATER_EQUAL(m_real a, m_real b)  { return (m_real) a>=b;}
m_real s2::SMALLER(m_real a, m_real b)  { return (m_real) a<b;}
m_real s2::SMALLER_EQUAL(m_real a, m_real b)  { return (m_real) a<=b;}
m_real s2::EQUAL(m_real a, m_real b)  { return (m_real) a==b;}
m_real s2::AVG(m_real a, m_real b)  { return (a+b)/(m_real)2.0;}
m_real s2::BOUND(m_real a, m_real b)  { return CLAMP(a, -1*b, b);}
int s2::INT_NOT_EQUAL(int a, int b)		{ return a!=b;}
int s2::INT_EQUAL(int a, int b)		{ return a==b;}

m_real CAggregate::InitZero() const	{ return 0;};
m_real CAggregate::InitFMax() const	{ return FLT_MAX;};
m_real CAggregate::InitFMin() const	{ return -FLT_MAX;};
void CAggregate::UpdateSquareSum(m_real &cur, m_real v) const { cur+=v*v;}
void CAggregate::UpdateMin(m_real &cur, m_real v) const{ cur=MIN(cur,v);}
void CAggregate::UpdateMax(m_real &cur, m_real v) const{ cur=MAX(cur,v);}
void CAggregate::UpdateSum(m_real &cur, m_real v) const{ cur+=v;}
m_real CAggregate::FinalSqrt(m_real cur, int n) const	{ return sqrt(cur);}
m_real CAggregate::FinalCur(m_real cur, int n) const	{ return (cur);}
m_real CAggregate::FinalDivN(m_real cur, int n) const	{ return (cur/(m_real)n);}
