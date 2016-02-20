#pragma once

class vector3;
class quater;
/// never make virtual functions. 
/** ����: �̰��� �ּҴ� direct�ϰ� D3DXMATRIX�� �ּҷ� �ٲ�� �ִ�. virtual function�� ����� VTable������ �̰��� �ȵȴ�.
�ٸ�, �������� opengl�� ������. ���ǻ��� ����.

���� ����: 
quaternion�� q1*q2*v���·� ���ǻ� �����ʲ� ���� ���� �������ٰ� �����ؾ� �Ѵ�.
openGL matrix�� quaternion�� ���������� R1*r2*v���·� �����ʿ� ���� ��������.
������ D3DXMATRIX�� v*R1*R2���·� ���� ���ʿ� ��������, ���������� D3DXQUATERNION�� q1*q2�� ���� ������ q2*q1�� ��ġ�ϵ��� 
���ϱ������ �ٲ�� �ִ�. �� D3DXMATRIX�� D3DXQUATERNION��� ���ʿ� �ִ� ���� ����, ���� �����ʿ� �ִ� ���� �۷ι��̴�.
�ݸ� quaterŬ������ matrix4Ŭ������ openGL�� convention�� ������. �� ���� �����ʿ��� ��������.
������ convention�� ���߱� ���ؼ� toDXmat�� fromDXmat�Լ��� ���� ���� transpose�� ���ִ� ���� �� �� �ִ�.
\ingroup group_math
*/

class matrix4
{
public:

	union {
        struct {
            m_real        _11, _12, _13, _14;
            m_real        _21, _22, _23, _24;
            m_real        _31, _32, _33, _34;
            m_real        _41, _42, _43, _44;

        };
        m_real m[4][4];
    };

#ifdef DIRECT3D_VERSION
	matrix4(const D3DXMATRIX& mat)				{ fromDXmat(mat);};
#endif
	matrix4(const matrix4& mat)					{ memcpy(this, &mat, sizeof(matrix4));}
	matrix4(const quater& rot, const vector3& trans)	{ setRotation(rot); setTranslation(trans);}
	matrix4();
	~matrix4();

	void identity()	{ setIdentityRot(); setTranslation(vector3(0.0, 0.0, 0.0));}

	// n-ary operators
	void lookAtLH(const vector3& eye, const vector3& at, const vector3& up);
	void lookAtRH(const vector3& eye, const vector3& at, const vector3& up);
	void setScaling(m_real sx, m_real sy, m_real sz);
	void setTranslation(const vector3& tx);		//!< �ٸ� set�迭 �Լ��� �޸� rotation��Ʈ�� �ǵ帮�� �ʴ´�. 	
	void setTranslationMat(const vector3& tx)	{ setIdentityRot(); setTranslation(tx);}
	void setTransform(const quater& rot, const vector3& trans);
	void setRotation(const quater& q);
	void setRotation(const vector3& axis, m_real angle);
	void setRotationX(m_real angle);
	void setRotationY(m_real angle);
	void setRotationZ(m_real angle);
	void setRotation(const char* aChannel, m_real *aValue, bool bRightToLeft=false);	//!< from euler angle. aChannel="YXZ" or something like that.
	void setAxisRotation(const vector3& vecAxis, const vector3& front, const vector3& vecTarget); 	//!< front���͸� vecAxis�� �߽����� ȸ���ؼ� vecTarget�� vecAxis�� �̷�� ��鿡 ���̵��� ����� Matrix�� ���Ѵ�.
	void setIdentityRot();

	void transpose();
	
	// left multiplication. eg) a=matScale*a
	void leftMult(m_real scalar);
	void leftMultScaling(m_real sx, m_real sy, m_real sz);
	void leftMultRotation(const quater& b);
	void leftMultRotation(const vector3& axis, m_real angle);
	void leftMultTranslation(const vector3& vec);
	void mult(const matrix4& a, const matrix4& b);
	void mult(const matrix4& a, const quater& b);
	void mult(const quater& a, const matrix4& b);


	// unary operators
	void adjoint(const matrix4& a);
	void inverse(const matrix4& a);
	void extractRot(const matrix4& a);
	void transpose(const matrix4& a);
	void leftMult(const matrix4& a);	//!< this=a*this;
	void operator*=(const matrix4& a);	//!< this=this*a;

	matrix4 operator*(matrix4 const& a) const { matrix4 t; t.mult(*this,a); return t;}
	
	// inquiry functions
	void decomposeLH(vector3& eye, vector3& at, vector3& up);
	void decomposeRH(vector3& eye, vector3& at, vector3& up);

	void setValue( m_real x00, m_real x01, m_real x02,
				   m_real x10, m_real x11, m_real x12,		// �������� 0,1���� �ʱ�ȭ 
				   m_real x20, m_real x21, m_real x22 )	
	{ m[0][0]=x00, m[0][1]=x01, m[0][2]=x02,  m[1][0]=x10, m[1][1]=x11, m[1][2]=x12,  
m[2][0]=x20, m[2][1]=x21, m[2][2]=x22,  m[0][3]=0,m[1][3]=0,m[2][3]=0,m[3][0]=0,m[3][1]=0,m[3][2]=0,m[3][3]=1;}

	void setValue( m_real x00, m_real x01, m_real x02, m_real x03,
				   m_real x10, m_real x11, m_real x12, m_real x13,
				   m_real x20, m_real x21, m_real x22, m_real x23,
				   m_real x30, m_real x31, m_real x32, m_real x33)	
	{ m[0][0]=x00, m[0][1]=x01, m[0][2]=x02,  m[0][3]=x03,
	  m[1][0]=x10, m[1][1]=x11, m[1][2]=x12,  m[1][3]=x13,
	  m[2][0]=x20, m[2][1]=x21, m[2][2]=x22,  m[2][3]=x23,
	  m[3][0]=x30, m[3][1]=x31, m[3][2]=x32,  m[3][3]=x33;}

	m_real determinant() const;
	m_real minor(const size_t r0, const size_t r1, const size_t r2, 
				const size_t c0, const size_t c1, const size_t c2) const;


#ifdef DIRECT3D_VERSION

#ifdef MATH_DOUBLE_PRECISION
   void fromDXmat( const D3DXMATRIX &mat);
   void toDXmat(D3DXMATRIX& mat) const;
#else
	void fromDXmat( const D3DXMATRIX &mat)  { D3DXMatrixTranspose(*this, &mat);};
	void toDXmat(D3DXMATRIX& mat) const		{ D3DXMatrixTranspose(&mat,*this);};
private:
	//�Ʒ� �Լ��� D3DX�� �����ϴ� optimize�� ���ϱ� �Լ����� ����ϱ� ���� ���̴�. �� ���� �뵵�θ� ���Ǿ�� �Ѵ�.
	// �ܺο��� ����ϴ� �Լ��� ���� fromDXmat, toDXmató�� transpose�� �����Ѵ�.
	inline operator D3DXMATRIX*()	const	{ return (D3DXMATRIX*)this;};	
#endif
#endif
};

