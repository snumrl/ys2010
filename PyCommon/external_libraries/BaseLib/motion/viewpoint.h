#ifndef _VIEWPOINT_H_
#define _VIEWPOINT_H_




//! �ϳ��� ������ �����Ѵ�.
/*!
ViewPosition, ViewUp, ViewDir, ViewAt�� ������ �ְ� ViewUp,ViewAt,ViewDir�� ���� �浵 ������ε� ���� �ִ�.
quaternion�� ����ϵ��� �������� �ʿ��ϴ�.*/

class Viewpoint{
public:

	Viewpoint() { m_bTurnAround=TRUE; m_fMinAngle=0; m_scale=300;};
	virtual ~Viewpoint() {};

	int TurnDown(m_real radian);
	int TurnUp(m_real radian);
	int TurnRight(m_real radian);
	int TurnLeft(m_real radian);

	int CameraTurnDown(m_real radian);
	int CameraTurnUp(m_real radian);
	int CameraTurnRight(m_real radian);
	int CameraTurnLeft(m_real radian);

	int PanDown(m_real pan_amt);
	int PanUp(m_real pan_amt);
	int PanRight(m_real pan_amt);
	int PanLeft(m_real pan_amt);
	int PanForward(m_real pan_amt);
	int PanBackward(m_real pan_amt);

	int ZoomIn(m_real ZoomAmount);
	int ZoomOut(m_real ZoomAmount);

	int CalcDepth();
	int CalcHAngle();
	int CalcVAngle();

	void ReadViewPoint(FILE *infile);
	void WriteViewPoint(FILE *fpFile);

	int CheckConstraint();
	int UpdateVPosFromVHD();

	int GetViewMatrix(matrix4& matView);
	int UpdateVHD();

	void interpolate(m_real t, const Viewpoint & a, const Viewpoint & b);
	Viewpoint& operator=(const Viewpoint& other);
	// utility
	m_real CalcNearMinY(); //���� view frustum���� near plane�� ���� ���� ���� z��ǥ(�ٻ簪)
	
	vector3 m_vecVPos;
	vector3 m_vecVAt;
	vector3 m_vecVUp;
	m_real m_fVAngle;
	m_real m_fHAngle;
	m_real m_fDepth;

	bool m_bTurnAround;
	m_real m_fMinAngle;
	m_real m_fDesiredZoom;

	int m_iWidth;
	int m_iHeight;
	m_real m_scale;
	void setScale(m_real f) {m_scale=f;}	// zoom�̳� pan�� �Ҷ� ������� scale�� ����.

	inline void GetViewDir(vector3& temp) { temp=m_vecVAt-m_vecVPos;}

	LRESULT     HandleMouseMessages2( HWND hwnd, UINT uMsg, WPARAM wparam, int iMouseX, int iMouseY);
private:
};

#endif