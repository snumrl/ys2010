/////////////////////////////////////////////////////////////////////////////
//	PixelPointer.cpp

#include "stdafx.h"
#include "stdimage.h"
#include "ImagePixelPtr.h"

/////////////////////////////////////////////////////////////////
// CColorPixel
CColorPixel::CColorPixel()
{

}

CColorPixel::~CColorPixel()
{

}


/////////////////////////////////////////////////////////////////
// CColorPixelPtr
CColorPixelPtr::CColorPixelPtr(CImage &Im)
{
	m_nHeight = Im.GetHeight();		// �̹����� ���� ����
	int nWidth = Im.GetRealWidth();	// �̹����� ���� ����

	// �̹����� ���� ���� ��ŭ ������ �迭 �Ҵ�
	m_pPtr = new LPCOLOR [m_nHeight];

	// �޸� ����� ����
	m_hHandle = Im.GetHandle();
	LPSTR lpDIBHdr  = (LPSTR)::GlobalLock((HGLOBAL) m_hHandle);

	// ��� ������ ������ ù��° �ȼ��� �����͸� ����
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// ���� ���� ��ŭ�� �����ϸ� m_pPtr�� �̹����� 
	// �� ���� ù��° �ȼ��� �ּҸ� ����� ����
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += nWidth;
	}
}

CColorPixelPtr::CColorPixelPtr(HDIB hHandle)
{
	// �޸� ����� ����
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) hHandle);
	m_hHandle = hHandle;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// �̹����� ���� ����
	int nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*24);	// �̹����� ���� ����

	// �̹����� ���� ���� ��ŭ ������ �迭 �Ҵ�
	m_pPtr = new LPCOLOR [m_nHeight];

	// ��� ������ ������ ù��° �ȼ��� �����͸� ����
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// ���� ���� ��ŭ�� �����ϸ� m_pPtr�� �̹����� 
	// �� ���� ù��° �ȼ��� �ּҸ� ����� ����
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += nWidth;
	}
} 


CColorPixelPtr::CColorPixelPtr(LPSTR lpDIBHdr)
{
	m_hHandle = NULL;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// �̹����� ���� ����
	m_nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*24);	// �̹����� ���� ����

	// �̹����� ���� ���� ��ŭ ������ �迭 �Ҵ�
	m_pPtr = new LPCOLOR [m_nHeight];

	// ��� ������ ������ ù��° �ȼ��� �����͸� ����
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// ���� ���� ��ŭ�� �����ϸ� m_pPtr�� �̹����� 
	// �� ���� ù��° �ȼ��� �ּҸ� ����� ����
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += m_nWidth;
	}
} 

CColorPixelPtr::~CColorPixelPtr()
{ 
	// �޸� ����� Ǯ����
	if(m_hHandle )
		// �Ҵ� �޾Ҵ� ������ �迭�� ����
		GlobalUnlock((HGLOBAL) m_hHandle);
	
	delete [] m_pPtr;	
}

/////////////////////////////////////////////////////////////////
// CPixel
CPixel::CPixel()
{

}

CPixel::~CPixel()
{

}

CPixel::CPixel(int i)
{
	I = i;
}

/////////////////////////////////////////////////////////////////
// CColorPixelPtr

CPixelPtr::CPixelPtr(CImage &Im)
{
	m_nHeight = Im.GetHeight();		// �̹����� ���� ����
	int nWidth = Im.GetRealWidth();	// �̹����� ���� ����

	// �̹����� ���� ���� ��ŭ ������ �迭 �Ҵ�
	m_pPtr = new BYTE *[m_nHeight]; 
	
	// �޸� ����� ����
	m_hHandle = Im.GetHandle();
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) m_hHandle);

	// ��� ������ ������ ù��° �ȼ��� �����͸� ����
	BYTE *lpDIBBits = (BYTE *) ::FindDIBBits(lpDIBHdr);

	// ���� ���� ��ŭ�� �����ϸ� m_pPtr�� �̹����� 
	// �� ���� ù��° �ȼ��� �ּҸ� ����� ����
	for(int i=m_nHeight -1 ; i>=0 ; i--)
	{
		m_pPtr[i] = lpDIBBits;
		lpDIBBits += nWidth;
	}
}

 
CPixelPtr::CPixelPtr(HDIB hHandle)
{
	// �޸� ����� ����
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) hHandle);
	m_hHandle = hHandle;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// �̹����� ���� ����
	int nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*8);	// �̹����� ���� ����

	// �̹����� ���� ���� ��ŭ ������ �迭 �Ҵ�
	m_pPtr = new BYTE *[m_nHeight];

	// ��� ������ ������ ù��° �ȼ��� �����͸� ����
	BYTE *lpDIBBits = (BYTE *)::FindDIBBits(lpDIBHdr);

	// ���� ���� ��ŭ�� �����ϸ� m_pPtr�� �̹����� 
	// �� ���� ù��° �ȼ��� �ּҸ� ����� ����
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = lpDIBBits;
		lpDIBBits += nWidth;
	}
}

CPixelPtr::~CPixelPtr()
{
	// �޸� ����� Ǯ����
	GlobalUnlock((HGLOBAL) m_hHandle);
	// �Ҵ� �޾Ҵ� ������ �迭�� ����
	delete [] m_pPtr;
}

