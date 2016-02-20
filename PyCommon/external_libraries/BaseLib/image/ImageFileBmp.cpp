#include "stdafx.h"
#include "stdimage.h"
#include "Image.h"
#include "../baselib/utility/fileex.h"

#define DIB_HEADER_MARKER   ((WORD) ('M' << 8) | 'B')
/////////////////////////////////////////////////////////////////////////////
// CCImage load/save bmp

BOOL CImage::SaveBMP(LPCTSTR lpszFileName)
{
	CFileEx file;
	//CFileExException fe;
	BITMAPFILEHEADER bmfHdr;
	LPBITMAPINFOHEADER lpBI;
	DWORD dwDIBSize;

	// ���� ���� ���� ����
	if (!file.Open(lpszFileName, CREATE_ALWAYS, GENERIC_WRITE))
		return FALSE;

	// �޸� �ڵ��� ��ȿ���� Ȯ��
	if (m_hImage == NULL) return FALSE;

	// �޸� ����
	lpBI = (LPBITMAPINFOHEADER)::GlobalLock((HGLOBAL)m_hImage);
	if (lpBI == NULL) return FALSE;

	// ��Ʈ�� ���� ��� ������ ����
	bmfHdr.bfType = DIB_HEADER_MARKER;  // "BM"
	dwDIBSize = *(LPDWORD)lpBI + ::PaletteSize((LPSTR)lpBI);
	if((lpBI->biCompression==BI_RLE8) || (lpBI->biCompression==BI_RLE4))
		dwDIBSize += lpBI->biSizeImage;
	else 
	{
		DWORD dwBmBitsSize;  // Size of Bitmap Bits only
		dwBmBitsSize = WIDTHBYTES((lpBI->biWidth)*((DWORD)lpBI->biBitCount)) * lpBI->biHeight;
		dwDIBSize += dwBmBitsSize;
		lpBI->biSizeImage = dwBmBitsSize;
	}

	bmfHdr.bfSize = dwDIBSize + sizeof(BITMAPFILEHEADER);
	bmfHdr.bfReserved1 = 0;
	bmfHdr.bfReserved2 = 0;
	bmfHdr.bfOffBits=(DWORD)sizeof(BITMAPFILEHEADER)+lpBI->biSize + PaletteSize((LPSTR)lpBI);
	try
	{
		// ��Ʈ�� ���� ����� ���Ͽ� ����
		file.Write((BYTE*)&bmfHdr, sizeof(BITMAPFILEHEADER));
		// ������ �����͸� ���Ͽ� ����
		//file.WriteHuge(lpBI, dwDIBSize);
		file.Write((BYTE*)lpBI, dwDIBSize);
	}
	catch (CFileExException e)
	{
		::GlobalUnlock((HGLOBAL) m_hImage);
		//throwTHROW_LAST();
	}
	//END_CATCH

	// �޸� Ǯ����
	::GlobalUnlock((HGLOBAL) m_hImage);
	return TRUE;
}


BOOL CImage::LoadBMP(LPCTSTR lpszFileName)
{
	CFileEx file;
	LPSTR pDIB;
	DWORD dwBitsSize;
	BITMAPFILEHEADER bmfHeader;

	// �б� ���� ���� ����

	if(!file.Open(lpszFileName, OPEN_EXISTING, GENERIC_READ))
		return FALSE;

	// ������ ���̸� ����
	dwBitsSize = file.GetFileSize();

	// ���� ��� �б�
	if(file.Read((BYTE*)&bmfHeader, sizeof(bmfHeader))!=sizeof(bmfHeader))
		return FALSE;

	// BMP �������� ��Ÿ���� "BM" ��Ŀ�� �ִ��� Ȯ��
	if (bmfHeader.bfType != DIB_HEADER_MARKER)
		return FALSE;

	// �޸� �Ҵ�
	if((m_hImage = (HDIB)::GlobalAlloc(GMEM_MOVEABLE | GMEM_ZEROINIT, dwBitsSize)) == NULL) return FALSE;

	// �޸� ����
	pDIB = (LPSTR) ::GlobalLock((HGLOBAL) m_hImage);

	// ���� �б�
	if (file.Read((BYTE*)pDIB, dwBitsSize - sizeof(BITMAPFILEHEADER)) != dwBitsSize - sizeof(BITMAPFILEHEADER) ) 
	{
		::GlobalUnlock((HGLOBAL) m_hImage);
		::GlobalFree((HGLOBAL) m_hImage);
		return FALSE;
	}

	// �޸� Ǯ����
	::GlobalUnlock((HGLOBAL) m_hImage);

	// DIB �ʱ�ȭ
	InitDIB(false);

	return TRUE;
}
