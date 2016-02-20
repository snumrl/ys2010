// TextFile.h: interface for the CTextFile class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_)
#define AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// �׻� NUM_UNDO�� �̳��� undo�� ������ Ŭ����. �� GetToken()=="ROOT"���ٸ�, �̸� undo �� �ٽ� GetToken�ϸ� �ٽ� "ROOT"�� ���´�. GetLine()�� undo�� GetToken()�� �����ϴ�.
#include "typestring.h"

class CTextFile  
{
public:
	CTextFile();
	virtual ~CTextFile();
	bool OpenReadFile(const char *fileName);
	void CloseFile();

	char* GetToken(bool& bLineChanged);
	char* GetToken()						{ bool bLineChanged; return GetToken(bLineChanged);};
	char* GetLine();
	void Undo();
private:
	enum { NUM_UNDO=3, NUM_LINES};
	void Init();
	char* Strtok();
	bool ReadOneLine();
	void SaveUndoState();
	FILE *m_pFile;
	TString m_strSeps;
	
	char m_szOutput[4096];
	
	char* m_pToken;
	bool m_bToUpper;
	
	class State
	{
	public:
		void Init()	{nIndex=0; nLine=-1;}
		void Clone(const State& other)	{ nIndex=other.nIndex; nLine=other.nLine;};
		int nIndex;
		int nLine;
	};

	int m_nCurrState;
	State m_aStates[NUM_UNDO];

	int m_nEmptyLine;
	char m_aszLine[NUM_LINES][4096];
	bool m_abEOF[NUM_LINES];
};

#endif // !defined(AFX_TEXTFILE_H__1F48B92D_AA9E_4B20_8F24_4F72D9B0C87E__INCLUDED_)
