// NameTable.h: interface for the NameTable class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NameTable_H__F2493FD8_DD43_437F_A824_961AF8095FA1__INCLUDED_)
#define AFX_NameTable_H__F2493FD8_DD43_437F_A824_961AF8095FA1__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "namedmapSupport.h"
#include <vector>

// name�� ����� �����ϰ�, name�� index�� �ٲٰų� index�� name���� �ٲٴ� ���� ȿ�������� ó���Ѵ�.
class NameTable
{
public:
	NameTable();
	virtual ~NameTable();

	void Insert(const char* name);
	bool find(const char* name);
	int operator[](const char* key);
	char* operator[](int keyvalue);
	void Clear();
	int Size()			{ return (int)m_aNames.size();};
private:
	
	char* Copy(const char* input)
	{
		char* string;
		string=new char[strlen(input)+1];
		strcpy(string, input);
		return string;
	}

	namedmapInt2 m_namedmap;
	std::vector<char*> m_aNames;

};

#endif // !defined(AFX_NameTable_H__F2493FD8_DD43_437F_A824_961AF8095FA1__INCLUDED_)
