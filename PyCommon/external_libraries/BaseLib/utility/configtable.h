// ConfigTable.h: interface for the ConfigTable class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_)
#define AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define CT_MAX_ITEM 100
#define CT_NBUCKET 100
#define CT_NCOUNT 30
#define MAX_ITEM 100

#include "namedMapSupport.h"

#include <TCHAR.H>
typedef std::map<std::string , TCHAR *, ltstr> namedmapLPTCHAR;

//! macro function .. usage : CTReadInt(m_ConfigTable, m_bFullScreen); �ǹ�: m_bFullScreen�� config file���� �о�ͼ� setting�Ѵ�.
#define CTReadInt(cname,vname)			vname=cname.GetInt(#vname)
#define CTReadFloat(cname,vname)		vname=cname.GetFloat(#vname)
#define CTReadString(cname,vname)		cname.GetString(vname,#vname)

//! ���Ͽ��� ������ �����ϰų� �о���� Ŭ���� Key�� Content�� �� string���� �����ȴ�.
/*!
a.txt
_________________________
key1 13
key2 15
_________________________
�� �ϸ�, 
GetInt(string("key1"))==13 �� ����
Find("key1") �ϸ� "13"�� return �ȴ�.

*/
class ConfigTable  
{
public:
	ConfigTable(const char *szFileName);
	virtual ~ConfigTable();

	TCHAR *Find(std::string &name);
	TCHAR *Find(TCHAR * name);

	int GetInt(const char* szName);
	float GetFloat(const char* szName);
	void GetString(char* output, const char* szName);

	//! key�� �ش��ϴ� ���� ������ �ٲ���, ���Ͽ� �����Ѵ�.
	void WriteInt(char szName[100], int num);
	void WriteFloat(char szName[100], float num);

	void WriteTable();
private:
	void Initialize(const char *szFileName);
	void Release();
	void Insert(std::string &key, int value);

	namedmapLPTCHAR m_namedmapContent;

	char m_szFileName[100];
};

#endif // !defined(AFX_CONFIGTABLE_H__BDBC6D61_7A07_11D5_9D95_0050BF107BFC__INCLUDED_)
