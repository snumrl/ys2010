
#pragma once
 
#include "typestring.h"
class Parser
{
public:
	/// bTextMode�� ���, text file���·� �����Ѵ�.
	Parser(const char* filename, const char* seperator=" ,\t\n", bool bToUpper=false);
	virtual ~Parser();

	// seperator�� ���е� token�� return�Ѵ�. #���� �����ϴ� line�� �ּ����� �����Ѵ�.
	TString getToken();
private:
	FILE *m_pFile;
	char buff[4096];
	bool m_bToUpper;	
	TString mSeperator;
	char* token;
};

