#pragma once
#include <typeinfo.h>
#include "tarray.h"
#include "Typestring.h"

namespace sz0
{
	struct nop : public Operator
	{
		nop(){}
		virtual void calc(TString& c) const{}
	};

	/// op�� ������ ' '�� '0'�� �ٲ۴�. ex) format("%3d",2)�� zero�� �����ϸ� 002�� �ȴ�.
	struct zero : public Operator
	{
		zero(const Operator& op):m_op(op){}
		virtual void calc(TString& c) const;// c.replace(' ','0');
		const Operator& m_op;
	};

	struct format : public Operator
	{
		format(const char* str,...);
		virtual void calc(TString& c) const;
		TString m_str;
	};

	/// format�� ������ ' '�� '0'�� �ٲ۴�. ex) format0("%3d",2)�� 002�� �ȴ�.
	struct format0 : public Operator
	{
		format0(const char* str, ...);
		virtual void calc(TString& c) const;
		TString m_str;
	};

	struct filename : public Operator
	{
		filename (){}
		virtual void calc(TString& c) const;
	};
}

namespace sz1
{
	TString format(const char* pszFormat, ...);
	TString format0(const char* pszFormat, ...);
	// extract filename: eg) fn="c:\a\b.bmp" -> return "b", where dir="c:\a\", ext=".bmp"
	TString filename(TString const& fn, TString& dir);
	
	// extract extension: eg) fn="c:\a\b.bmp" -> return "bmp"
	TString extension(TString const& fn);
}

