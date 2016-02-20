
// util.h: interface for the util class.
//
//////////////////////////////////////////////////////////////////////

#pragma once

namespace Msg
{
	void verify(bool bExpression, const char*, ...);// release������ �������� �ʴ� verify�� �ϰ� ������� ���.
	void print(const char*,...);
	void print2(const char*,...);
	void error(const char*,...);
	void msgBox(const char*,...);
	bool confirm(const char*,...);
	void flush();
	void output(const char* key, const char* formatString, ...);
	void outputState(bool bOutput);
	
	class Base
	{
	public:
		Base(){m_bOutput=true;}
		virtual ~Base(){}
		// inherit and implement followings
		virtual void print(const char* msg);
		virtual void print2(const char* msg);
		virtual void flush();
		virtual void error(const char* msg);
		virtual void msgBox(const char* msg);
		virtual void output(const char* key, const char* msg);
		// ���� �����ȵ�. ����ڿ��� yes or no����� ���.
		virtual bool confirm(const char* msg);
		bool m_bOutput;
		
	};
		
	extern Base* g_pMsgUtil;
}


int fast_strcmp(const char *a, const char *b);
int fast_strcmp_upper(const char *a, const char *b);

//!���� �Լ� , �Ƹ� ���� ��������.
int calcMaxPrefix(char* a, char*b);
int calcMaxSubstring(char *a, char*b);	//!< a������ b�� �����̸鼭 ���� ���� ��ĥ�� ��ġ�� ���ڼ� return

void FindAndSubstitute(char *source, char *pattern, char *output);
char * GetToken(FILE *file);		//!< #���� �����ϴ� ���� �ּ����� ����Ѵ�. token�� space�Ǵ� ,�� ���еȴ�.
char * GetTokenCLang(FILE *file);	//!< //�� /* */�� �ּ����� ����Ѵ�. token�� C�� ������ �������� �и��Ǿ� ���´�. ������ 4000�ڸ� ���� �ʾƾ� �Ѵ�.

bool IsFileExist(const char* filename);
bool IsFileWritable(const TCHAR *szFileName);
//!���� Token�� NULL�� �ɶ����� GetToken�� ���� �ʰ� �ٸ� ������ GetToken�ؾ� �ϴ� ��� ������ FileCloseForGetToken�� call���־�� �Ѵ�.
void FileCloseForGetToken();
void FileCloseForGetTokenCLang();

void ParseCommandLineString(const char* input,int& argc, char**& argv);
void FreeCommandLineString(int argc, char** argv);

bool ConfirmWritable(const char* strFilePath);
char* CopyStr(const char* str);

template <class T> T** AllocMatrix(int height, int width)	//!< �࿭
{
	T **aaType;
	aaType=new T*[height];
	for(int i=0; i< height; i++)
		aaType[i]=new T[width];
	return aaType;
}

template <class T> void FreeMatrix(int height, T** array)
{
	for( int i=0; i<height; i++)
		delete[] array[i];	// delete columns
	delete[] array;
}

template <class T> void SaveMatrix(T **array, int height, int width, const char* filename)
{
	TFile file;
	file.OpenWriteFile(filename);
	for(int i=0; i<height; i++)
	{
		file.PackArray((void*)(array[i]), width, sizeof(T));
	}
	file.CloseFile();
}

template <class T> void LoadMatrix(T **array, int height, int width, const char* filename)
{
	TFile file;
	file.OpenReadFile(filename);
	for(int i=0; i<height; i++)
	{
		file.UnpackArray((void*)(array[i]), width, sizeof(T));
	}
	file.CloseFile();
}

void GetSimpleAccurateTime(LARGE_INTEGER &iCounter);

#ifdef DIRECT3D_VERSION
inline float DWToFloat(DWORD dw)	{ return *((float*)(&dw));};
inline DWORD FloatToDW(float f)		{ return *((DWORD*)(&f));};
void GetRotationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource);
void GetTranslationMatrix(D3DXMATRIX *pTarget, D3DXMATRIX* pSource);
void GetAxisRotationMatrix(D3DXMATRIX& matRot, const D3DXVECTOR3& vecAxis, const D3DXVECTOR3& front, const D3DXVECTOR3& vecTarget);
#endif

template <class T> T* ToPtr(int i)		{ return *((T**)(&i));};
template <class T> int ToInt(T* ptr)	{ return *((int*)(&ptr));};


//! MOutputToFile("a.txt", ("(%d,%d)",a,b) );	�̷� ������ ����ϼ���.
#define MOutputToFile(filename, arg) { TString str;str.format arg ; OutputToFile(filename, (const char *)str); }

void OutputToFile(const char* filename, const char* string);

