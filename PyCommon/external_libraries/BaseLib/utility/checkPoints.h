#pragma once
#include "Tfile.h"
/// release�� debug ��� ���̿� ����� �ٸ��ų� �ý��۸��� ����� �ٸ��ų�, ������ �ɼǿ� ���� ����� �޶����� ����
/// ������� ����� ��������, checkpoint���� ����� ����� ���� ����� ���Ͽ� ������ ���� ������ ã�Ƴ��� ���� Ŭ����.

class checkPoints
{
public:
	/// bSave��true�� ��� ���Ͽ� �����ϰ�, false�� ��� file�� ����� ���� ���Ѵ�.
	checkPoints(const char* filename, bool bSave, m_real checkThr=0.0, bool throwError=true);
    virtual ~checkPoints(void);

	void check(const vectorn& vec);
	void check(const matrixn& vec);
	void check(const int& n);
	void check(const m_real& n);
private:
	void increment();
	bool isSimilar(const vectorn& vec, const vectorn& vec2);
	BinaryFile mFile;
	bool mbSave;	
	bool mbTrace;
	bool mbThrow;
	int mnCurrCheckPoint;
	const m_real mCheckThreshold;
	
};

#include <windows.h>
typedef __int64 INT64;

class Profiler
{
	int m_nCount;
	INT64 m_int64StartTime;
	INT64 m_int64MaxTime;
	INT64 m_int64AverageTime;
	INT64 m_int64Diff;
	static UINT64 m_ValuePerMilliSecond;
public:
	Profiler();
	~Profiler(){}

	void start();
	void stop();
	void draw();

};


class Stopwatch
{
private:
	int mState;
	enum {END, START, PAUSE};
public:
	Stopwatch(void) {
		QueryPerformanceFrequency((LARGE_INTEGER*)&m_freq);	// Ŭ������
		mState=END;
	};

public:
	~Stopwatch(void) {
	};

	// start()-> stop() ->start()->stop()->end() �̼����� call�Ǵ� ���, 
	// start�� pause()������ �ð��� �����Ǽ� ��µȴ�.

	// start the timer
	void start();
	
	inline void stop()
	{
		Msg::verify(mState==START, "already stopped %d", mState);
		mState=PAUSE;
		QueryPerformanceCounter((LARGE_INTEGER*)&m_temp);
		m_pause.QuadPart=m_temp.QuadPart-m_start.QuadPart;	// m_pause�� �������� ������ �ð�.
	};

	// end the timer and return the end time
	m_real end() ;

	inline m_real end(const char* str) 
	{
		m_real time=end();
		printf("%s : %f (%g)\n", str, time, time);
		return time;
	}

	inline m_real endOutput(const char* str)
	{
		m_real time=end();
		Msg::output(str, "%f (%g)", time, time);
		return time;
	}

private:	
	LARGE_INTEGER m_freq, m_start, m_end, m_pause, m_temp;		// 64-bit integer type
	m_real m_time;
};


class Tracer
{
	TString mFilename;
public:
	Tracer(const char* filename);
	virtual ~Tracer();
	void trace(const char* pszFormat, ...);
};