#pragma once

template <class T>
/**
* �⺻���� ������ TArray�� std::vector��� �����ϴ�. ���� ū �������� ��̻���� maxSize�̻����� �����Ұ�� �պκ��� �������ٴ°�.
*/
class TOnlineArray
{
	int m_nSize;
	int m_maxCapacity;
	std::vector<T> m_vector;
public:
	TOnlineArray(int n=0, int maxCapacity=100);
	virtual ~TOnlineArray(void){}

	void resize(int n);
	int size() const						{ return m_nSize;}
	T const& data(int nIndex) const			{ return m_vector[nIndex%m_maxCapacity]; }
	T const& operator[](int nIndex) const	{ return m_vector[nIndex%m_maxCapacity]; }
	T & data(int nIndex)					{ return m_vector[nIndex%m_maxCapacity]; }
	T & operator[](int nIndex)				{ return m_vector[nIndex%m_maxCapacity]; }
	int maxCapacity() const					{return m_maxCapacity;}

	void pushBack(T const& data);
};

template <class T>
TOnlineArray<T>::TOnlineArray(int n, int maxCapacity)
{
	m_nSize=0;
	m_maxCapacity=maxCapacity;
	resize(n);	
}

template <class T>
void TOnlineArray<T>::resize(int n)
{
	m_nSize=n;
	m_vector.resize(MIN(n, m_maxCapacity));
}

template <class T>
void TOnlineArray<T>::pushBack(T const& d)
{
	resize(size()+1);
	data(size()-1)=d;
}