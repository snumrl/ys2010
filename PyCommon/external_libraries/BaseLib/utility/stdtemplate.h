#pragma once
#include <stdio.h>
//#include <tchar.h>
#include <vector>

template <class T>
bool vector_remove(std::vector<T>& vec, const T& elt)
{
	for(int i=0; i<vec.size(); i++)
	{
		if(vec[i]==elt)
		{
			for(int j=i; i<vec.size()-1; i++)
				std::swap(vec[i],vec[i+1]);

			vec.resize(vec.size()-1);
			return true;
		}
	}
	return false;
}








//////////////////////
// �Ʒ��� deprecated.
// use boost::shared_ptr..
/// reference�� �����Ѵ�. �� new�� delete�� ������� �ʰ� pointer�� ������ ���̴�.
template <class T>
class TFactory
{
public:
	
	TFactory(){ }		virtual ~TFactory(){}
	virtual T* create(int index=-1) { return NULL; }	// do nothing
	virtual T* copy(const T& elt)	{ return NULL;}	// do nothing
	virtual void release(T* pElt)	{ }	// do nothing
	virtual TFactory<T>* clone() const	{ return new TFactory<T>;}
};

// factory �� ��Ÿ� ���°��� ���� �޸� ���� ����� �޶�����. �޸� ������ ������ å������ �ٶ�.
// factory�� list, array��� ����.


/// ������ �����ϰ� ������ �����.
template <class T>
class TDefaultFactory : public TFactory<T>
{
public:
	TDefaultFactory(){}		virtual ~TDefaultFactory(){}
	virtual T* create(int index=-1) { return new T(); }
	virtual T* copy(const T& elt)	{ T* ptr=create(); *ptr=elt; return ptr;}	// copy value
	virtual void release(T* pElt)	{ ASSERT(pElt); delete pElt;}
	virtual TFactory<T>* clone() const	{ return new TDefaultFactory<T>;}
};

template <class T, class T2>
class TDefaultFactoryDerived : public TDefaultFactory<T>
{
public:
	TDefaultFactoryDerived (){}
	virtual ~TDefaultFactoryDerived(){}

	virtual T* create(int index=-1) { return (T*)new T2(); }
	virtual TFactory<T>* clone() const	{ return new TDefaultFactoryDerived<T,T2>;}
};

/// ������ ������ ���� ������, ������ �����.
template <class T>
class TReleaseFactory : public TFactory<T>
{
public:
	TReleaseFactory(){}		virtual ~TReleaseFactory(){}
	virtual T* create(int index=-1) { return NULL; }	// do nothing
	virtual T* copy(const T& elt)	{ ASSERT(0); return NULL;} // do nothing
	virtual void release(T* pElt)	{ if(pElt) delete pElt; }	// release
	virtual TFactory<T>* clone() const	{ return new TReleaseFactory<T>;}
};

/// ������ ������, ������ �ʴ´�.
template <class T>
class TCreateFactory : public TFactory<T>
{
public:
	TCreateFactory(){}		virtual ~TCreateFactory(){}
	virtual T* create(int index=-1)		{ return new T(); }	// create
	virtual T* copy(const T& elt)		{ return &elt;}		// copy reference
	virtual void release(T* pElt)		{ } // do nothing
	virtual TFactory<T>* clone() const	{ return new TCreateFactory<T>;}
};

