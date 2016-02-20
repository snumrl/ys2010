#pragma once

#include "stdtemplate.h"
#pragma warning (disable: 4786)

template <class T>
struct TNode
{
	TNode(TNode<T>* p, T* c, TNode<T>* n):m_pPrev(p), m_pCurr(c), m_pNext(n){}
	void release()	{ if(m_pPrev) delete m_pPrev; }
	TNode<T>* m_pPrev;
	TNode<T>* m_pNext;
	T* m_pCurr;
};

// deprecated. std::list<boost::shared_ptr<...>> ������ ����� ��.
template <class T>
/**
 * �⺻���� ������ CArray�� std::vector��� �����ϴ�. ���� ū �������� Factory�� ���� ������ �ʱ�ȭ�� �ٸ��� �Ҽ� �ִٴ� ���̴�. 
 * Ư�� TDefaultFactory�� ����Ѱ��(bReference=false�� ���) ��� ������ �����ڰ� call �ȴ�. �̴� ��� ���Ұ� factory�� ���� �����Ǳ� ������ �����ϴ�.
 * ������ ������ *�� array��, std::vector�� �ٸ���. ���� bReference==false �̸�, new�� call���� �ʰ�, ���Ұ� NULL�� �ʱ�ȭ�ȴ�.
 */
class TList  
{
public:

	struct iterator
	{
		iterator():m_pNode(NULL){}
		iterator(TNode<T>* p):m_pNode(p) {}
		TNode<T> *m_pNode;
		void operator++( )		{ m_pNode=m_pNode->m_pNext;}
		void operator--( )		{ m_pNode=m_pNode->m_pPrev;}
		bool operator==(const iterator& other) { return m_pNode==other.m_pNode; }
		bool operator!=(const iterator& other) { return m_pNode!=other.m_pNode; }
		void operator=(const iterator& other)	{ m_pNode=other.m_pNode;}
		T& operator*()							{ return *(m_pNode->m_pCurr);}
		T& data()								{ return *(m_pNode->m_pCurr);}
		T* dataPtr()							{ return m_pNode->m_pCurr;}

		iterator next()							{ return iterator(m_pNode->m_pNext);}
		iterator prev()							{ return iterator(m_pNode->m_pPrev);}
	};
	TList(bool bReference=false);	//!< use TDefaultFactory if bReference=false, else use TFactory
	TList(TFactory<T>* pF);
	virtual ~TList();

	void insert(iterator position, T* pElt);
	void remove(iterator position);	// ���Ҹ� release�Ѵ�.
	void erase(iterator position)					{ remove(position);}
	T*	pop(iterator position);		// ������ ptr�� return�ϹǷ�, release���� �ʴ´�.	
	T	popValue(iterator position);// ���Ҹ� release�Ѵ�. 
	void pushFront(const T& elt)					{ insert(begin(), m_pFactory->copy(elt)); }
	void pushFront(T* pElt)							{ insert(begin(), pElt); }
	void pushBack(const T& elt)						{ insert(end(), m_pFactory->copy(elt));}
	void pushBack(T* pElt)							{ insert(end(), pElt); }	
	void pushBack()									{ insert(end(), m_pFactory->create(0));}	
	T*	popBackPtr()								{ return pop(end());}	// release�ؾ��Կ� ����.
	T	popBack()									{ return popValue(end());}

	iterator find(const T& v)						{ iterator i;for(i=begin(); i!=end(); ++i) if(data(i)==v) return i; return i;}

	bool member(const T& v)							{ return find(v)!=end();}

	void init();
	void release()	{ init();}
	int size() const								{ return m_nSize;}
	T& operator[](iterator i) const					{ return *(i.m_pNode->m_pCurr);}
	// i��° ����- ����.
	T& operator[](int i) const;
	T& data(iterator i) const						{ return *(i.m_pNode->m_pCurr);}
	T* ptr(iterator i) const						{ return (i.m_pNode->m_pCurr);}
	
	void changeFactory(TFactory<T>* pF)		{ASSERT(m_nSize==0); init(); delete m_pFactory; m_pFactory=pF; }

	// !empty()�ϰ��
	T& front() const								{ return data(begin());}
	T& back() const									{ return data(iterator(m_pDummyTail->m_pPrev));}

	// for(i=list.begin(); i!=list.end(); i++)...
	iterator begin() const							{ return iterator(m_pHead);}
	iterator end() const							{ return iterator(m_pDummyTail);}
	bool empty() const								{ return m_pHead==m_pDummyTail; }
protected:
	TFactory<T>* m_pFactory;
	int m_nSize;
	TNode<T> *m_pHead;	//!< �ڽĳ�� list�� head, singly linked list�̴�. 
	TNode<T> *m_pDummyTail; //!< �ڽĳ�� list�� tail
};

//#include "stdafx.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T>
TList<T>::TList(bool bReference)
{
	if(bReference)
		m_pFactory=new TFactory<T>();
	else
		m_pFactory=new TDefaultFactory<T>();

	m_pHead=m_pDummyTail=new TNode<T>(NULL, NULL, NULL);
	m_nSize=0;
}

template <class T>
TList<T>::TList(TFactory<T>* pF)
{
	m_pFactory=pF;

	m_pHead=m_pDummyTail=new TNode<T>(NULL, NULL, NULL);
	m_nSize=0;
}

template <class T>
TList<T>::~TList()
{
	init();
	delete m_pFactory;
	delete m_pDummyTail;
}

template <class T>
void TList<T>::init()
{
	for(iterator i=begin(); i!=end(); ++i)
		m_pFactory->release(ptr(i));

	if(!empty())
	{
		end().m_pNode->release();
		end().m_pNode->m_pPrev=NULL;
		m_pHead=end().m_pNode;
	}
}

template <class T>
T& TList<T>::operator[](int i) const
{
	int c=0;
	iterator il;
	for(il=begin(); il!=end(); ++il)
	{
		if(c==i) break;	
		c++;
	}
	return *il;
}

template <class T>
void TList<T>::insert(iterator position, T* pElt)
{
	if(empty())
	{
		m_pHead=new TNode<T>(NULL,pElt,m_pDummyTail);
		m_pDummyTail->m_pPrev=m_pHead;
	}
	else if(position==begin())
	{
		TNode<T>* next=m_pHead;
		m_pHead=new TNode<T>(NULL, pElt, next);
		next->m_pPrev=m_pHead;
	}
	else
	{
		TNode<T>* prev, *next;
		next=position.m_pNode;
		prev=position.m_pNode->m_pPrev;
		position.m_pNode=new TNode<T>(prev, pElt, next);
		prev->m_pNext=position.m_pNode;
		next->m_pPrev=position.m_pNode;
	}

	m_nSize++;
}

template <class T>
T*	TList<T>::pop(iterator position)		// ������ ptr�� return�ϹǷ�, release���� �ʴ´�.
{
	if(empty())
		return NULL;

	if(position==end())
	{
		position--;
		return pop(position);
	}
	if(position==begin())
	{
		m_pHead=position.m_pNode->m_pNext;
		m_pHead->m_pPrev=NULL;
	}
	else
	{
		TNode<T>* prev, *next;
		next=position.m_pNode->m_pNext;
		prev=position.m_pNode->m_pPrev;
		prev->m_pNext=next;
		next->m_pPrev=prev;
	}

	T* out=position.m_pNode->m_pCurr;
	delete position.m_pNode;

	m_nSize--;
	return out;
}

template <class T>
void TList<T>::remove(iterator position)
{
	ASSERT(!empty());
	m_pFactory->release(pop(position));
}

template <class T>
T TList<T>::popValue(iterator position)
{
	T temp; T* ptr; ptr=pop(position); temp=*ptr; m_pFactory->release(ptr); return temp;
}