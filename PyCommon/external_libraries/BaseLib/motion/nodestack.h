// NodeStack.h: interface for the NodeStack class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_)
#define AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define NODE_STACK_SIZE 100
class AlzzaTransform;
class AlzzaCTransform;
class Node;

class NodeStackElt
{
	Node* src;
	AlzzaTransform *dst;
	friend class NodeStack;
};
//! PLDLoader�� ���� model hierarchy�� traversal�Ҷ� ���ȴ�.
class NodeStack  
{
public:
	NodeStack();
	virtual ~NodeStack();
	void Initiate();

	inline void Push(Node* src) { AlzzaTransform*dst=NULL; Push(src,dst); };
	inline void Pop(Node** psrc) { AlzzaTransform* dst; Pop(psrc,&dst); };

	Node* GetTop()		{ if(top<0) return NULL; return(stack[top].src);};

	//! AlzzaTree skinning version�� ���� traverse�� ��� ���
	void Push(Node* src, AlzzaCTransform *dst);
	//! AlzzaTree skinning version�� ���� traverse�� ��� ��� 
	void Pop(Node** psrc, AlzzaCTransform** pdst);

	//! AlzzaTree�� ���� traverse�� ��� ���
	void Push(Node* src, AlzzaTransform *dst);
	//! AlzzaTree�� ���� traverse�� ��� ��� 
	void Pop(Node** psrc, AlzzaTransform** pdst);
	void GetTopNth(Node** psrc, AlzzaTransform **pdst, int n);	//!< n==0�϶� top return
private:
	NodeStackElt stack[NODE_STACK_SIZE];
	int top;
};

#endif // !defined(AFX_NodeStack_H__4902AE40_9DDE_11D4_A268_0020AF755DA9__INCLUDED_)
