// Node.h: interface for the Node class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
#define AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "my.h"
#include "../utility/TFile.h"
#include "../utility/TextFile.h"
#include <list>
/* TreeNode
*/
class Node  
{
public:
	Node();
	virtual ~Node();
	
	virtual void pack(BinaryFile& file, int nVersion) const;
	virtual void unpack(BinaryFile& file, int nVersion) ;
	void AddChild(Node* pChild);	

	void detachAllChildren(std::list<Node*>& children);
	void addChildren(std::list<Node*>& children);

	void SetNameId(const char* name);
	const char* GetNameId() const	{return NameId;}
	int CountChildren();

	void printHierarchy(int depth=0);

	char* NameId;	//!< Node�� �̸��� ���� ��� ���� 
	NodeType_T NodeType;	//!< TransformNode���� IndexedFaceSetNode���� ���� �����Ѵ�.
	Node *m_pChildHead;	//!< �ڽĳ�� list�� head, singly linked list�̴�. 
	Node *m_pChildTail; //!< �ڽĳ�� list�� tail
	Node *m_pSibling;	//!< �ڽĳ��鳢�� �����ϴ� pointer, ������ sibling�� ���Ѵ�. 
	Node *m_pParent;
	int m_nIndex;
	int GetIndex() const	{return m_nIndex;}
};

#endif // !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
