// Node.cpp: implementation of the Node class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Node.h"

//#include "headers.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Node::Node()
{
	m_pChildHead=NULL;
	m_pChildTail=NULL;
	m_pSibling=NULL;
	m_pParent=NULL;
	m_nIndex=-1;
	NodeType=GENERAL;
	NameId=NULL;
}

Node::~Node()
{
	if(m_pChildHead)
		delete m_pChildHead;
	if(m_pSibling)
		delete m_pSibling;
	if(NameId) delete[] NameId;
}

void Node::pack(BinaryFile& file, int nVersion) const
{
	file.packInt(NodeType);
	file.pack(NameId);
}

void Node::unpack(BinaryFile& file, int nVersion) 
{
	// NodeType�� unpack�ϴ� ������ ������ ����� �����Ұ� (�ۿ��� NodeType�� unpack�ؾ� type�� ���� new�� �ؾ��ϱ� ����)
	TString name;
	file.unpack(name);

	if(name.length())
	{
		NameId=new char[name.length()+1];
		strcpy(NameId, name.ptr());
	}
}

void Node::AddChild(Node* pChild)
{
	if(!m_pChildHead)
	{
		m_pChildHead=pChild;
		m_pChildTail=pChild;
	}
	else
	{
		m_pChildTail->m_pSibling=pChild;
		m_pChildTail=pChild;
	}

	pChild->m_pParent=this;
}

void Node::SetNameId(const char* name)
{
	if(NameId) delete[] NameId;
	NameId=new char[strlen(name)+1];
	strcpy(NameId, name);
}

int Node::CountChildren()
{
	int count=0;
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		count++;
	return count;
}

void Node::detachAllChildren(std::list<Node*>& children)
{
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		children.push_back(i);

	m_pChildHead=NULL;
	m_pChildTail=NULL;
}

void Node::addChildren(std::list<Node*>& children)
{
	std::list<Node*>::iterator i;
	for(i=children.begin(); i!=children.end(); i++)
	{
		AddChild(*i);
	}
}

void Node::printHierarchy(int depth)
{
	for(int ii=0; ii<depth; ii++) printf(" ");
	printf("%s \n", NameId);
	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		i->printHierarchy(depth+1);
}
