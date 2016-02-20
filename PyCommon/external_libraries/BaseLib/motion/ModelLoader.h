#pragma once

#include "Node.h"
#include "NodeStack.h"
#include <list>
class PLDPrim;

//! Abstract class for model hierarchy, animation loading
class ModelLoader  : noncopyable
{
public:
	ModelLoader();
	virtual ~ModelLoader();

	//! model hierarchy tree�� root
	Node* m_pTreeRoot;

	inline int GetNumTreeNode() const				{ return m_nNumTreeNode;};
	int GetIndex(Node* target) const;
	int GetIndex(const char *szNameID) const;
	char* GetName(int index)						{ return m_apNode[index]->NameId;};

	//! �θ����� ù��° child node�� return�Ѵ�. ������ NULL return
	inline Node* GetFirstChild(Node* pParent) const		{ return pParent->m_pChildHead; };
	//! ���� child node�� �Է¹޾� ���� child node�� return�Ѵ�.
	inline Node* GetNextSibling(Node* pPrevChild) const	{ return pPrevChild->m_pSibling; };
	//! Traverse������ nIndex��°�� �ش��ϴ� node�� return �Ѵ�.
	inline Node* GetNode(int nIndex) const				{ return m_apNode[nIndex]; };

	void ExportNames(const char* filename);

	// tree �� traversal�ϸ鼭, index�� parent�� �����Ѵ�.
	void UpdateIndexes();

	static NodeStack m_TreeStack; //!< multi-threading�� ���� , �޸� ������ ���ؼ� static���� ����
	

protected:
	int CountTreeNode();
	//! ��� Node pointer���� tree inorder traversal ������ ����ȴ�.
	/*!
	�����ڿ��� �ʱ�ȭ �ǵ��� �����Ѵ�.
	���Ͽ� ������ ����Ǿ� �ִ� �����̴�. �ε��Ҷ� �տ������� ä������.
	�ε��� ������ �ʿ� ������, VertexBuffer�� ���鶧 tree traversal�� �����ϱ� ���ؼ�
	���ܳ��Ҵ�. ���� modeling tool�� ���鶧�� ����� �� �ִ�.*/
	std::vector<Node*> m_apNode;

	int m_nNumTreeNode;		//!< m_nNumTreeNode==m_apNode.size();
};

