#pragma once
#include "afxwin.h"
#include "trefarray.h"
class TDialog :
	public CDialog
{
public:
	
	// show�ϱ��� �ݵ�� create���־�� �Ѵ�.
	TDialog(UINT nIDTemplate, CWnd* pParent, int nIDDlgItem); //!< nIDDlgItem�� IDC_BUTTON_DRAW_MODLG ó�� button�� id�� �ָ� �� �ڸ��� dialog�� �����. ���� �������� �ʴ´�.
	virtual ~TDialog(void);

	UINT m_nIDD;

	virtual void create();

	virtual bool show();
	virtual bool hide();

	
	/**
	 * �ڽ��� Ư�� item��ġ�� childDlg Create�� ����.
	 * �ڽ��� create�� �� ȣ���Ҽ� �ִ�. ���� ���� ����� create�� ����ؼ� __super::create����� addchild�ϴ°��̴�.
	 * \param pChild new�� �� TDialog instance
	 * \param nIDDlgItem create�� ��ġ�� ��Ÿ���� iditem
	 * \return nChildIndex (embedDlg�Ҷ� ����)
	 */
	int addChild(TDialog* pChild);	
	void embedDlg(int nChildIndex);
	int numChildDlg()					{ return m_aChildren.size();}
	TDialog& childDlg(int nChildIndex)	{ return m_aChildren[nChildIndex];}
	
protected:	

	CWnd* m_pParent;
	int m_nIDdlgItem;
	bool m_bCheckShow;
	TRefArray<TDialog> m_aChildren;
};
