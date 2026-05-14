
// PhotogrammetryAppDlg.h: 头文件
//
#include <Eigen/Dense>
#pragma once


// CPhotogrammetryAppDlg 对话框
class CPhotogrammetryAppDlg : public CDialogEx
{
    DECLARE_DYNAMIC(CPhotogrammetryAppDlg)
// 构造
public:
	CPhotogrammetryAppDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_PHOTOGRAMMETRYAPP_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();

	afx_msg void OnBnClickedBtnAddPt();     // 添加点按钮
	afx_msg void OnBnClickedBtnResection(); // 计算按钮
	afx_msg void OnBnClickedBtnDelPt();     // 删除点按钮
	afx_msg void OnLvnItemchangedListPoints(NMHDR* pNMHDR, LRESULT* pResult); // 列表改变
  // 前方交会计算按钮
	afx_msg void OnBnClickedBtnForwardCompute();
  // 额外按钮：载入/清除数据
	afx_msg void OnBnClickedBtnLoad();
	afx_msg void OnBnClickedBtnClear();
	// 滚动消息响应函数（在实现文件中已定义）
	afx_msg void OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);
	afx_msg void OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar);

	DECLARE_MESSAGE_MAP()
public:
	// 相机主距
	double m_f;
	// 像主点 x
	double m_x0;
	// 像主点 y
	double m_y0;
	// 摄站 X
	double m_resXs;
	// 摄站Y
	double m_resYs;
	// 摄站Z
	double m_resZs;
	// 航向倾角
	double m_resPhi;
	double m_resOmega;
	double m_resKappa;
	CListCtrl m_listPoints;
	// x
	double m_curX;
	// y
	double m_curY;
	// X
	double m_curGndX;
	// Y
	double m_curGndY;
	// Z
	double m_curGndZ;
	// 比例尺分母
	double m_scaleM;
	// m_sigma0
	double m_sigma0;
	// m_mXs
	double m_mXs;
	// m_mYs
	double m_mYs;
	// m_mZs
	double m_mZs;
	// m_mPhi
	double m_mPhi;
	// m_mOmg
	double m_mOmg;
	// m_mKap
	double m_mKap;
	CListCtrl m_listQMat;

	int m_nVScrollPos;
	int m_nVScrollMax;
	int m_nHScrollPos;
	int m_nHScrollMax;

	void UpdateQMatrixDisplay(const Eigen::MatrixXd& Q);
};
