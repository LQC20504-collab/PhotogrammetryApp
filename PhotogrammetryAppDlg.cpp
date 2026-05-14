
// PhotogrammetryAppDlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "PhotogrammetryApp.h"
#include "PhotogrammetryAppDlg.h"
#include "afxdialogex.h"
#include "Resection.h"
#include "Forward.h"
#include <gdiplus.h>
#include <fstream>
#include <sstream>
#include <string>
#include <istream>
using namespace Gdiplus;

IMPLEMENT_DYNAMIC(CPhotogrammetryAppDlg, CDialogEx)

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CPhotogrammetryAppDlg::OnBnClickedBtnLoad()
{
	CFileDialog dlg(TRUE, _T("csv"), NULL, OFN_FILEMUSTEXIST | OFN_HIDEREADONLY,
		_T("CSV Files (*.csv;*.txt)|*.csv;*.txt|All Files (*.*)|*.*||"));
	if (dlg.DoModal() != IDOK) return;
	CString path = dlg.GetPathName();
    CStdioFile file;
	if (!file.Open(path, CFile::modeRead | CFile::shareDenyNone | CFile::typeText)) {
		AfxMessageBox(_T("无法打开文件"));
		return;
	}

	m_listPoints.DeleteAllItems();
	CString line;
	int row = 0;
	while (file.ReadString(line)) {
		if (line.IsEmpty()) continue;
		// replace commas with spaces for simple parsing
		for (int i = 0; i < line.GetLength(); ++i) if (line[i] == ',') line.SetAt(i, ' ');
        CT2A conv(line);
		std::string sline = std::string((LPCSTR)conv);
		std::stringstream ss(sline);
		double x, y, X, Y, Z;
		if (!(ss >> x >> y >> X >> Y >> Z)) continue;

		CString str;
		str.Format(_T("%d"), ++row);
		int idx = m_listPoints.InsertItem(row - 1, str);
		str.Format(_T("%.4f"), x); m_listPoints.SetItemText(idx, 1, str);
		str.Format(_T("%.4f"), y); m_listPoints.SetItemText(idx, 2, str);
		str.Format(_T("%.3f"), X); m_listPoints.SetItemText(idx, 3, str);
		str.Format(_T("%.3f"), Y); m_listPoints.SetItemText(idx, 4, str);
		str.Format(_T("%.3f"), Z); m_listPoints.SetItemText(idx, 5, str);
	}
	AfxMessageBox(_T("载入完成"));
}

void CPhotogrammetryAppDlg::OnBnClickedBtnClear()
{
	m_listPoints.DeleteAllItems();
	// reset some params
	m_resXs = m_resYs = m_resZs = 0.0;
	m_resPhi = m_resOmega = m_resKappa = 0.0;
	m_sigma0 = 0.0;
	m_mXs = m_mYs = m_mZs = m_mPhi = m_mOmg = m_mKap = 0.0;
	UpdateData(FALSE);
}

void CPhotogrammetryAppDlg::OnBnClickedBtnForwardCompute()
{
	// Read inputs from dialog controls (assume controls exist with these IDs)
	CString s;
	// helper lambda to read and parse
	auto readDouble = [&](int id, double &out)->bool {
		GetDlgItemText(id, s);
		if (s.IsEmpty()) return false;
		out = _ttof(s);
		return true;
	};

	double Xs1, Ys1, Zs1, phi1, omega1, kappa1;
	double Xs2, Ys2, Zs2, phi2, omega2, kappa2;
	double f, x0, y0, x1, y1, x2, y2;

	// read required fields, show error if missing
	if (!readDouble(IDC_EDIT_XS1, Xs1) || !readDouble(IDC_EDIT_YS1, Ys1) || !readDouble(IDC_EDIT_ZS1, Zs1) ||
		!readDouble(IDC_EDIT_PHI1, phi1) || !readDouble(IDC_EDIT_OMEGA1, omega1) || !readDouble(IDC_EDIT_KAPPA1, kappa1) ||
		!readDouble(IDC_EDIT_x1, x1) || !readDouble(IDC_EDIT_y1, y1) ||
		!readDouble(IDC_EDIT_XS2, Xs2) || !readDouble(IDC_EDIT_YS2, Ys2) || !readDouble(IDC_EDIT_ZS2, Zs2) ||
		!readDouble(IDC_EDIT_PHI2, phi2) || !readDouble(IDC_EDIT_OMEGA2, omega2) || !readDouble(IDC_EDIT_KAPPA2, kappa2) ||
		!readDouble(IDC_EDIT_x2, x2) || !readDouble(IDC_EDIT_y2, y2) ||
		!readDouble(IDC_EDIT_F, f) || !readDouble(IDC_EDIT_X0, x0) || !readDouble(IDC_EDIT_Y0, y0))
	{
		AfxMessageBox(_T("请输入所有前方交会所需的输入数值（摄站、角、像点、内方位）。"));
		return;
	}

	// Call forwardIntersection
	double u1,v1,w1,u2,v2,w2,N1,N2, XA,YA,ZA, residual;
	bool ok = forwardIntersection(
		Xs1, Ys1, Zs1, phi1, omega1, kappa1,
		Xs2, Ys2, Zs2, phi2, omega2, kappa2,
		f, x0, y0,
		x1, y1, x2, y2,
		u1, v1, w1,
		u2, v2, w2,
		N1, N2,
		XA, YA, ZA,
		residual);

	if (!ok) {
		AfxMessageBox(_T("前方交会失败：可能为退化配置或数值不稳定（如视线近似平行）。"));
		return;
	}

	// Display results into dialog controls (formatting)
	CString out;
	out.Format(_T("%.6f"), u1); SetDlgItemText(IDC_EDIT_U1, out);
	out.Format(_T("%.6f"), v1); SetDlgItemText(IDC_EDIT_V1, out);
	out.Format(_T("%.6f"), w1); SetDlgItemText(IDC_EDIT_W1, out);

	out.Format(_T("%.6f"), u2); SetDlgItemText(IDC_EDIT_U2, out);
	out.Format(_T("%.6f"), v2); SetDlgItemText(IDC_EDIT_V2, out);
	out.Format(_T("%.6f"), w2); SetDlgItemText(IDC_EDIT_W2, out);

	out.Format(_T("%.6f"), N1); SetDlgItemText(IDC_EDIT_N1, out);
	out.Format(_T("%.6f"), N2); SetDlgItemText(IDC_EDIT_N2, out);

	out.Format(_T("%.6f"), XA); SetDlgItemText(IDC_EDIT_XA, out);
	out.Format(_T("%.6f"), YA); SetDlgItemText(IDC_EDIT_YA, out);
	out.Format(_T("%.6f"), ZA); SetDlgItemText(IDC_EDIT_ZA, out);

	out.Format(_T("%.6f"), residual); SetDlgItemText(IDC_EDIT_RESIDUAL, out);

	AfxMessageBox(_T("前方交会完成，结果已填入界面。"));
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPhotogrammetryAppDlg 对话框



CPhotogrammetryAppDlg::CPhotogrammetryAppDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_PHOTOGRAMMETRYAPP_DIALOG, pParent)
	, m_f(153.24)
	, m_x0(0.0)
	, m_y0(0.0)
	, m_resXs(0)
	, m_resYs(0)
	, m_resZs(0)
	, m_resPhi(0)
	, m_resOmega(0)
	, m_resKappa(0)
	, m_curX(0)
	, m_curY(0)
	, m_curGndX(0)
	, m_curGndY(0)
	, m_curGndZ(0)
	, m_scaleM(50000)
	, m_mXs(0)
	, m_mYs(0)
	, m_mZs(0)
	, m_mPhi(0)
	, m_mOmg(0)
	, m_mKap(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPhotogrammetryAppDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_F, m_f);
	DDX_Text(pDX, IDC_EDIT_X0, m_x0);
	DDX_Text(pDX, IDC_EDIT_Y0, m_y0);
	DDX_Text(pDX, IDC_EDIT_RES_XS, m_resXs);
	DDX_Text(pDX, IDC_EDIT_RES_YS, m_resYs);
	DDX_Text(pDX, IDC_EDIT_RES_ZS, m_resZs);
	DDX_Text(pDX, IDC_EDIT_RES_PHI, m_resPhi);
	DDX_Text(pDX, IDC_EDIT_RES_OMEGA, m_resOmega);
	DDX_Text(pDX, IDC_EDIT_RES_KAPPA, m_resKappa);
	DDX_Control(pDX, IDC_LIST_POINTS, m_listPoints);
	DDX_Text(pDX, IDC_EDIT_CUR_X, m_curX);
	DDX_Text(pDX, IDC_EDIT_CUR_Y, m_curY);
	DDX_Text(pDX, IDC_EDIT_CUR_GNDX, m_curGndX);
	DDX_Text(pDX, IDC_EDIT_CUR_GNDY, m_curGndY);
	DDX_Text(pDX, IDC_EDIT_CUR_GNDZ, m_curGndZ);
	DDX_Text(pDX, IDC_EDIT_SCALE_M, m_scaleM);
	DDX_Text(pDX, IDC_EDIT_SIGMA0, m_sigma0);
	DDX_Text(pDX, IDC_EDIT_MXS, m_mXs);
	DDX_Text(pDX, IDC_EDIT_MYS, m_mYs);
	DDX_Text(pDX, IDC_EDIT_MZS, m_mZs);
	DDX_Text(pDX, IDC_EDIT_MPHI, m_mPhi);
	DDX_Text(pDX, IDC_EDIT_MOMG, m_mOmg);
	DDX_Text(pDX, IDC_EDIT_MKAP, m_mKap);
	DDX_Control(pDX, IDC_LIST_Q_MAT, m_listQMat);
}

BEGIN_MESSAGE_MAP(CPhotogrammetryAppDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()

	ON_BN_CLICKED(IDC_BTN_ADD_PT, &CPhotogrammetryAppDlg::OnBnClickedBtnAddPt)
	ON_BN_CLICKED(IDC_BTN_RESECTION, &CPhotogrammetryAppDlg::OnBnClickedBtnResection)

	ON_BN_CLICKED(IDC_BTN_DEL_PT, &CPhotogrammetryAppDlg::OnBnClickedBtnDelPt)
   ON_BN_CLICKED(IDC_BTN_FORWARD_CALC, &CPhotogrammetryAppDlg::OnBnClickedBtnForwardCompute)
	ON_BN_CLICKED(IDC_BTN_LOAD, &CPhotogrammetryAppDlg::OnBnClickedBtnLoad)
	ON_BN_CLICKED(IDC_BTN_CLEAR, &CPhotogrammetryAppDlg::OnBnClickedBtnClear)
	// map several calculation buttons to the forward compute implementation
	ON_BN_CLICKED(IDC_BTN_CALC_AUX, &CPhotogrammetryAppDlg::OnBnClickedBtnForwardCompute)
	ON_BN_CLICKED(IDC_BTN_CALC_N, &CPhotogrammetryAppDlg::OnBnClickedBtnForwardCompute)
	ON_BN_CLICKED(IDC_BTN_CALC_XYZ, &CPhotogrammetryAppDlg::OnBnClickedBtnForwardCompute)

	ON_WM_VSCROLL()
	ON_WM_HSCROLL()

END_MESSAGE_MAP()


// CPhotogrammetryAppDlg 消息处理程序

BOOL CPhotogrammetryAppDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	ShowWindow(SW_MAXIMIZE);

	ShowWindow(SW_MINIMIZE);

	// TODO: 在此添加额外的初始化代码

	// 1. 设置列表样式：整行选中、显示网格
	m_listPoints.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);

	// 2. 插入表头 (列索引, 标题, 对齐, 宽度)
	m_listPoints.InsertColumn(0, _T("点号"), LVCFMT_CENTER, 50);
	m_listPoints.InsertColumn(1, _T("x (mm)"), LVCFMT_CENTER, 80);
	m_listPoints.InsertColumn(2, _T("y (mm)"), LVCFMT_CENTER, 80);
	m_listPoints.InsertColumn(3, _T("X (m)"), LVCFMT_CENTER, 100);
	m_listPoints.InsertColumn(4, _T("Y (m)"), LVCFMT_CENTER, 100);
	m_listPoints.InsertColumn(5, _T("Z (m)"), LVCFMT_CENTER, 100);

	m_listQMat.SetExtendedStyle(LVS_EX_FULLROWSELECT | LVS_EX_GRIDLINES);
	CString headers[] = { _T("Xs"), _T("Ys"), _T("Zs"), _T("Phi"), _T("Omega"), _T("Kappa") };
	for (int i = 0; i < 6; i++) {
		m_listQMat.InsertColumn(i, headers[i], LVCFMT_CENTER, 70);
	}

	// 从文件载入 JPG 并生成 HICON
	{
		std::wstring path = L"\\hfjh.jpg"; // 改为实际路径或从资源读取路径
		std::unique_ptr<Bitmap> bmp(Bitmap::FromFile(path.c_str()));
		if (bmp && bmp->GetLastStatus() == Ok)
		{
			HICON h = nullptr;
			if (bmp->GetHICON(&h) == Ok && h != nullptr)
			{
				// 保存并设置为对话框图标（大/小）
				if (m_hIcon) ::DestroyIcon(m_hIcon);
				m_hIcon = h;
				SetIcon(m_hIcon, TRUE);
				SetIcon(m_hIcon, FALSE);
			}
		}
	}

	// 计算内容大小（示例：内容高度 contentHeight，根据实际需要计算）
	int contentHeight = 800; // 例如整体控件所需高度
	int contentWidth = 700; // 例如整体控件所需宽度

	CRect client;
	GetClientRect(&client);
	m_nVScrollMax = max(0, contentHeight - client.Height());
	m_nHScrollMax = max(0, contentWidth - client.Width());

	m_nVScrollPos = 0;
	m_nHScrollPos = 0;

	SetScrollRange(SB_VERT, 0, m_nVScrollMax, FALSE);
	SetScrollRange(SB_HORZ, 0, m_nHScrollMax, FALSE);
	SetScrollPos(SB_VERT, m_nVScrollPos, TRUE);
	SetScrollPos(SB_HORZ, m_nHScrollPos, TRUE);

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CPhotogrammetryAppDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CPhotogrammetryAppDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CPhotogrammetryAppDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

void CPhotogrammetryAppDlg::OnLvnItemchangedListPoints(NMHDR* pNMHDR, LRESULT* pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: 在此添加控件通知处理程序代码
	*pResult = 0;
}

void CPhotogrammetryAppDlg::OnBnClickedBtnAddPt()
{
	// 1. 同步输入框数据
	if (!UpdateData(TRUE)) return;

	// 2. 插入到列表中
	int row = m_listPoints.GetItemCount();
	CString str;
	str.Format(_T("%d"), row + 1);
	m_listPoints.InsertItem(row, str); // 点号

	str.Format(_T("%.4f"), m_curX);    m_listPoints.SetItemText(row, 1, str);
	str.Format(_T("%.4f"), m_curY);    m_listPoints.SetItemText(row, 2, str);
	str.Format(_T("%.3f"), m_curGndX); m_listPoints.SetItemText(row, 3, str);
	str.Format(_T("%.3f"), m_curGndY); m_listPoints.SetItemText(row, 4, str);
	str.Format(_T("%.3f"), m_curGndZ); m_listPoints.SetItemText(row, 5, str);

	// 3. (可选) 清空输入框方便下次输入
	m_curX = 0; m_curY = 0; // ... 
	UpdateData(FALSE);
}

void CPhotogrammetryAppDlg::OnBnClickedBtnResection()
{
	if (!UpdateData(TRUE)) return;

	// --- 新增：比例尺转初值高度 ---
	double sumZ = 0;
	int rowCount = m_listPoints.GetItemCount();
	if (rowCount < 4) { AfxMessageBox(_T("至少需要4个点")); return; }

	for (int i = 0; i < rowCount; i++)
		sumZ += _ttof(m_listPoints.GetItemText(i, 5));
	double Z_avg = sumZ / rowCount;
	// 航高 H = m * f / 1000，Zs = H + Z_avg
	double initZs = (m_scaleM * m_f / 1000.0) + Z_avg;

	if (rowCount < 4) {
		AfxMessageBox(_T("空间后方交会至少需要4个控制点！"));
		return;
	}

	// 1. 从列表中提取所有点
	std::vector<ControlPoint> points;
	for (int i = 0; i < rowCount; i++) {
		ControlPoint pt;
		pt.x = _ttof(m_listPoints.GetItemText(i, 1));
		pt.y = _ttof(m_listPoints.GetItemText(i, 2));
		pt.X = _ttof(m_listPoints.GetItemText(i, 3));
		pt.Y = _ttof(m_listPoints.GetItemText(i, 4));
		pt.Z = _ttof(m_listPoints.GetItemText(i, 5));
		points.push_back(pt);
	}

	// 2. 调用算法
	Resection solver(m_f, m_x0, m_y0, initZs);
	Eigen::MatrixXd Q_mat(6, 6); // 准备接收矩阵的容器
	if (solver.compute(points, m_resXs, m_resYs, m_resZs, m_resPhi, m_resOmega, m_resKappa,
		m_sigma0, Q_mat, m_mXs, m_mYs, m_mZs, m_mPhi, m_mOmg, m_mKap))
	{
		// 1. 处理角度精度单位 (弧度转分)
		m_mPhi *= 3437.75; m_mOmg *= 3437.75; m_mKap *= 3437.75;

		// 2. 刷新矩阵显示
		UpdateQMatrixDisplay(Q_mat);

		// 3. 更新界面其他数值
		UpdateData(FALSE);
		AfxMessageBox(_T("计算成功！精度评价矩阵已更新。"));
	}
	else {
		AfxMessageBox(_T("迭代失败，请检查初值或控制点分布。"));
	}
}

void CPhotogrammetryAppDlg::OnBnClickedBtnDelPt()
{
	int nItem = m_listPoints.GetNextItem(-1, LVNI_SELECTED);
	if (nItem != -1)
	{
		m_listPoints.DeleteItem(nItem);
		// 重新排列剩余点的点号（可选）
		for (int i = 0; i < m_listPoints.GetItemCount(); i++) {
			CString str;
			str.Format(_T("%d"), i + 1);
			m_listPoints.SetItemText(i, 0, str);
		}
	}
	else
	{
		AfxMessageBox(_T("请先选择要删除的行！"));
	}
}

void CPhotogrammetryAppDlg::UpdateQMatrixDisplay(const Eigen::MatrixXd& Q) {
	m_listQMat.DeleteAllItems(); // 清空旧数据
	for (int i = 0; i < 6; i++) {
		int row = m_listQMat.InsertItem(i, _T("")); // 插入新行
		for (int j = 0; j < 6; j++) {
			CString str;
			// 矩阵数值通常很小，建议保留 6-8 位小数
			str.Format(_T("%.8f"), Q(i, j));
			m_listQMat.SetItemText(row, j, str);
		}
	}
}

void CPhotogrammetryAppDlg::OnVScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	int nNewPos = m_nVScrollPos;
	switch (nSBCode)
	{
	case SB_LINEUP:      nNewPos = max(0, m_nVScrollPos - 10); break;
	case SB_LINEDOWN:    nNewPos = min(m_nVScrollMax, m_nVScrollPos + 10); break;
	case SB_PAGEUP:      nNewPos = max(0, m_nVScrollPos - 50); break;
	case SB_PAGEDOWN:    nNewPos = min(m_nVScrollMax, m_nVScrollPos + 50); break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:  nNewPos = nPos; break;
	default: break;
	}

	if (nNewPos != m_nVScrollPos)
	{
		int dy = nNewPos - m_nVScrollPos;
		// 向上滚动（内容向上移动）时 dy>0，应滚动窗口 (0, -dy)
		ScrollWindow(0, -dy);
		m_nVScrollPos = nNewPos;
		SetScrollPos(SB_VERT, m_nVScrollPos, TRUE);
		UpdateWindow();
	}
	CDialogEx::OnVScroll(nSBCode, nPos, pScrollBar);
}

void CPhotogrammetryAppDlg::OnHScroll(UINT nSBCode, UINT nPos, CScrollBar* pScrollBar)
{
	int nNewPos = m_nHScrollPos;
	switch (nSBCode)
	{
	case SB_LINELEFT:    nNewPos = max(0, m_nHScrollPos - 10); break;
	case SB_LINERIGHT:   nNewPos = min(m_nHScrollMax, m_nHScrollPos + 10); break;
	case SB_PAGELEFT:    nNewPos = max(0, m_nHScrollPos - 50); break;
	case SB_PAGERIGHT:   nNewPos = min(m_nHScrollMax, m_nHScrollPos + 50); break;
	case SB_THUMBPOSITION:
	case SB_THUMBTRACK:  nNewPos = nPos; break;
	default: break;
	}

	if (nNewPos != m_nHScrollPos)
	{
		int dx = nNewPos - m_nHScrollPos;
		ScrollWindow(-dx, 0);
		m_nHScrollPos = nNewPos;
		SetScrollPos(SB_HORZ, m_nHScrollPos, TRUE);
		UpdateWindow();
	}
	CDialogEx::OnHScroll(nSBCode, nPos, pScrollBar);
}