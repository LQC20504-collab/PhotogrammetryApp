
// PhotogrammetryApp.h: PROJECT_NAME 应用程序的主头文件
//

#pragma once

#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含 'pch.h' 以生成 PCH"
#endif

#include "resource.h"		// 主符号
#include <gdiplus.h>

// CPhotogrammetryAppApp:
// 有关此类的实现，请参阅 PhotogrammetryApp.cpp
//

class CPhotogrammetryAppApp : public CWinApp
{
public:
	CPhotogrammetryAppApp();

	ULONG_PTR m_gdiplusToken;

	virtual BOOL InitInstance() override;
	virtual int ExitInstance() override;

// 实现

	DECLARE_MESSAGE_MAP()
};

extern CPhotogrammetryAppApp theApp;
