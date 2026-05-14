// Forward.cpp
// 功能：实现双像前方交会（利用两幅影像的外方位与像点坐标反算地面点）
// 该文件对旋转矩阵生成函数和前方交会函数的每个重要步骤均添加了逐行注释，
// 便于读者逐句理解实现细节与几何意义（不改变原有算法逻辑）。

#include "pch.h"          // 预编译头，包含常用系统和项目头文件
#include "Forward.h"       // 前方交会函数声明
#include <Eigen/Dense>      // Eigen 向量/矩阵类型
#include <cmath>            // 数学函数（cos, sin, fabs）

// 生成旋转矩阵（与 main 中 computeRotation 对应）
// 参数：phi, omega, kappa - 三个姿态角（弧度）
// 返回：3x3 旋转矩阵 R，满足相机坐标 = R * 相对向量
static Eigen::Matrix3d rotMat(double phi, double omega, double kappa) {
    // 计算三角函数，避免重复计算
    double cphi = cos(phi); // cos(phi)
    double sphi = sin(phi); // sin(phi)
    double com = cos(omega); // cos(omega)
    double som = sin(omega); // sin(omega)
    double cka = cos(kappa); // cos(kappa)
    double ska = sin(kappa); // sin(kappa)

    // 构造旋转矩阵 R
    Eigen::Matrix3d R; // 定义 3x3 矩阵
    R <<
        cphi * cka - sphi * som * ska,   // R(0,0)
        -cphi * ska - sphi * som * cka,  // R(0,1)
        -sphi * com,                     // R(0,2)
        com * ska,                       // R(1,0)
        com * cka,                       // R(1,1)
        -som,                            // R(1,2)
        sphi * cka + cphi * som * ska,   // R(2,0)
        -sphi * ska + cphi * som * cka,  // R(2,1)
        cphi * com;                      // R(2,2)
    // 返回旋转矩阵
    return R;
}

// 前方交会函数
// 说明：根据左右两片的外方位元素和对应像点（以像平面坐标传入），
//       通过在相机坐标系中建立射线并求交近似重建地面点坐标。
// 参数：
//   Xs1,Ys1,Zs1,phi1,omega1,kappa1 - 左片外方位（m, rad）
//   Xs2,Ys2,Zs2,phi2,omega2,kappa2 - 右片外方位（m, rad）
//   f,x0,y0 - 相机内方位（mm）
//   x1,y1,x2,y2 - 左右像点坐标（mm）
// 输出：
//   X,Y,Z - 重建的地面点坐标（m）
// 返回：
//   成功返回 true；若两条投影线平行或数值不稳定返回 false
bool forwardIntersection(
    double Xs1, double Ys1, double Zs1,
    double phi1, double omega1, double kappa1,
    double Xs2, double Ys2, double Zs2,
    double phi2, double omega2, double kappa2,
    double f, double x0, double y0,
    double x1, double y1, double x2, double y2,
    double& out_u1, double& out_v1, double& out_w1,
    double& out_u2, double& out_v2, double& out_w2,
    double& out_N1, double& out_N2,
    double& XA, double& YA, double& ZA,
    double& residual) {
    // p 向量（相机坐标系中的像点方向，单位 mm）
    Eigen::Matrix3d R1 = rotMat(phi1, omega1, kappa1); // 摄站1 旋转矩阵
    Eigen::Matrix3d R2 = rotMat(phi2, omega2, kappa2); // 摄站2 旋转矩阵
    // 输入像坐标单位为 mm，地面坐标为 m。将像空间向量转换为米。
    Eigen::Vector3d p1_mm(x1 - x0, y1 - y0, -f); // 像空间向量1 (mm)
    Eigen::Vector3d p2_mm(x2 - x0, y2 - y0, -f); // 像空间向量2 (mm)
    Eigen::Vector3d p1 = p1_mm / 1000.0; // 转换为 m
    Eigen::Vector3d p2 = p2_mm / 1000.0; // 转换为 m

    // 将像空间向量变换到地面坐标系（比例方向向量）
    Eigen::Vector3d d1 = R1 * p1; // 方向向量1（比例）
    Eigen::Vector3d d2 = R2 * p2; // 方向向量2（比例）

    // 输出像空间辅助坐标
    out_u1 = d1(0); out_v1 = d1(1); out_w1 = d1(2);
    out_u2 = d2(0); out_v2 = d2(1); out_w2 = d2(2);

    // 摄站坐标
    Eigen::Vector3d C1(Xs1, Ys1, Zs1);
    Eigen::Vector3d C2(Xs2, Ys2, Zs2);

    // 处理可能的退化：如果方向向量过小则失败
    double eps = 1e-12;
    if (d1.squaredNorm() < eps || d2.squaredNorm() < eps) return false;

    // 求解最短连线参数 s,t，使得 C1 + s*d1 与 C2 + t*d2 最近
    double A = d1.dot(d1);
    double B = d1.dot(d2);
    double C = d2.dot(d2);
    Eigen::Vector3d r = C1 - C2; // r = C1 - C2
    double E = d1.dot(r);
    double F = d2.dot(r);

    double Den = A * C - B * B;
    if (std::fabs(Den) < 1e-12) return false; // 近似平行或退化

    double s = (B * F - C * E) / Den; // 对应原来 N1
    double t = (A * F - B * E) / Den; // 对应原来 N2

    // 最近点与中点
    Eigen::Vector3d P1 = C1 + s * d1;
    Eigen::Vector3d P2 = C2 + t * d2;
    Eigen::Vector3d mid = 0.5 * (P1 + P2);

    // 设置输出
    out_N1 = s;
    out_N2 = t;
    XA = mid(0); YA = mid(1); ZA = mid(2);
    residual = (P1 - P2).norm();

    return true;
}
bool forwardIntersection(
    double Xs1, double Ys1, double Zs1,
    double phi1, double omega1, double kappa1,
    double Xs2, double Ys2, double Zs2,
    double phi2, double omega2, double kappa2,
    double f, double x0, double y0,
    double x1, double y1, double x2, double y2,
    double& X, double& Y, double& Z) {
    // 将像点从像平面坐标转换到相机参考系的方向向量 p = (x-x0, y-y0, -f)
    Eigen::Matrix3d R1 = rotMat(phi1, omega1, kappa1); // 左片旋转矩阵
    Eigen::Matrix3d R2 = rotMat(phi2, omega2, kappa2); // 右片旋转矩阵
    Eigen::Vector3d p1(x1 - x0, y1 - y0, -f); // 左片像向量
    Eigen::Vector3d p2(x2 - x0, y2 - y0, -f); // 右片像向量

    // 将像平面方向向量旋转到世界坐标系（或相机中心坐标系）
    Eigen::Vector3d uvw1 = R1 * p1; // 左片射线方向（uvw1）
    Eigen::Vector3d uvw2 = R2 * p2; // 右片射线方向（uvw2）
    double u1 = uvw1(0); // 左片射线的 u 分量
    double v1 = uvw1(1); // 左片射线的 v 分量
    double w1 = uvw1(2); // 左片射线的 w 分量
    double u2 = uvw2(0); // 右片射线的 u 分量
    double v2 = uvw2(1); // 右片射线的 v 分量
    double w2 = uvw2(2); // 右片射线的 w 分量

    // 基线向量：右片投影中心相对于左片的差
    double Bu = Xs2 - Xs1; // 基线在 X 方向的分量
    double Bv = Ys2 - Ys1; // 基线在 Y 方向的分量
    double Bw = Zs2 - Zs1; // 基线在 Z 方向的分量

    // 通过消元求解参数（N1），基于两射线参数表达式的关系
    double denom = u1 * w2 - u2 * w1; // 分母项，若为 0 则射线平行或退化
    if (std::fabs(denom) < 1e-8) return false; // 退化判断，返回失败
    double N1 = (Bu * w2 - Bw * u2) / denom; // 求得左片射线参数 N1

    // 根据左片投影中心与 N1 得到地面点坐标（按左片射线参数）
    X = Xs1 + N1 * u1; // 地面点 X
    Y = Ys1 + N1 * v1; // 地面点 Y
    Z = Zs1 + N1 * w1; // 地面点 Z
    return true; // 成功返回
}
