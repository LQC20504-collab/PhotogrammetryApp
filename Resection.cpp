#include "pch.h"
#include "Resection.h"
#include <cmath>

// Resection.cpp
// 功能说明：
//   实现空间后方交会（space resection）求解器的具体算法。该实现
//   使用数值差分构造雅可比矩阵，借助线性最小二乘方法在迭代中逐步
//   更新参数，直至收敛。输入为若干控制点（每点含像坐标与地面坐标），
//   输出为摄站外方位元素 (Xs,Ys,Zs) 以及姿态角 (phi,omega,kappa)，并
//   给出单位权中误差和参数协方差评定结果。
//
// 单位约定：
//   - 像面坐标及相机内参数 f, x0, y0: 毫米 (mm)
//   - 地面坐标 X, Y, Z 以及求得的摄站坐标 Xs, Ys, Zs: 米 (m)
//
// 实现要点：
//   1) 通过 rotationMatrix() 生成由姿态角构成的旋转矩阵 R；
//   2) 通过 collinearity() 使用共线方程计算像点投影位置；
//   3) 使用数值差分（前向差商）计算设计矩阵 A 的各偏导数；
//   4) 求解正规方程得到参数增量并更新参数；
//   5) 迭代至参数增量小于阈值或达到最大迭代次数；
//   6) 计算残差、单位权中误差 sigma0 以及协方差矩阵 Q，进而得到各参数
//      的中误差（mXs, mYs, ...）。

// 构造函数：保存相机内方位参数及高度初值
Resection::Resection(double f, double x0, double y0, double initH)
    : m_f(f), m_x0(x0), m_y0(y0), m_initH(initH) {
}

// 生成旋转矩阵（与 main 中 computeRotation 对应）
/**
 * rotationMatrix
 * 依据姿态角 phi, omega, kappa 构造从地面坐标系到相机坐标系的旋转矩阵 R。
 * 参数均为弧度制 (rad)。返回 3x3 矩阵，按照常见的航测旋转序列构造。
 */
static Eigen::Matrix3d rotationMatrix(double phi, double omega, double kappa) {
    // 计算三角函数以减少重复运算
    double cphi = cos(phi);
    double sphi = sin(phi);
    double com = cos(omega);
    double som = sin(omega);
    double cka = cos(kappa);
    double ska = sin(kappa);

    // 构造旋转矩阵（矩阵元素注释表示行列索引）
    Eigen::Matrix3d R;
    R <<
        cphi * cka - sphi * som * ska,  // R(0,0)
        -cphi * ska - sphi * som * cka, // R(0,1)
        -sphi * com,                    // R(0,2)
        com * ska,                      // R(1,0)
        com * cka,                      // R(1,1)
        -som,                           // R(1,2)
        sphi * cka + cphi * som * ska,  // R(2,0)
        -sphi * ska + cphi * som * cka, // R(2,1)
        cphi * com;                     // R(2,2)
    return R;
}

// 共线方程：计算在给定外方位下的像点坐标（x_calc,y_calc）
/**
 * collinearity
 * 使用共线方程将地面点 (X,Y,Z) 投影到像平面，得到像坐标 (x_calc,y_calc)。
 * 输入参数：
 *   - X,Y,Z: 地面点坐标 (m)
 *   - Xs,Ys,Zs: 摄站坐标 (m)
 *   - R: 由姿态角构造的旋转矩阵
 *   - f,x0,y0: 相机内方位量 (mm)
 * 输出参数通过引用返回 x_calc, y_calc (mm)
 */
static void collinearity(double X, double Y, double Z,
    double Xs, double Ys, double Zs,
    const Eigen::Matrix3d& R,
    double f, double x0, double y0,
    double& x_calc, double& y_calc) {
    // 计算地面点到投影中心的差向量（单位：m）
    double dX = X - Xs;
    double dY = Y - Ys;
    double dZ = Z - Zs;
    // 在相机坐标系中计算 U,V,W 分量（注意 R 的构造对应的坐标变换）
    double U = R(0, 0) * dX + R(1, 0) * dY + R(2, 0) * dZ;
    double V = R(0, 1) * dX + R(1, 1) * dY + R(2, 1) * dZ;
    double W = R(0, 2) * dX + R(1, 2) * dY + R(2, 2) * dZ;
    // 共线方程：x = x0 - f * U / W，y = y0 - f * V / W
    // （单位：f,x0,y0 为 mm，而 U,V,W 在 m 单位下比值仍正确，但需保证单位一致）
    x_calc = x0 - f * U / W;
    y_calc = y0 - f * V / W;
}

// compute：使用最小二乘法迭代求解空间后方交会
/**
 * compute
 * 使用最小二乘迭代法求解空间后方交会并进行精度评定。
 * 步骤概述：
 *   1) 初始化：以地面点平均坐标作为 Xs,Ys 初值，Zs 取平均 + m_initH；姿态角初值为 0；
 *   2) 迭代：对每个控制点，计算观测量与计算量的差 L，使用数值差分构造设计矩阵 A，
 *      求解正规方程得到参数增量 dX 并更新参数；
 *   3) 收敛判据：当位置与角度的最大改变量同时小于给定阈值时停止迭代；
 *   4) 精度评定：计算最终残差、单位权中误差 sigma0、参数协方差矩阵 Q 以及各参数的中误差。
 */
bool Resection::compute(const std::vector<ControlPoint>& points,
    double& Xs, double& Ys, double& Zs,
    double& phi, double& omega, double& kappa,
    double& sigma0,
    Eigen::MatrixXd& Q_mat,
    double& mXs, double& mYs, double& mZs,
    double& mPhi, double& mOmg, double& mKap,
    double limitAng, double limitPos, int maxIter) {

    // 输入点数校验：至少需要 3 个控制点才能形成本实现的 6 个未知量的方程组
    if (points.size() < 3) return false;

    // --- 1. 赋初值 ---
    // 平均地面坐标作为 Xs,Ys 的初值，Zs 使用平均地面高加上 m_initH
    Xs = 0.0; Ys = 0.0; Zs = 0.0;
    for (const auto& p : points) { Xs += p.X; Ys += p.Y; Zs += p.Z; }
    Xs /= points.size(); Ys /= points.size();
    Zs = Zs / points.size() + m_initH;
    phi = omega = kappa = 0.0;

    int n = static_cast<int>(points.size());
    Eigen::MatrixXd A(2 * n, 6);
    Eigen::VectorXd L(2 * n);
    Eigen::VectorXd dX;

    // --- 2. 迭代求解 ---
    for (int iter = 0; iter < maxIter; ++iter) {
        for (int i = 0; i < n; ++i) {
            double Xg = points[i].X, Yg = points[i].Y, Zg = points[i].Z;
            double x_obs = points[i].x, y_obs = points[i].y;

            Eigen::Matrix3d R = rotationMatrix(phi, omega, kappa);
            double x_calc = 0.0, y_calc = 0.0;
            collinearity(Xg, Yg, Zg, Xs, Ys, Zs, R, m_f, m_x0, m_y0, x_calc, y_calc);
            // 观测-计算（单位：mm）
            L(2 * i) = x_obs - x_calc;
            L(2 * i + 1) = y_obs - y_calc;

            // 使用 lambda 便于用数值差分计算偏导数
            auto fVal = [this, Xg, Yg, Zg](double Xs_, double Ys_, double Zs_,
                double phi_, double omega_, double kappa_) -> Eigen::Vector2d {
                    Eigen::Matrix3d R_ = rotationMatrix(phi_, omega_, kappa_);
                    double xc = 0.0, yc = 0.0;
                    collinearity(Xg, Yg, Zg, Xs_, Ys_, Zs_, R_, this->m_f, this->m_x0, this->m_y0, xc, yc);
                    return Eigen::Vector2d(xc, yc);
                };

            // 数值差分步长：位置以米为单位，角度以弧度为单位
            double deltaPos = 0.01;    // m
            double deltaAng = 1e-6;    // rad
            Eigen::Vector2d f0 = fVal(Xs, Ys, Zs, phi, omega, kappa);

            // 填充设计矩阵 A 的对应两行（x,y 对应的偏导）
            A.row(2 * i) << (fVal(Xs + deltaPos, Ys, Zs, phi, omega, kappa)(0) - f0(0)) / deltaPos,
                (fVal(Xs, Ys + deltaPos, Zs, phi, omega, kappa)(0) - f0(0)) / deltaPos,
                (fVal(Xs, Ys, Zs + deltaPos, phi, omega, kappa)(0) - f0(0)) / deltaPos,
                (fVal(Xs, Ys, Zs, phi + deltaAng, omega, kappa)(0) - f0(0)) / deltaAng,
                (fVal(Xs, Ys, Zs, phi, omega + deltaAng, kappa)(0) - f0(0)) / deltaAng,
                (fVal(Xs, Ys, Zs, phi, omega, kappa + deltaAng)(0) - f0(0)) / deltaAng;

            A.row(2 * i + 1) << (fVal(Xs + deltaPos, Ys, Zs, phi, omega, kappa)(1) - f0(1)) / deltaPos,
                (fVal(Xs, Ys + deltaPos, Zs, phi, omega, kappa)(1) - f0(1)) / deltaPos,
                (fVal(Xs, Ys, Zs + deltaPos, phi, omega, kappa)(1) - f0(1)) / deltaPos,
                (fVal(Xs, Ys, Zs, phi + deltaAng, omega, kappa)(1) - f0(1)) / deltaAng,
                (fVal(Xs, Ys, Zs, phi, omega + deltaAng, kappa)(1) - f0(1)) / deltaAng,
                (fVal(Xs, Ys, Zs, phi, omega, kappa + deltaAng)(1) - f0(1)) / deltaAng;
        }

        // 正规方程求解参数增量（采用 LDLT 更稳定）
        dX = (A.transpose() * A).ldlt().solve(A.transpose() * L);
        Xs += dX(0); Ys += dX(1); Zs += dX(2);
        phi += dX(3); omega += dX(4); kappa += dX(5);

        // 收敛判据：位置与角度改变量同时满足阈值
        if (dX.head(3).cwiseAbs().maxCoeff() < limitPos && dX.tail(3).cwiseAbs().maxCoeff() < limitAng) break;
    }

    // --- 3. 精度评定 ---
    Eigen::Matrix3d R = rotationMatrix(phi, omega, kappa);

    for (int i = 0; i < n; ++i) {
        double Xg = points[i].X, Yg = points[i].Y, Zg = points[i].Z;
        double x_obs = points[i].x, y_obs = points[i].y;

        double x_calc = 0.0, y_calc = 0.0;
        collinearity(Xg, Yg, Zg, Xs, Ys, Zs, R, m_f, m_x0, m_y0, x_calc, y_calc);

        L(2 * i) = x_obs - x_calc;
        L(2 * i + 1) = y_obs - y_calc;

        auto fVal = [this, Xg, Yg, Zg](double Xs_, double Ys_, double Zs_,
            double phi_, double omega_, double kappa_) -> Eigen::Vector2d {
                Eigen::Matrix3d R_ = rotationMatrix(phi_, omega_, kappa_);
                double xc = 0.0, yc = 0.0;
                collinearity(Xg, Yg, Zg, Xs_, Ys_, Zs_, R_,
                    this->m_f, this->m_x0, this->m_y0, xc, yc);
                return Eigen::Vector2d(xc, yc);
            };

        double deltaPos = 0.01;
        double deltaAng = 1e-6;
        Eigen::Vector2d f0 = fVal(Xs, Ys, Zs, phi, omega, kappa);

        A.row(2 * i) <<
            (fVal(Xs + deltaPos, Ys, Zs, phi, omega, kappa)(0) - f0(0)) / deltaPos,
            (fVal(Xs, Ys + deltaPos, Zs, phi, omega, kappa)(0) - f0(0)) / deltaPos,
            (fVal(Xs, Ys, Zs + deltaPos, phi, omega, kappa)(0) - f0(0)) / deltaPos,
            (fVal(Xs, Ys, Zs, phi + deltaAng, omega, kappa)(0) - f0(0)) / deltaAng,
            (fVal(Xs, Ys, Zs, phi, omega + deltaAng, kappa)(0) - f0(0)) / deltaAng,
            (fVal(Xs, Ys, Zs, phi, omega, kappa + deltaAng)(0) - f0(0)) / deltaAng;

        A.row(2 * i + 1) <<
            (fVal(Xs + deltaPos, Ys, Zs, phi, omega, kappa)(1) - f0(1)) / deltaPos,
            (fVal(Xs, Ys + deltaPos, Zs, phi, omega, kappa)(1) - f0(1)) / deltaPos,
            (fVal(Xs, Ys, Zs + deltaPos, phi, omega, kappa)(1) - f0(1)) / deltaPos,
            (fVal(Xs, Ys, Zs, phi + deltaAng, omega, kappa)(1) - f0(1)) / deltaAng,
            (fVal(Xs, Ys, Zs, phi, omega + deltaAng, kappa)(1) - f0(1)) / deltaAng,
            (fVal(Xs, Ys, Zs, phi, omega, kappa + deltaAng)(1) - f0(1)) / deltaAng;
    }

    // 最终残差向量（观测 - 计算）取负号以符合符号习惯
    Eigen::VectorXd V = -L;

    // 单位权中误差 sigma0
    double VV = V.transpose() * V;
    sigma0 = sqrt(VV / (2 * n - 6));

    // 协方差矩阵 Q = (A^T A)^{-1}
    Eigen::MatrixXd Q = (A.transpose() * A).inverse();
    Q_mat = Q;
    // 各参数的中误差 = sigma0 * sqrt(对应协方差对角元素)
    mXs = sigma0 * sqrt(Q(0, 0));
    mYs = sigma0 * sqrt(Q(1, 1));
    mZs = sigma0 * sqrt(Q(2, 2));
    mPhi = sigma0 * sqrt(Q(3, 3));
    mOmg = sigma0 * sqrt(Q(4, 4));
    mKap = sigma0 * sqrt(Q(5, 5));

    return true;
}
