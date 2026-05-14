#ifndef RESECTION_H
#define RESECTION_H

#include <vector>
#include <Eigen/Dense>

/**
 * ControlPoint: 表示一个控制点的像点与相应地面点。
 *
 * 字段说明：
 *   - x, y: 像平面坐标，单位为毫米(mm)。这是图像测得或像点像素按内方位换算后的坐标，
 *            原点与方向应与相机内方位参数(m_x0,m_y0)一致。
 *   - X, Y, Z: 地面三维坐标，单位为米(m)。用于反算摄站位置和姿态。
 */
struct ControlPoint {
    double x; /**< 像点 x (mm) */
    double y; /**< 像点 y (mm) */
    double X; /**< 地面 X (m) */
    double Y; /**< 地面 Y (m) */
    double Z; /**< 地面 Z (m) */
};

/**
 * Resection: 实现空间后方交会的求解器。
 *
 * 说明：给定若干个控制点（每个控制点同时含像坐标与地面坐标），
 *       通过最小二乘迭代求解摄站的外方位元素：投影中心 (Xs,Ys,Zs)
 *       及姿态角 (phi, omega, kappa)。
 *
 * 单位约定（与工程中其它文件保持一致）：
 *   - 像面坐标及相机内参数 f,x0,y0: 毫米 (mm)
 *   - 地面坐标 X,Y,Z 以及求得的摄站坐标 Xs,Ys,Zs: 米 (m)
 *
 * 构造函数参数：
 *   - f    : 相机主距 (mm)
 *   - x0,y0: 主点坐标 (mm)
 *   - initH: 用于初始化 Zs 的高度偏移 (m)，通常为相机高度的大致值
 *
 * 主要接口：
 *   bool compute(const std::vector<ControlPoint>& points,
 *                double& Xs, double& Ys, double& Zs,
 *                double& phi, double& omega, double& kappa,
 *                double& sigma0,
 *                Eigen::MatrixXd& Q_mat,
 *                double& mXs, double& mYs, double& mZs,
 *                double& mPhi, double& mOmg, double& mKap,
 *                double limitAng = 1e-6, double limitPos = 1e-3, int maxIter = 30);
 *
 * 参数说明：
 *   - points: 输入控制点集合，至少需要 3 个点才能进行求解（实际精度要求通常>=4）。
 *   - Xs,Ys,Zs: 输出的摄站空间坐标 (m)。
 *   - phi,omega,kappa: 输出的姿态角，弧度制 (rad)。
 *   - sigma0: 输出的单位权中误差 (unit weight standard deviation)，对应像坐标单位 (mm)。
 *   - Q_mat: 输出的参数协方差阵（未乘单位权），维度 6x6，对应 [Xs,Ys,Zs,phi,omega,kappa]。
 *   - mXs,mYs,mZs,mPhi,mOmg,mKap: 输出各参数的中误差（实际标准差）= sigma0 * sqrt(diagonal(Q_mat)).
 *   - limitAng, limitPos: 迭代收敛阈值，分别对应角度分量与位置分量的最大允许改变量。
 *   - maxIter: 最大迭代次数，超过则返回当前结果并认为收敛失败。
 *
 * 返回值：
 *   - 成功返回 true（已计算出解与精度评定值）；失败返回 false（如输入点数不足或矩阵奇异）。
 *
 * 备注：
 *   - 算法采用数值差分构造雅可比矩阵并用线性最小二乘法求解更新量，迭代至收敛。
 *   - 用户应保证输入的像点与地面点是一一对应的且单位一致。
 */
class Resection {
public:
    /**
     * 构造函数
     * @param f 相机主距 (mm)
     * @param x0 主点 x 坐标 (mm)
     * @param y0 主点 y 坐标 (mm)
     * @param initH 摄站高度初值 (m)，用于初始化 Zs
     */
    Resection(double f, double x0, double y0, double initH);

    /**
     * 执行空间后方交会计算并进行精度评定。
     * 参考实现采用数值差分法构造设计矩阵并解正规方程得到参数更新量，最后计算残差、单位权中误差和协方差矩阵。
     */
    bool compute(const std::vector<ControlPoint>& points,
        double& Xs, double& Ys, double& Zs,
        double& phi, double& omega, double& kappa,
        double& sigma0,
        Eigen::MatrixXd& Q_mat,
        double& mXs, double& mYs, double& mZs,
        double& mPhi, double& mOmg, double& mKap,
        double limitAng = 1e-6, double limitPos = 1e-3, int maxIter = 30);

private:
    double m_f;   /**< 主距 (mm) */
    double m_x0;  /**< 主点 x (mm) */
    double m_y0;  /**< 主点 y (mm) */
    double m_initH; /**< 用于初始化 Zs 的高度偏移 (m) */
};

#endif // RESECTION_H
