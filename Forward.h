#ifndef FORWARD_H
#define FORWARD_H

// forwardIntersection:
//   说明：对两幅影像执行前方交会，给定左右影像的外方位参数与像点坐标，
//         返回重建的地面点坐标。
// 参数：
//   Xs1,Ys1,Zs1 - 左片投影中心（m）
//   phi1,omega1,kappa1 - 左片姿态角（rad）
//   Xs2,Ys2,Zs2 - 右片投影中心（m）
//   phi2,omega2,kappa2 - 右片姿态角（rad）
//   f,x0,y0 - 相机内方位（焦距和主点，单位 mm）
//   x1,y1,x2,y2 - 左右片像点（单位 mm）
// 输出：
//   X,Y,Z - 重建的地面点坐标（m）
// 返回：
//   成功返回 true，退化或数值不稳定返回 false
// Extended version: in addition to ground point (X,Y,Z) this fills:
//  - image-space auxiliary coordinates u1,v1,w1 and u2,v2,w2
//  - projection coefficients N1 (along ray1) and N2 (along ray2)
//  - ground coordinate of intersection (XA,YA,ZA) (midpoint of closest points)
//  - residual distance between the two closest points on the rays
// Angle units for phi/omega/kappa are radians.
bool forwardIntersection(
    double Xs1, double Ys1, double Zs1,
    double phi1, double omega1, double kappa1,
    double Xs2, double Ys2, double Zs2,
    double phi2, double omega2, double kappa2,
    double f, double x0, double y0,
    double x1, double y1, double x2, double y2,
    // outputs
    double& u1, double& v1, double& w1,
    double& u2, double& v2, double& w2,
    double& N1, double& N2,
    double& XA, double& YA, double& ZA,
    double& residual);

#endif
