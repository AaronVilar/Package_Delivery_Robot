#include "subsystems/\DifferentialDriveCoeffs.hpp"

#include <Eigen/Core>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDifferentialDrivePlantCoeffs() {
  Eigen::Matrix<double, 4, 4> A;
  A(0, 0) = 1.0;
  A(0, 1) = 0.004937684721965187;
  A(0, 2) = 0.0;
  A(0, 3) = -8.603071808824775e-06;
  A(1, 0) = 0.0;
  A(1, 1) = 0.9751798718961207;
  A(1, 2) = 0.0;
  A(1, 3) = -0.003412455848986536;
  A(2, 0) = 0.0;
  A(2, 1) = -8.603071808824773e-06;
  A(2, 2) = 1.0;
  A(2, 3) = 0.004937684721965186;
  A(3, 0) = 0.0;
  A(3, 1) = -0.0034124558489865356;
  A(3, 2) = 0.0;
  A(3, 3) = 0.9751798718961207;
  Eigen::Matrix<double, 4, 2> B;
  B(0, 0) = 2.2303456426485836e-05;
  B(0, 1) = 3.07915238883885e-06;
  B(1, 0) = 0.008883449823579522;
  B(1, 1) = 0.0012213627658478623;
  B(2, 0) = 3.07915238883885e-06;
  B(2, 1) = 2.2303456426485826e-05;
  B(3, 0) = 0.0012213627658478625;
  B(3, 1) = 0.00888344982357952;
  Eigen::Matrix<double, 2, 4> C;
  C(0, 0) = 1.0;
  C(0, 1) = 0.0;
  C(0, 2) = 0.0;
  C(0, 3) = 0.0;
  C(1, 0) = 0.0;
  C(1, 1) = 0.0;
  C(1, 2) = 1.0;
  C(1, 3) = 0.0;
  Eigen::Matrix<double, 2, 2> D;
  D(0, 0) = 0.0;
  D(0, 1) = 0.0;
  D(1, 0) = 0.0;
  D(1, 1) = 0.0;
  return frc::StateSpacePlantCoeffs<4, 2, 2>(A, B, C, D);
}

frc::StateSpaceControllerCoeffs<4, 2, 2>
MakeDifferentialDriveControllerCoeffs() {
  Eigen::Matrix<double, 2, 4> K;
  K(0, 0) = 80.7019556931764;
  K(0, 1) = 12.723057462542451;
  K(0, 2) = -0.5283697116283724;
  K(0, 3) = -0.4924368928607337;
  K(1, 0) = -0.5283697116287063;
  K(1, 1) = -0.49243689286073405;
  K(1, 2) = 80.7019556931737;
  K(1, 3) = 12.72305746254244;
  Eigen::Matrix<double, 2, 4> Kff;
  Kff(0, 0) = 0.28802138505167046;
  Kff(0, 1) = 114.7370214693621;
  Kff(0, 2) = -0.03943519971785974;
  Kff(0, 3) = -15.774900065904372;
  Kff(1, 0) = -0.03943519971787586;
  Kff(1, 1) = -15.774900065904374;
  Kff(1, 2) = 0.28802138505165253;
  Kff(1, 3) = 114.73702146936213;
  Eigen::Matrix<double, 2, 1> Umin;
  Umin(0, 0) = -12.0;
  Umin(1, 0) = -12.0;
  Eigen::Matrix<double, 2, 1> Umax;
  Umax(0, 0) = 12.0;
  Umax(1, 0) = 12.0;
  return frc::StateSpaceControllerCoeffs<4, 2, 2>(K, Kff, Umin, Umax);
}

frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDifferentialDriveObserverCoeffs() {
  Eigen::Matrix<double, 4, 2> K;
  K(0, 0) = 0.9999963001032672;
  K(0, 1) = -9.793510653744096e-09;
  K(1, 0) = 14.816692744520699;
  K(1, 1) = -0.5095801088465637;
  K(2, 0) = -9.793510654012382e-09;
  K(2, 1) = 0.9999963001032672;
  K(3, 0) = -0.5095801088465735;
  K(3, 1) = 14.816692744520825;
  return frc::StateSpaceObserverCoeffs<4, 2, 2>(K);
}

frc::StateSpaceLoop<4, 2, 2> MakeDifferentialDriveLoop() {
  return frc::StateSpaceLoop<4, 2, 2>(MakeDifferentialDrivePlantCoeffs(),
                                      MakeDifferentialDriveControllerCoeffs(),
                                      MakeDifferentialDriveObserverCoeffs());
}
