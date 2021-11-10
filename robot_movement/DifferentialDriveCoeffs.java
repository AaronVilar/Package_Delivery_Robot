import edu.wpi.first.wpilibj.controller.StateSpaceControllerCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpaceLoop;
import edu.wpi.first.wpilibj.controller.StateSpaceObserverCoeffs;
import edu.wpi.first.wpilibj.controller.StateSpacePlantCoeffs;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.*;

public class DifferentialDriveCoeffs {
  public static StateSpacePlantCoeffs<N4, N2, N2>
    makeDifferentialDrivePlantCoeffs() {
    Matrix<N4, N4> A = MatrixUtils.mat(Nat.N4(), Nat.N4()).fill(1.0, 0.004937684721965187, 0.0, -8.603071808824775e-06, 0.0, 0.9751798718961207, 0.0, -0.003412455848986536, 0.0, -8.603071808824773e-06, 1.0, 0.004937684721965186, 0.0, -0.0034124558489865356, 0.0, 0.9751798718961207);
    Matrix<N4, N2> B = MatrixUtils.mat(Nat.N4(), Nat.N2()).fill(2.2303456426485836e-05, 3.07915238883885e-06, 0.008883449823579522, 0.0012213627658478623, 3.07915238883885e-06, 2.2303456426485826e-05, 0.0012213627658478625, 0.00888344982357952);
    Matrix<N2, N4> C = MatrixUtils.mat(Nat.N2(), Nat.N4()).fill(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    Matrix<N2, N2> D = MatrixUtils.mat(Nat.N2(), Nat.N2()).fill(0.0, 0.0, 0.0, 0.0);
    return new StateSpacePlantCoeffs<N4, N2, N2>(Nat.N4(), Nat.N2(), Nat.N2(), A, B, C, D);
  }

  public static StateSpaceControllerCoeffs<N4, N2, N2>
    makeDifferentialDriveControllerCoeffs() {
    Matrix<N2, N4> K = MatrixUtils.mat(Nat.N2(), Nat.N4()).fill(80.7019556931764, 12.723057462542451, -0.5283697116283724, -0.4924368928607337, -0.5283697116287063, -0.49243689286073405, 80.7019556931737, 12.72305746254244);
    Matrix<N2, N4> Kff = MatrixUtils.mat(Nat.N2(), Nat.N4()).fill(0.28802138505167046, 114.7370214693621, -0.03943519971785974, -15.774900065904372, -0.03943519971787586, -15.774900065904374, 0.28802138505165253, 114.73702146936213);
    Matrix<N2, N1> Umin = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(-12.0, -12.0);
    Matrix<N2, N1> Umax = MatrixUtils.mat(Nat.N2(), Nat.N1()).fill(12.0, 12.0);
    return new StateSpaceControllerCoeffs<N4, N2, N2>(K, Kff, Umin, Umax);
  }

  public static StateSpaceObserverCoeffs<N4, N2, N2>
    makeDifferentialDriveObserverCoeffs() {
    Matrix<N4, N2> K = MatrixUtils.mat(Nat.N4(), Nat.N2()).fill(0.9999963001032672, -9.793510653744096e-09, 14.816692744520699, -0.5095801088465637, -9.793510654012382e-09, 0.9999963001032672, -0.5095801088465735, 14.816692744520825);
    return new StateSpaceObserverCoeffs<N4, N2, N2>(K);
  }

  public static StateSpaceLoop<N4, N2, N2> makeDifferentialDriveLoop() {
    return new StateSpaceLoop<N4, N2, N2>(makeDifferentialDrivePlantCoeffs(),
                                          makeDifferentialDriveControllerCoeffs(),
                                          makeDifferentialDriveObserverCoeffs());
  }
}
