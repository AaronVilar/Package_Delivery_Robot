#pragma once

#include <frc/controller/StateSpaceControllerCoeffs.h>
#include <frc/controller/StateSpaceLoop.h>
#include <frc/controller/StateSpaceObserverCoeffs.h>
#include <frc/controller/StateSpacePlantCoeffs.h>

frc::StateSpacePlantCoeffs<4, 2, 2> MakeDifferentialDrivePlantCoeffs();
frc::StateSpaceControllerCoeffs<4, 2, 2>
MakeDifferentialDriveControllerCoeffs();
frc::StateSpaceObserverCoeffs<4, 2, 2> MakeDifferentialDriveObserverCoeffs();
frc::StateSpaceLoop<4, 2, 2> MakeDifferentialDriveLoop();
