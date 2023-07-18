package org.team100.frc2023.control;

import org.team100.frc2023.autonomous.DriveToWaypoint2;
import org.team100.frc2023.autonomous.DriveToWaypoint3;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveScaled;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.SetConeMode;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.CloseSlow;
import org.team100.frc2023.commands.Manipulator.Eject;
import org.team100.frc2023.commands.Manipulator.Home;
import org.team100.frc2023.commands.Manipulator.Open;
import org.team100.frc2023.commands.Retro.DriveToRetroReflectiveTape;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.commands.Retro.LedOn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/** Implementations should do their own deadbanding, scaling, expo, etc. */
public interface Control {

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    void driveToLeftGrid(DriveToWaypoint2 command);

    void autoLevel(AutoLevel command);

    void driveToCenterGrid(DriveToWaypoint2 command);

    void driveToRightGrid(DriveToWaypoint2 command);

    void driveToSubstation(DriveToWaypoint2 command);

    void resetRotation0(ResetRotation command);

    void resetRotation180(ResetRotation command);

    /** forward positive, left positive, counterclockwise positive, [-1,1] */
    Twist2d twist();

    void driveSlow(DriveScaled command);

    void resetPose(ResetPose command);

    Rotation2d desiredRotation();

    GoalOffset goalOffset();

    void defense(Defense defense);

    void rumbleOn();

    void rumbleTrigger(RumbleOn command);

    void rumbleOff();

    void rotate0(Rotate command);

    void driveMedium(DriveScaled command);

    void moveConeWidthLeft(MoveConeWidth command);

    void moveConeWidthRight(MoveConeWidth command);

    void driveWithLQR(DriveToWaypoint3 command);

    ///////////////////////////////
    //
    // OPERATOR: arm and manipulator controls

    /** @return [-1,1] */
    double openSpeed();

    /** @return [-1,1] */
    double closeSpeed();

    /** @return [-1,1] */
    double lowerSpeed();

    /** @return [-1,1] */
    double upperSpeed();

    double armX();
    double armY();

    void armHigh(ArmTrajectory command);

    void armLow(ArmTrajectory command);

    void armSafe(ArmTrajectory command);

    void safeWaypoint(ArmTrajectory command);

    void armSafeSequential(ArmTrajectory command, ArmTrajectory command2);

    void armSafeBack(ArmTrajectory command);

    void closeSlow(CloseSlow command);

    void armSubstation(ArmTrajectory command);

    void armMid(ArmTrajectory command);

    void open(Open command);

    void home(Home command);

    void close(Eject command);

    void cubeMode(SetCubeMode command);

    void coneMode(SetConeMode command);

    void armToSub(ArmTrajectory command);

    void ledOn(LedOn command);

    void oscillate(ArmTrajectory command);

    void tapeDetect(DriveToRetroReflectiveTape command);

    void armSubSafe(ArmTrajectory command);
}
