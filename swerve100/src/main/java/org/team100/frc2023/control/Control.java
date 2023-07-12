package org.team100.frc2023.control;

import org.team100.frc2023.autonomous.DriveToWaypoint2;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveMedium;
import org.team100.frc2023.commands.DriveSlow;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.Oscillate;
import org.team100.frc2023.commands.Arm.SetConeMode;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Close;
import org.team100.frc2023.commands.Manipulator.CloseSlow;
import org.team100.frc2023.commands.Manipulator.Home;
import org.team100.frc2023.commands.Manipulator.Open;
import org.team100.frc2023.commands.Retro.DriveToRetroReflectiveTape;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.commands.Retro.LedOn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    /** Forward is positive. @return [-1,1] */
    double xSpeed();

    /** Forward is positive. @return [-1,1] */
    double ySpeed();

    /** Counterclockwise is positive. @return [-1,1] */
    double rotSpeed();

    // used for position control
    Trigger trigger();

    Trigger thumb();

    void driveSlow(DriveSlow command);

    void resetPose(ResetPose command);

    Rotation2d desiredRotation();

    GoalOffset goalOffset();

    void defense(Defense defense);

    void rumbleOn();

    void rumbleTrigger(RumbleOn command);

    void rumbleOff();

    void rotate0(Rotate command);

    void driveMedium(DriveMedium command);

    void moveConeWidthLeft(MoveConeWidth command);

    void moveConeWidthRight(MoveConeWidth command);

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

    void close(Close command);

    void cubeMode(SetCubeMode command);

    void coneMode(SetConeMode command);

    void armToSub(ArmTrajectory command);

    void ledOn(LedOn command);

    void oscillate(Oscillate command);

    void tapeDetect(DriveToRetroReflectiveTape command);

    void armSubSafe(ArmTrajectory command);
}
