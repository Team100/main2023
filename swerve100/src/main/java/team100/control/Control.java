package team100.control;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.ResetPose;
// import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetRotation;
// import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;

public interface Control {
    public void resetPose(ResetPose command);

    public void trajtoApril(SwerveDriveSubsystem m_robotDrive, int ID);

    public void driveToAprilTag2(DriveToAprilTag command); 

    public void driveToAprilTag(DriveToAprilTag command);

    // public void driveToWaypoint(DriveToAprilTag command);


    public void driveToID1(DriveToWaypoint2 command);

    public void driveToID2(DriveToWaypoint2 command);

    public void driveToID3(DriveToWaypoint2 command);

    public void driveToID4(DriveToWaypoint2 command);


    public void autoLevel(autoLevel command);

    // public void sanjanAuto(SanjanAutonomous command);


    public void armHigh(ArmHigh command);


    public void driveWithHeading0(DriveWithHeading command);

    public void driveWithHeading90(DriveWithHeading command);

    public void driveWithHeading180(DriveWithHeading command);

    public void driveWithHeading270(DriveWithHeading command);

    public void driveRotation(DriveRotation command);

    public void resetRotation(ResetRotation command);


    // DRIVETRAIN
    /** @return [-1,1] */
    public double xSpeed();

    /** @return [-1,1] */
    public double ySpeed();

    /** @return [-1,1] */
    public double rotSpeed();

    /** @return [0, 1] */
    public double throttle();

    // MANIPULATOR
    /** @return [-1,1] */
    public double openSpeed();

    /** @return [-1,1] */
    public double closeSpeed();

    // ARM
    /** @return [-1,1] */
    public double lowerSpeed();

    /** @return [-1,1] */
    public double upperSpeed();
}
