package team100.control;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.GoalOffset;
import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.commands.Arm.DriveToSetpoint;
import frc.robot.commands.Manipulator.Close;
import frc.robot.commands.Manipulator.Home;
import frc.robot.commands.Manipulator.Open;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.autoLevel;
import frc.robot.subsystems.SwerveDriveSubsystem;

public interface Control {

    public XboxController getController();

    public void armHigh(ArmTrajectory command);

    public void armSafe(ArmTrajectory command);
    
    public void open(Open command);

    public void home(Home command);

    public void close(Close command);

    public void resetPose(ResetPose command);

    public void trajtoApril(SwerveDriveSubsystem m_robotDrive, int ID);

    public void driveToAprilTag2(DriveToAprilTag command); 

    public void driveToAprilTag(DriveToAprilTag command);

    // public void driveToWaypoint(DriveToAprilTag command);

    public void driveToHigh(DriveToSetpoint command);

    public void driveToSafe(SequentialCommandGroup command);



    public void driveToLeftGrid(DriveToWaypoint2 command);

    public void driveToCenterGrid(DriveToWaypoint2 command);

    public void driveToRightGrid(DriveToWaypoint2 command);

    public void driveToSubstation(DriveToWaypoint2 command);


    public void autoLevel(autoLevel command);

    // public void sanjanAuto(SanjanAutonomous command);




    // public void driveWithHeading0(DriveWithHeading command);

    // public void driveWithHeading90(DriveWithHeading command);

    // public void driveWithHeading180(DriveWithHeading command);

    // public void driveWithHeading270(DriveWithHeading command);

    public void driveRotation(DriveRotation command);

    public void resetRotation(ResetRotation command);


    // DRIVETRAIN
    public GoalOffset goalOffset();
    /** @return [-1,1] */
    public double xSpeed();

    /** @return [-1,1] */
    public double ySpeed();

    /** @return [-1,1] */
    public double rotSpeed();

    public Rotation2d desiredRotation();

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
