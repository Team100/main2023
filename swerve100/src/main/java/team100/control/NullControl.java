package team100.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.ResetPose;
// import frc.robot.commands.ResetPose;
// import frc.robot.commands.ResetPose;
// import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetRotation;
import frc.robot.commands.Arm.ArmTrajectory;
import frc.robot.commands.Arm.DriveToSetpoint;
import frc.robot.commands.Manipulator.Close;
import frc.robot.commands.Manipulator.Home;
import frc.robot.commands.Manipulator.Open;
import frc.robot.subsystems.SwerveDriveSubsystem;


public class NullControl implements Control {

    @Override
    public void resetPose(ResetPose command) {
    }

    @Override
    public void trajtoApril(SwerveDriveSubsystem m_robotDrive, int ID){
    };

    @Override
    public void autoLevel(frc.robot.commands.autoLevel command) {
    }


    @Override
    public void resetRotation(ResetRotation command) {
    }

    @Override
    public void driveWithHeading0(DriveWithHeading command){}

    @Override
    public void driveWithHeading90(DriveWithHeading command){}

    @Override
    public void driveWithHeading180(DriveWithHeading command){}

    @Override
    public void driveWithHeading270(DriveWithHeading command){}

    @Override
    public void driveRotation(DriveRotation command){}

    @Override
    public double xSpeed() {
        return 0;
    }

    @Override
    public double ySpeed() {
        return 0;
    }

    @Override
    public double rotSpeed() {
        return 0;
    }

    @Override
    public double throttle() {
        return 0;
    }

    @Override
    public double openSpeed() {
        return 0;
    }

    @Override
    public double closeSpeed() {

        return 0;
    }

    @Override
    public double lowerSpeed() {
        return 0;
    }

    @Override
    public double upperSpeed() {
        return 0;
    }

    @Override
    public void driveToAprilTag(DriveToAprilTag command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToAprilTag2(DriveToAprilTag command) {
        // TODO Auto-generated method stub
        
    }

    // @Override
    // public void driveToWaypoint(DriveToWaypoint2 command) {
    //     // TODO Auto-generated method stub
        
    // }

    @Override
    public void driveToID1(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToID2(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToID3(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToID4(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToHigh(DriveToSetpoint command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToSafe(SequentialCommandGroup command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public XboxController getController() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void armHigh(ArmTrajectory command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void armSafe(ArmTrajectory command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void open(Open command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void home(Home command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void close(Close command) {
        // TODO Auto-generated method stub
        
    }
    
}
