package team100.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.autonomous.DriveToWaypoint2;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.GoalOffset;
import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetRotation;
import frc.robot.subsystems.SwerveDriveSubsystem;


/* Uses my RealFlight controller, does not support buttons. */
public class FlightControl implements Control {
    private static final int kPort = 0;

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
    public void armHigh(ArmHigh command) {
    }

    @Override
    public void resetRotation(ResetRotation command) {
    }

    // @Override
    // public void driveWithHeading0(DriveWithHeading command){}

    // @Override
    // public void driveWithHeading90(DriveWithHeading command){}

    // @Override
    // public void driveWithHeading180(DriveWithHeading command){}

    // @Override
    // public void driveWithHeading270(DriveWithHeading command){}

    @Override
    public void driveRotation(DriveRotation command){}
    
    @Override
    public double xSpeed() {
        return -1.25 * DriverStation.getStickAxis(kPort, 1);
    }

    @Override
    public double ySpeed() {
        return -1.25 * DriverStation.getStickAxis(kPort, 0);
    }

    @Override
    public double rotSpeed() {
        return -1.25 * DriverStation.getStickAxis(kPort, 4);
    }

    @Override
    public double throttle() {
        return 0.5 - (0.75 * DriverStation.getStickAxis(kPort, 2));
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
    public void driveToLeftGrid(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToCenterGrid(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToRightGrid(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void driveToSubstation(DriveToWaypoint2 command) {
        // TODO Auto-generated method stub
    }
        
    @Override
    public Rotation2d desiredRotation() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public GoalOffset goalOffset() {
        // TODO Auto-generated method stub
        return null;
    }
}
