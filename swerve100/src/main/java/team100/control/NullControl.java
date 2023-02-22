package team100.control;

import frc.robot.autonomous.DriveToAprilTag;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
// import frc.robot.commands.ResetPose;
// import frc.robot.commands.ResetPose;
import frc.robot.commands.ResetRotation;


public class NullControl implements Control {

    // @Override
    // public void resetPose(ResetPose command) {
    // }

    @Override
    public void autoLevel(frc.robot.commands.autoLevel command) {
    }

    @Override
    public void armHigh(ArmHigh command) {
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
    
}
