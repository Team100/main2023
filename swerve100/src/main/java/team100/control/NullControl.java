package team100.control;

import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;

public class NullControl implements Control {

    @Override
    public void resetPose(ResetPose command) {
    }

    @Override
    public void moveToAprilTag(MoveToAprilTag command) {
    }

    @Override
    public void autoLevel(frc.robot.commands.autoLevel command) {
    }

    @Override
    public void armHigh(ArmHigh command) {
    }

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
    
}
