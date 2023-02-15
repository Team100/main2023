package team100.control;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;

/* Uses my RealFlight controller, does not support buttons. */
public class FlightControl implements Control {
    private static final int kPort = 0;

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

}