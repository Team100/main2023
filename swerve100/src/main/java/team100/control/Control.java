package team100.control;

import frc.robot.autonomous.MoveToAprilTag;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;

public interface Control {
    public void resetPose(ResetPose command);

    public void moveToAprilTag(MoveToAprilTag command);

    public void autoLevel(autoLevel command);

    // public void sanjanAuto(SanjanAutonomous command);

    public void armHigh(ArmHigh command);

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
