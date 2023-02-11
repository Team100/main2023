package team100.control;

import frc.robot.autonomous.MoveToAprilTag;
import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;
import frc.robot.commands.autoLevel;

public interface Control {
    public void resetPose(ResetPose command);

    public void moveToAprilTag(MoveToAprilTag command);

    public void autoLevel(autoLevel command);

    //public void sanjanAuto(SanjanAutonomous command);

    public void armHigh(ArmHigh command);
}
