package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class DriveToSetpoint extends ParallelCommandGroup {
    public DriveToSetpoint(ArmController arm, double x, double y, double upperVelocity, double lowerVelocity) {
        addCommands(
                new UpperArmToGoal(arm.calculate(x, y).upperTheta, arm, upperVelocity),
                new LowerArmToGoal(arm.calculate(x, y).lowerTheta, arm));
    }
}
