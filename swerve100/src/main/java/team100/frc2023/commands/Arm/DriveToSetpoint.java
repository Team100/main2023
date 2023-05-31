package team100.frc2023.commands.Arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import team100.frc2023.subsystems.Arm.ArmController;

public class DriveToSetpoint extends ParallelCommandGroup {
    public DriveToSetpoint(ArmController arm, double x, double y, double upperVelocity, double lowerVelocity) {
        addCommands(
                UpperArmToGoal.factory(arm.calculate(x, y).upperTheta, arm, upperVelocity),
                LowerArmToGoal.factory(arm.calculate(x, y).lowerTheta, arm)

        );
    }
}
