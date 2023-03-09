package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoLevel extends CommandBase {
    private final SwerveDriveSubsystem drivetrain;
    private AHRS m_gyro;
    private int count = 0;
    double startX = 0;

    public AutoLevel(AHRS gyro, SwerveDriveSubsystem we) {
        drivetrain = we;
        m_gyro = gyro;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startX = drivetrain.getPose().getX();
        count = 0;
    }

    public void execute() {
        if (drivetrain.getPose().getX() <= 4.155) {
            double Roll = m_gyro.getRoll();
            double Pitch = m_gyro.getPitch();
            // System.out.println(Roll);
            double driveRollAmount = MathUtil.clamp(0.005 * Roll, -0.08, 0.08);
            double drivePitchAmount = MathUtil.clamp(0.005 * Pitch, -0.08, 0.08);
            System.out.println(drivePitchAmount);

            if (Math.abs(Roll) > 2.5 || Math.abs(Pitch) > 2.5) {
                count = 0;
                drivetrain.drive(drivePitchAmount, -driveRollAmount, 0, false);
            } else {
                count++;
            }
        } else {
            drivetrain.drive(-0.3, 0, 0, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return count >= 20;
    }
}
