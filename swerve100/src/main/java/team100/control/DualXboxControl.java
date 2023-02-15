package team100.control;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.MoveToAprilTag;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ResetPose;

/**
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Control, Sendable {
    // TODO: express these limits in m/s.
    private static final int ySlewRateLimit = 3;
    private static final int xSlewRateLimit = 3;
    
    private final CommandXboxController controller0;
    private final CommandXboxController controller1;

    // private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(xSlewRateLimit);
    // private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(ySlewRateLimit);

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        System.out.printf("Controller0: %s\n", controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        System.out.printf("Controller1: %s\n", controller1.getHID().getName());
        SmartDashboard.putData("Robot Container", this);
    }

    @Override
    public void resetPose(ResetPose command) {
        // TODO: choose one
        controller0.leftBumper().onTrue(command);
        controller0.a().onTrue(command);
    }

    @Override
    public void moveToAprilTag(MoveToAprilTag command) {
        controller0.b().onTrue(command);
    }

    // TODO: decide what "Y" should do.

    @Override
    public void autoLevel(frc.robot.commands.autoLevel command) {
        controller0.y().onTrue(command);
    }

    // @Override
    // public void sanjanAuto(SanjanAutonomous command) {
    // controller0.y().onTrue(command);
    // }

    @Override
    public void armHigh(ArmHigh command) {
        controller1.b().onTrue(command);
    }

    @Override
    public double xSpeed() {
        //return m_xspeedLimiter.calculate(-1.0 * controller0.getRightY());
        return -1.0 * controller0.getRightY();
    }

    @Override
    public double ySpeed() {
        //return m_yspeedLimiter.calculate(-1.0 * controller0.getRightX());
        return -1.0 * controller0.getRightX();
    }

    @Override
    public double rotSpeed() {
        return -1.0 * controller0.getLeftX();
    }

    @Override
    public double throttle() {
        return 1.0;
    }

    @Override
    public double openSpeed() {
        return controller1.getRightTriggerAxis();
    }

    @Override
    public double closeSpeed() {
        return controller1.getLeftTriggerAxis();
    }

    @Override
    public double lowerSpeed() {
        return controller1.getRightX();
    }

    @Override
    public double upperSpeed() {
        return controller1.getLeftY();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("xbox control");
        builder.addDoubleProperty("right y", () -> controller0.getRightY(), null);
        builder.addDoubleProperty("right x", () -> controller0.getRightX(), null);
        builder.addDoubleProperty("left x", () -> controller0.getLeftX(), null);
    }
}
