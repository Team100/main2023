package team100.control;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.MoveToAprilTag;
//import frc.robot.autonomous.SanjanAutonomous;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.DriveRotation;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Control, Sendable {
    private final CommandXboxController controller0;
    private final CommandXboxController controller1;

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        System.out.printf("Controller0: %s\n",
                controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        System.out.printf("Controller1: %s\n",
                controller1.getHID().getName());
        SmartDashboard.putData("Robot Container", this);
    }


    @Override
    public void resetPose(ResetPose command) {
        // TODO: choose one
        controller0.leftBumper().onTrue(command);
        // controller0.a().onTrue(command);
    }

    public void driveSlow(SwerveDriveSubsystem m_robotDrive){
        // controller0.rightBumper().onTrue(m_robotDrive.driveSl)
    }

    @Override
    public void moveToAprilTag(MoveToAprilTag command) {
        controller0.b().onTrue(command);
    }

    // TODO: decide what "Y" should do.

    @Override
    public void autoLevel(frc.robot.commands.autoLevel command) {
        controller0.y().whileTrue(command);
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
    public void driveWithHeading0(DriveWithHeading command){
        controller0.povUp().onTrue(command);
    }

    @Override
    public void driveWithHeading90(DriveWithHeading command){
        controller0.povLeft().onTrue(command);
    }

    @Override
    public void driveWithHeading180(DriveWithHeading command){
        controller0.povDown().onTrue(command);
    }

    @Override
    public void driveWithHeading270(DriveWithHeading command){
        controller0.povRight().onTrue(command);
    }

    @Override
    public void driveRotation(DriveRotation command){
        controller0.rightBumper().whileTrue(command);
    }


    @Override
    public double xSpeed() {
        return -1.0 * controller0.getRightY();
    }

    @Override
    public double ySpeed() {
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
