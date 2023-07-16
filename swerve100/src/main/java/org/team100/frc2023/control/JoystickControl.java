package org.team100.frc2023.control;

import static org.team100.lib.control.ControlUtil.clamp;
import static org.team100.lib.control.ControlUtil.deadband;
import static org.team100.lib.control.ControlUtil.expo;

import org.team100.frc2023.autonomous.DriveToWaypoint2;
import org.team100.frc2023.autonomous.DriveToWaypoint3;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveScaled;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.SetConeMode;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.CloseSlow;
import org.team100.frc2023.commands.Manipulator.Eject;
import org.team100.frc2023.commands.Manipulator.Home;
import org.team100.frc2023.commands.Manipulator.Open;
import org.team100.frc2023.commands.Retro.DriveToRetroReflectiveTape;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.commands.Retro.LedOn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Experiment for driving swerve with the big joystick.
 * X, Y, and twist should work.
 * POV rotation should work.
 * Only one joystick is required.
 * Operator features are not implemented.
 * Command buttons are not implemented.
 */
public class JoystickControl implements Control, Sendable {
    public static class Config {
        public double kDeadband = 0.02;
        public double kExpo = 0.5;
    }

    private final Config m_config = new Config();
    private final CommandJoystick controller0;
    // private final CommandJoystick controller1;
    private Rotation2d previousRotation = new Rotation2d(0);

    public JoystickControl() {
        controller0 = new CommandJoystick(0);
        System.out.printf("Controller0: %s\n", controller0.getHID().getName());
        // controller1 = new CommandJoystick(1);
        // System.out.printf("Controller1: %s\n", controller1.getHID().getName());
        SmartDashboard.putData("Robot Container", this);
    }

    @Override
    public void driveToLeftGrid(DriveToWaypoint2 command) {
    }

    @Override
    public void autoLevel(AutoLevel command) {
    }

    @Override
    public void driveToCenterGrid(DriveToWaypoint2 command) {
    }

    @Override
    public void driveToRightGrid(DriveToWaypoint2 command) {
    }

    @Override
    public void driveToSubstation(DriveToWaypoint2 command) {
    }

    @Override
    public void resetRotation0(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(controller0.getHID(), 2);
        startButton.onTrue(command);
    }

    @Override
    public void resetRotation180(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(controller0.getHID(), 3);
        startButton.onTrue(command);
    }

    @Override
    public Twist2d twist() {
        double dx = expo(deadband(-1.0 * clamp(controller0.getY(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dy = expo(deadband(-1.0 * clamp(controller0.getX(), 1), m_config.kDeadband, 1), m_config.kExpo);
        double dtheta = expo(deadband(-1.0 * clamp(controller0.getTwist(), 1), m_config.kDeadband, 1), m_config.kExpo);
        return new Twist2d(dx, dy, dtheta);
    }

    @Override
    public Trigger trigger() {
        return controller0.trigger();
    }

    @Override
    public Trigger thumb() {
        return controller0.top();
    }

    @Override
    public void driveSlow(DriveScaled command) {
    }

    @Override
    public void resetPose(ResetPose command) {
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = controller0.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    @Override
    public GoalOffset goalOffset() {
        return GoalOffset.center;
    }

    @Override
    public void defense(Defense defense) {
    }

    @Override
    public void rumbleOn() {
    }

    @Override
    public void rumbleTrigger(RumbleOn command) {
    }

    @Override
    public void rumbleOff() {
    }

    @Override
    public void rotate0(Rotate command) {
    }

    @Override
    public void driveMedium(DriveScaled command) {
    }

    @Override
    public void moveConeWidthLeft(MoveConeWidth command) {
    }

    @Override
    public void moveConeWidthRight(MoveConeWidth command) {
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
    public void armHigh(ArmTrajectory command) {
    }

    @Override
    public void armLow(ArmTrajectory command) {
    }

    @Override
    public void armSafe(ArmTrajectory command) {
    }

    @Override
    public void safeWaypoint(ArmTrajectory command) {
    }

    @Override
    public void armSafeSequential(ArmTrajectory command, ArmTrajectory command2) {
    }

    @Override
    public void armSafeBack(ArmTrajectory command) {
    }

    @Override
    public void closeSlow(CloseSlow command) {
    }

    @Override
    public void armSubstation(ArmTrajectory command) {
    }

    @Override
    public void armMid(ArmTrajectory command) {
    }

    @Override
    public void open(Open command) {
    }

    @Override
    public void home(Home command) {
    }

    @Override
    public void close(Eject command) {
    }

    @Override
    public void cubeMode(SetCubeMode command) {
    }

    @Override
    public void coneMode(SetConeMode command) {
    }

    @Override
    public void armToSub(ArmTrajectory command) {
    }

    @Override
    public void ledOn(LedOn command) {
    }

    @Override
    public void oscillate(ArmTrajectory command) {
    }

    @Override
    public void tapeDetect(DriveToRetroReflectiveTape command) {
    }

    @Override
    public void armSubSafe(ArmTrajectory command) {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("joystick control");
        // builder.addDoubleProperty("right y", () -> controller0.getRightY(), null);
        // builder.addDoubleProperty("right x", () -> controller0.getRightX(), null);
        // builder.addDoubleProperty("left x", () -> controller0.getLeftX(), null);
        // builder.addDoubleProperty("x limited", () -> xLimited(), null);
        // builder.addDoubleProperty("y limtied", () -> yLimited(), null);
        // builder.addDoubleProperty("rot Limited", () -> rotLimited(), null);
    }

    @Override
    public void driveWithLQR(DriveToWaypoint3 command) {
        // TODO: implement this
    }

}
