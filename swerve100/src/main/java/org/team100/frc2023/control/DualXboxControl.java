package org.team100.frc2023.control;

import org.team100.frc2023.autonomous.DriveToWaypoint2;
import org.team100.frc2023.autonomous.MoveConeWidth;
import org.team100.frc2023.autonomous.Rotate;
import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.Defense;
import org.team100.frc2023.commands.DriveMedium;
import org.team100.frc2023.commands.DriveSlow;
import org.team100.frc2023.commands.GoalOffset;
import org.team100.frc2023.commands.ResetPose;
import org.team100.frc2023.commands.ResetRotation;
import org.team100.frc2023.commands.RumbleOn;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.Oscillate;
import org.team100.frc2023.commands.Arm.SetConeMode;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Close;
import org.team100.frc2023.commands.Manipulator.CloseSlow;
import org.team100.frc2023.commands.Manipulator.Home;
import org.team100.frc2023.commands.Manipulator.Open;
import org.team100.frc2023.commands.Retro.DriveToRetroReflectiveTape;
import org.team100.frc2023.commands.Retro.LedOn;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * see
 * https://docs.google.com/document/d/1M89x_IiguQdY0VhQlOjqADMa6SYVp202TTuXZ1Ps280/edit#
 */
public class DualXboxControl implements Sendable {
    // private static final double kDtSeconds = 0.02;
    // private static final double kMaxRotationRateRadiansPerSecond = Math.PI;
    private static final double kTriggerThreshold = .5;

    private final CommandXboxController controller0;
    private final CommandXboxController controller1;
    Rotation2d previousRotation = new Rotation2d(0);

    public DualXboxControl() {
        controller0 = new CommandXboxController(0);
        System.out.printf("Controller0: %s\n", controller0.getHID().getName());
        controller1 = new CommandXboxController(1);
        System.out.printf("Controller1: %s\n", controller1.getHID().getName());
        SmartDashboard.putData("Robot Container", this);
    }

    ///////////////////////////////
    //
    // DRIVER: manual driving and auto navigation controls

    public void driveToLeftGrid(DriveToWaypoint2 command) {
        // controller0.x().whileTrue(command);
    };

    public void autoLevel(AutoLevel command){
        // controller0.x().whileTrue(command);
    }

    public void driveToCenterGrid(DriveToWaypoint2 command) {
        // controller0.a().whileTrue(command);
    };

    public void driveToRightGrid(DriveToWaypoint2 command) {
        // controller0.b().whileTrue(command);
    };

    public void driveToSubstation(DriveToWaypoint2 command) {
        // controller0.y().whileTrue(command);
    };

    public void resetRotation0(ResetRotation command) {
        JoystickButton startButton = new JoystickButton(controller0.getHID(), 7);
        startButton.onTrue(command);
    }

    public void resetRotation180(ResetRotation command) {
            JoystickButton startButton = new JoystickButton(controller0.getHID(), 8);
            startButton.onTrue(command);
        }

        /** @return [-1,1] */
        public double xSpeed() {
            return -1.0 * controller0.getRightY();
    }

    /** @return [-1,1] */
    public double ySpeed() {
        return -1.0 * controller0.getRightX();
    }


    /** @return [-1,1] */
    public double rotSpeed() {
        return -1.0 * controller0.getLeftX();
    }

    public void driveSlow(DriveSlow command) {
        controller0.leftBumper().whileTrue(command);
    }

    public XboxController getController0() {
        return controller0.getHID();
    }

    public void resetPose(ResetPose command){
        // controller0.leftBumper().onTrue(command);
    }

    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = controller0.getHID().getPOV();

        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    public GoalOffset goalOffset() {
        double left = controller0.getLeftTriggerAxis();
        double right = controller0.getRightTriggerAxis();
        if (left > kTriggerThreshold) {
            if (right > kTriggerThreshold) {
                return GoalOffset.center;
            }
            return GoalOffset.left;
        }
        if (right > kTriggerThreshold) {
            return GoalOffset.right;
        }
        return GoalOffset.center;
    }

    

    public void defense(Defense defense) {
        JoystickButton button = new JoystickButton(controller0.getHID(), 2);

        button.whileTrue(defense);
    }

    public void rumbleOn() {
        controller0.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
        controller0.getHID().setRumble(RumbleType.kRightRumble, 0.0);
    }
    
    public void rumbleTrigger(RumbleOn command){
        controller0.a().whileTrue(command);
    }

    public void rumbleOff() {
        controller0.getHID().setRumble(RumbleType.kLeftRumble, 0);
        controller0.getHID().setRumble(RumbleType.kRightRumble, 0);

    }

    public void rotate0(Rotate command){
        JoystickButton button = new JoystickButton(controller0.getHID(), 9);
        button.whileTrue(command);
    }

    public void driveMedium(DriveMedium command){
        controller0.rightBumper().whileTrue(command);
    }

    public void moveConeWidthLeft(MoveConeWidth command){
        controller0.y().whileTrue(command);
    }

    public void moveConeWidthRight(MoveConeWidth command){
        controller0.a().whileTrue(command);
    }

    ///////////////////////////////
    //
    // OPERATOR: arm and manipulator controls

    /** @return [-1,1] */
    public double openSpeed() {
        return controller1.getRightTriggerAxis();
    }

    /** @return [-1,1] */
    public double closeSpeed() {
        return controller1.getLeftTriggerAxis();
    }

    /** @return [-1,1] */
    public double lowerSpeed() {
        return controller1.getRightX();
    }

    /** @return [-1,1] */
    public double upperSpeed() {
        return controller1.getLeftY();
    }

    public XboxController getController() {
        return controller1.getHID();
    }

    public void armHigh(ArmTrajectory command) {
        controller1.povUp().whileTrue(command);
    }

    public void armLow(ArmTrajectory command){
        controller1.povLeft().whileTrue(command);
    }

    public void armSafe(ArmTrajectory command) {
        controller1.povDown().whileTrue(command);
    }

    public void safeWaypoint(ArmTrajectory command) {
        // SequentialCommandGroup commandGroup = new SequentialCommandGroup(command, comman)
        // controller1.rightBumper().whileTrue(command);
    }

    public void armSafeSequential(ArmTrajectory command, ArmTrajectory command2) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(command, command2);
        controller1.povDown().whileTrue(commandGroup);
    }

    public void armSafeBack(ArmTrajectory command) {
        // controller1.leftBumper().whileTrue(command);
    }

    public void closeSlow(CloseSlow command) {
        // controller1.leftBumper().whileTrue(command);

        // controller1.a().whileTrue(command);


        controller1.leftBumper().whileTrue(command);
    }

    public void armSubstation(ArmTrajectory command) {
        controller1.povRight().whileTrue(command);
    }

    public void armMid(ArmTrajectory command){
        JoystickButton button = new JoystickButton(controller0.getHID(), 7);
        button.whileTrue(command);
    }

    public void open(Open command) {
        // controller1.a().whileTrue(command);
    }

    public void home(Home command) {
        controller1.b().whileTrue(command);
    }

    public void close(Close command) {
        controller1.x().whileTrue(command);
    }

    public void cubeMode(SetCubeMode command) {
        controller1.y().onTrue(command);
    }

    public void coneMode(SetConeMode command) {
        controller1.a().onTrue(command);
    }

    public void armToSub(ArmTrajectory command){
        // JoystickButton button = new JoystickButton(controller1.getHID(), 7);
        // button.onTrue(command);

        // controller1.rightBumper().whileTrue(command);
    }

    public void ledOn(LedOn command){
        // controller1.rightBumper().whileTrue(command);
    }

    public void oscillate(Oscillate command){
        controller1.rightBumper().whileTrue(command);
    }

    public void tapeDetect(DriveToRetroReflectiveTape command){
        // controller1.leftBumper().whileTrue(command);
    }

    public void armSubSafe(ArmTrajectory command){
        // controller1.rightBumper().whileTrue(command);
    }

    

    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("xbox control");
        builder.addDoubleProperty("right y", () -> controller0.getRightY(), null);
        builder.addDoubleProperty("right x", () -> controller0.getRightX(), null);
        builder.addDoubleProperty("left x", () -> controller0.getLeftX(), null);

        // builder.addDoubleProperty("x limited", () -> xLimited(), null);
        // builder.addDoubleProperty("y limtied", () -> yLimited(), null);
        // builder.addDoubleProperty("rot Limited", () -> rotLimited(), null);
    }
}
