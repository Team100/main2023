package org.team100.frc2023.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static org.team100.lib.control.ControlUtil.expo;

import org.team100.lib.commands.Command100;

import static org.team100.lib.control.ControlUtil.deadband;

public class VKBControl implements Control {

    private final CommandJoystick m_controller;
    private final CommandJoystick m_controllerRotate;
    private final CommandXboxController controller1;
    private Rotation2d previousRotation = new Rotation2d(0);
    private double ticks = 0;
    private final double kDegreesPerTick = 0;
    private final double kRadiansPerTick = 0;
    private double translateMultipler = 1;
    private double rotationMultiplier = 1;

    public static class Config {
        public static int kTriggerSoftChannel = 1;
        public static int kTriggerHardChannel = 2;
        public static int kBigRedButtonChannel = 3;
        public static int kHighGreyButtonChannel = 4;
        public static int kMidGreyButtonChannel = 5;
        public static int kLowGreyButtonChannel = 6;
        public static int kF1ButtonChannel = 7;
        public static int kF2ButtonChannel = 8;
        public static int kF3ButtonChannel = 9;
        public static int kSw1UpChannel = 10;
        public static int kSw1DownChannel = 11;
        public static int kEn1IncChannel = 12;
        public static int kEn1DecChannel = 13;

    }

    public VKBControl() {
        m_controller = new CommandJoystick(0);
        System.out.printf("Controller0: %s\n", m_controller.getHID().getName());

        m_controllerRotate = new CommandJoystick(1);
        System.out.printf("Controller0: %s\n", m_controller.getHID().getName());

        controller1 = new CommandXboxController(2);
        EventLoop loop = CommandScheduler.getInstance().getActiveButtonLoop();
        loop.bind(updateTicks());
        loop.bind(updateScaleFactor());
    }

    @Override
    public Twist2d twist() {

        double dx = expo(-m_controller.getY(), .5) * translateMultipler;
        double dy = expo(-m_controller.getX(), .5) * translateMultipler;
        // double dtheta = expo(m_controller.getRawAxis(5), .5) * rotationMultiplier;
        double dtheta = expo(-m_controllerRotate.getRawAxis(0), .5) * rotationMultiplier;
        // dtheta = -m_controllerRotate.getRawAxis(0) * rotationMultiplier;

        // System.out.println("RAW AXIS" + m_controllerRotate.getRawAxis(1));

        return new Twist2d(dx, dy, dtheta);
    }

    private Runnable updateScaleFactor() {
        return new Runnable() {

            @Override
            public void run() {
                boolean mediumButton = button(Config.kTriggerSoftChannel).getAsBoolean();
                boolean slowButton = button(Config.kTriggerHardChannel).getAsBoolean();

                if (slowButton) {
                    translateMultipler = 0.2;
                    rotationMultiplier = 0.2;
                } else if (mediumButton) {
                    translateMultipler = 0.5;
                    rotationMultiplier = 0.5;
                } else {
                    translateMultipler = 1;
                    rotationMultiplier = 1;
                }
            }
        };
    }

    @Override
    public Rotation2d desiredRotation() {
        double desiredAngleDegrees = m_controller.getHID().getPOV(1);
        System.out.println(desiredAngleDegrees);
        if (desiredAngleDegrees < 0) {
            return null;
        }
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
        return previousRotation;
    }

    public double getEn1Raw() {
        return ticks;
    }

    public double getEn1Radians() {
        return MathUtil.angleModulus(ticks * kRadiansPerTick);
    }

    public double getEn1Degrees() {
        return ticks * kDegreesPerTick - 360 * Math.floor(ticks * kDegreesPerTick / 360);
    }

    @Override
    public void resetRotation0(Command command) {
        button(Config.kF2ButtonChannel).onTrue(command);
    }

    @Override
    public void resetRotation180(Command command) {
        button(Config.kF3ButtonChannel).onTrue(command);
    }

    @Override
    public void defense(Command command) {
        button(Config.kLowGreyButtonChannel).whileTrue(command);
    }

    @Override
    public void driveMedium(Command command) {
        button(1).whileTrue(command.unless(() -> button(2).getAsBoolean()));
    }

    @Override
    public void driveSlow(Command command) {
        button(2).whileTrue(command);
    }

    private Runnable updateTicks() {
        return new Runnable() {
            private boolean m_incLast = button(Config.kEn1IncChannel).getAsBoolean();
            private boolean m_decLast = button(Config.kEn1DecChannel).getAsBoolean();

            @Override
            public void run() {
                boolean inc = button(Config.kEn1IncChannel).getAsBoolean();
                boolean dec = button(Config.kEn1DecChannel).getAsBoolean();
                if (!m_incLast && inc) {
                    ticks++;
                }
                if (!m_decLast && dec) {
                    ticks--;
                }
                m_incLast = inc;
                m_decLast = dec;
            }
        };
    }

    private JoystickButton button(int button) {
        return new JoystickButton(m_controller.getHID(), button);
    }

    ///////////////////////////////
    //
    // OPERATOR: arm and manipulator controls

    /** @return [-1,1] */
    @Override
    public double openSpeed() {
        return controller1.getRightTriggerAxis();
    }

    /** @return [-1,1] */
    @Override
    public double closeSpeed() {
        return controller1.getLeftTriggerAxis();
    }

    /** @return [-1,1] */
    @Override
    public double lowerSpeed() {
        return deadband(controller1.getRightX(), 0.2, 1.0);
        // return controller1.getLeftX();
    }

    /** @return [-1,1] */
    @Override
    public double upperSpeed() {
        return deadband(controller1.getLeftY(), 0.2, 1.0);
        // return controller1.getLeftY();
    }

    @Override
    public void armHigh(Command command) {
        controller1.povUp().whileTrue(command);
    }

    @Override
    public void armLow(Command command) {
        controller1.povLeft().whileTrue(command);
    }

    @Override
    public void armSafe(Command command) {
        controller1.povDown().whileTrue(command);
    }

    @Override
    public void safeWaypoint(Command command) {
        // SequentialCommandGroup commandGroup = new SequentialCommandGroup(command,
        // comman)
        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void armSafeSequential(Command command, Command command2) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(command, command2);
        controller1.povDown().whileTrue(commandGroup);
    }

    @Override
    public void armSafeBack(Command command) {
        // controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void closeSlow(Command command) {
        // controller1.leftBumper().whileTrue(command);

        // controller1.a().whileTrue(command);

        controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void armSubstation(Command command) {
        controller1.povRight().whileTrue(command);
    }

    @Override
    public void armMid(Command command) {
        JoystickButton button = new JoystickButton(controller1.getHID(), 7);
        button.whileTrue(command);
    }

    @Override
    public void open(Command command) {
        controller1.b().whileTrue(command);
    }

    @Override
    public void home(Command command) {
        // controller1.b().whileTrue(command);
    }

    @Override
    public void close(Command command) {
        controller1.x().whileTrue(command);
    }

    @Override
    public void cubeMode(Command command) {
        controller1.y().onTrue(command);
    }

    @Override
    public void coneMode(Command command) {
        controller1.a().onTrue(command);
    }

    @Override
    public void armToSub(Command command) {
        // JoystickButton button = new JoystickButton(controller1.getHID(), 7);
        // button.onTrue(command);

        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void ledOn(Command command) {
        // controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void oscillate(Command command) {
        controller1.rightBumper().whileTrue(command);
    }

    @Override
    public void hold(Command command) {
        controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void tapeDetect(Command command) {
        // controller1.leftBumper().whileTrue(command);
    }

    @Override
    public void armSubSafe(Command command) {
        // controller1.rightBumper().whileTrue(command);
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    // builder.setSmartDashboardType("xbox control");
    // builder.addDoubleProperty("right y", () -> m_controller.getRightY(), null);
    // builder.addDoubleProperty("right x", () -> m_controller.getRightX(), null);
    // builder.addDoubleProperty("left x", () -> m_controller.getLeftX(), null);
    // }

    @Override
    public double armX() {
        // TODO: wire this up
        // return 0.2 * deadband(-1.0 * controller1.getLeftY(), 0.15, 1.0);
        return 0;
    }

    @Override
    public double armY() {
        // TODO: wire this up
        // return 0.2 * deadband(controller1.getRightX(), 0.15, 1.0);
        return 0;
    }

}