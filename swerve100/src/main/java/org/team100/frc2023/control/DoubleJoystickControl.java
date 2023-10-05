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

import java.net.http.HttpResponse.ResponseInfo;

import javax.xml.crypto.dsig.spec.C14NMethodParameterSpec;

import org.opencv.core.Mat;
import org.team100.lib.commands.Command100;

import com.fasterxml.jackson.databind.cfg.ConfigFeature;

import static org.team100.lib.control.ControlUtil.deadband;

public class DoubleJoystickControl implements Control {

  private final CommandJoystick m_controller;
  private final CommandJoystick m_controllerRotate;
  private final CommandXboxController controller1;
  private Rotation2d previousRotation = new Rotation2d(0);
  private double ticks = 0;
  private final double kDegreesPerTick = 0;
  private final double kRadiansPerTick = 0;
  private double translateMultipler = 1;
  private double rotationMultiplier = 1;
  private Boolean forwardPOVActive = true;
  private Double desiredAngle = 0.0;
  private boolean operatorOverride = false;
  private double override = 1;


  // With snap toggle, when you press it, one of them has to be the default

  public static class Config {

    static boolean snapToggle = false;

    public static class Translate {
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

    public static class Rotate {
      public static int kTrigger = 1;
      public static int kleftTop = 4;
      public static int kRightTop = 5;
      public static int kFrontTop = 3;
      public static int kBackTop = 2;

    }

  }

  public DoubleJoystickControl() {
    m_controller = new CommandJoystick(0);
    System.out.printf("Controller0: %s\n", m_controller.getHID().getName());

    m_controllerRotate = new CommandJoystick(1);
    System.out.printf("Controller0: %s\n", m_controller.getHID().getName());

    controller1 = new CommandXboxController(2);
    EventLoop loop = CommandScheduler.getInstance().getActiveButtonLoop();
    loop.bind(updateTicks());
    loop.bind(updateScaleFactor());
    // loop.bind(setDesiredAngleDegrees());
    loop.bind(setDesiredPOV());
    loop.bind(override());
  }

  @Override
  public Twist2d twist() {
    

    if(!operatorOverride){
      double dx = expo(-m_controller.getY(), .5) * translateMultipler;
      double dy = expo(-m_controller.getX(), .5) * translateMultipler;
      // double dtheta = expo(m_controller.getRawAxis(5), .5) * rotationMultiplier;
      double dtheta = expo(deadband(-m_controllerRotate.getRawAxis(0), 0.15, 1), .5) * rotationMultiplier;
      // dtheta = -m_controllerRotate.getRawAxis(0) * rotationMultiplier;
      
      // System.out.println("RAW AXIS" + m_controllerRotate.getRawAxis(1));
  
      return new Twist2d(dx, dy, dtheta);
    } else {
      double dx = expo(-controller1.getLeftY(), .5) * 0.1;
      double dy = expo(-controller1.getLeftX(), .5) * 0.1;
      double dtheta = expo(-controller1.getRightX(), .5) * 0.1;

      return new Twist2d(dx, dy, dtheta);

    }

    
  }

  private Runnable updateScaleFactor() {
    return new Runnable() {

      @Override
      public void run() {
        boolean mediumButton = button(Config.Translate.kTriggerSoftChannel).getAsBoolean();
        boolean slowButton = button(Config.Translate.kTriggerHardChannel).getAsBoolean();

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

    if (Config.snapToggle) {
      Double desiredAngleDegrees = null;

      if (forwardPOVActive) {
        desiredAngleDegrees = Math.PI / 2;
      } else if (!forwardPOVActive) {
        desiredAngleDegrees = 3 * Math.PI / 2;
      } else {
        desiredAngleDegrees = -1.0;
      }

      if (desiredAngleDegrees < 0 && desiredAngleDegrees > -179) {
        return null;
      }
      previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);
      return previousRotation;
    } else {
      // double desiredAngleDegrees = m_controller.getHID().getPOV(1);
      double desiredAngleDegrees = desiredAngle;

      // System.out.println("DESIRED ANGLE DEG " + desiredAngleDegrees);

      if (desiredAngleDegrees < 0.0 && desiredAngleDegrees > -179.0) {
        // System.out.println(desiredAngleDegrees);
        return null;
      }

      if (desiredAngleDegrees == -180.0) {
        previousRotation = Rotation2d.fromDegrees(desiredAngleDegrees);

      } else {
        previousRotation = Rotation2d.fromDegrees(-1.0 * desiredAngleDegrees);

      }
      return previousRotation;

    }

  }

  public void getDesiredTrigger() {
    button(Config.Rotate.kleftTop, m_controllerRotate).onTrue((Command) setDesiredAngleDegrees());

  }

  public Command setDesiredAngleDegrees() {
    return new Command() {

      @Override
      public void initialize() {

        // if(triggerPressed){
        if (forwardPOVActive) {
          forwardPOVActive = false;
        } else if (!forwardPOVActive) {
          forwardPOVActive = true;
        }
        // }

        System.out.println("AHHHHHHHHHHHHHHH");
      }
    };

  }

  public Runnable setDesiredPOV() {
    return new Runnable() {

      @Override
      public void run() {

        boolean leftPOV = button(Config.Rotate.kleftTop, m_controllerRotate).getAsBoolean();
        boolean rightPOV = button(Config.Rotate.kRightTop, m_controllerRotate).getAsBoolean();
        boolean frontPOV = button(Config.Rotate.kFrontTop, m_controllerRotate).getAsBoolean();
        boolean backPOV = button(Config.Rotate.kBackTop, m_controllerRotate).getAsBoolean();

        if (frontPOV) {
          desiredAngle = 0.0;
        } else if (backPOV) {
          desiredAngle = -180.0;
        } else {
          desiredAngle = -1.0;
        }

      }
    };

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
    button(Config.Translate.kBigRedButtonChannel).onTrue(command);
  }

  @Override
  public void resetRotation180(Command command) {
    button(Config.Translate.kHighGreyButtonChannel).onTrue(command);
  }

  @Override
  public void defense(Command command) {
    button(Config.Translate.kLowGreyButtonChannel).whileTrue(command);
  }

  @Override
  public void driveMedium(Command command) {
    button(1).whileTrue(command.unless(() -> button(2).getAsBoolean()));
  }

  @Override
  public void driveSlow(Command command) {
    button(2).whileTrue(command);
  }

  private Runnable override() {
    return new Runnable() {
      @Override
      public void run() {
        if (controller1.getRightTriggerAxis() > 0.5) {
          operatorOverride = true;
          override = 0;
        } else {
          operatorOverride = false;
          override = 1;
        }
      }
    };
  }

  private Runnable updateTicks() {
    return new Runnable() {
      private boolean m_incLast = button(Config.Translate.kEn1IncChannel).getAsBoolean();
      private boolean m_decLast = button(Config.Translate.kEn1DecChannel).getAsBoolean();

      @Override
      public void run() {
        boolean inc = button(Config.Translate.kEn1IncChannel).getAsBoolean();
        boolean dec = button(Config.Translate.kEn1DecChannel).getAsBoolean();
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

  private JoystickButton button(int button, CommandJoystick joystick) {
    return new JoystickButton(joystick.getHID(), button);
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
    return deadband(controller1.getRightX(), 0.2, 1.0) * override;
    // return controller1.getLeftX();
  }

  /** @return [-1,1] */
  @Override
  public double upperSpeed() {
    return deadband(controller1.getLeftY(), 0.2, 1.0) * override;
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