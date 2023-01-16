// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class KAuto {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = 20;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 100;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class KModule {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 20 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 20 * Math.PI;

    public static final double kWheelDiameterMeters = 0.1016; // AndyMark Swerve & Steer has 4 inch wheel
    public static final double kDriveReduction = 6.67; // see andymark.com/products/swerve-and-steer

    public static final double kPModuleTurningController = 1;
    public static final double kPModuleDriveController = .1;
  }

  public static class KSwerve {
    public static final double kTrackWidth = 0.38;
    public static final double kWheelBase = 0.445;

    public static final double ksVolts = 2;
    public static final double kvVoltSecondsPerMeter = 2.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.5;
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = -5;

    // Order is front left, front right, back left, back right
    public static final double[] kTurningOffsets = {0.51, 0.54, 0.74, 0.74}; // Number of rotations, should be between 0 and 1
    public static final int[] kDriveMotorIDs = {11, 12, 21, 22};
    public static final int[] kTurnMotorIDs = {3, 1, 2, 0};
    public static final int[] kTurnEncoderIDs = {1, 3, 0, 2};
    public static final boolean[] kDriveEncoderReversed = {false, false, false, false};
    public static final boolean[] kTurnEncoderReversed = {false, false, false, false};
  }
}
