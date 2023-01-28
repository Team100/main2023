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

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = 10;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 100;

    public static final double kPXController = .15;//.1;
    public static final double kPYController = 0.15 ;//.1;
    public static final double kPThetaController = .45;

    public static final double kIXController = 0;
    public static final double kIYController = 0;
    public static final double kIThetaController = 0;

    public static final double kDXController = 0;
    public static final double kDYController = 0;
    public static final double kDThetaController = .775;//.4;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }


  // swerve 2
  // public static final class SwerveConstants {
  //   public static final double FRONT_LEFT_TURNING_OFFSET = .662;
  //   public static final double FRONT_RIGHT_TURNING_OFFSET = .543;
  //   public static final double REAR_LEFT_TURNING_OFFSET = .745;
  //   public static final double REAR_RIGHT_TURNING_OFFSET = .75;
  //   public static final double kTrackWidth = 0.38;
  //   public static final double kWheelBase = 0.445;
  // }

  // swerve1

  public static final class SwerveConstants {


    public static final double FRONT_LEFT_TURNING_OFFSET = .70;
    public static final double FRONT_RIGHT_TURNING_OFFSET = .71;
    public static final double REAR_LEFT_TURNING_OFFSET = .37;
    public static final double REAR_RIGHT_TURNING_OFFSET = .98;

    public static final double kTrackWidth = 0.449;
    public static final double kWheelBase = 0.464;

    public static final int FRONT_LEFT_TURNING_MOTOR = 0;
    public static final int FRONT_RIGHT_TURNING_MOTOR = 2;
    public static final int REAR_LEFT_TURNING_MOTOR = 1;
    public static final int REAR_RIGHT_TURNING_MOTOR = 3;




  }


}
