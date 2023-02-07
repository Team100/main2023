// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class AutoConstants {
        // public static final double kSpeedModifier = 2;
        public static final double kMaxSpeedMetersPerSecond = 4;//4; //4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;//1; //.5;

        // pbublic static final double kMaxSpeedMetersPerSecond = 8;//4; //4;
        // public static final double kMaxAccelerationMetersPerSecondSquared = 3;//1; //.5;

        public static final double kMaxAngularSpeedRadiansPerSecond = 10;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 100;

        public static final double kPXController = .15;// .15;
        public static final double kPYController = 0.15;// .1;
        // public static final double kPThetaController = .5 / Math.pow(2, kSpeedModifier); //.125 //.45
        public static final double kPThetaController = 2; //.125 //.45

        public static final double kIXController = 0;
        public static final double kIYController = 0;
        public static final double kIThetaController = 0;

        public static final double kDXController = 0;
        public static final double kDYController = 0;
        public static final double kDThetaController = 0; //.775;// .4;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);



        public static final TrapezoidProfile.Constraints kXControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

        public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);

        public static final TrajectoryConfig kTrajectoryConfigSlow = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
    }

    // Constraint for the motion profiled robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
    //     new TrapezoidProfile.Constraints(
    //         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  


  // swerve 2
  // public static final class SwerveConstants {
  //   //public static final double FRONT_LEFT_TURNING_OFFSET = .86;
  //   //public static final double FRONT_RIGHT_TURNING_OFFSET = .55;
  //   //public static final double REAR_LEFT_TURNING_OFFSET = .75;
  //   //public static final double REAR_RIGHT_TURNING_OFFSET = .75;

  //   public static final double FRONT_LEFT_TURNING_OFFSET = 0.012360;
  //   public static final double FRONT_RIGHT_TURNING_OFFSET = 0.543194;
  //   public static final double REAR_LEFT_TURNING_OFFSET = 0.744816;
  //   public static final double REAR_RIGHT_TURNING_OFFSET = 0.750468;
  //   public static final double kTrackWidth = 0.38;
  //   public static final double kWheelBase = 0.445;
  // }

  // swerve1

  // public static final class SwerveConstants {
  //   public static final double FRONT_LEFT_TURNING_OFFSET = .984;
  //   public static final double FRONT_RIGHT_TURNING_OFFSET = .873 - 0.5;
  //   public static final double REAR_LEFT_TURNING_OFFSET = .719;
  //   public static final double REAR_RIGHT_TURNING_OFFSET = .688;
  //   public static final double kTrackWidth = 0.449;
  //   public static final double kWheelBase = 0.464;
  // }

  // squarebot
  public static final class SwerveConstants {

    public static final double FRONT_LEFT_TURNING_OFFSET = .812;
    public static final double FRONT_RIGHT_TURNING_OFFSET = .382;
    public static final double REAR_LEFT_TURNING_OFFSET = .172;
    public static final double REAR_RIGHT_TURNING_OFFSET = .789;
    public static final double kTrackWidth = 0.65;
    public static final double kWheelBase = 0.65;
  }

  public static final class swerveMeasurments {
    
  }

  // swerve 2
  // public static final class SwerveConstants {
  //   //public static final double FRONT_LEFT_TURNING_OFFSET = .86;
  //   //public static final double FRONT_RIGHT_TURNING_OFFSET = .55;
  //   //public static final double REAR_LEFT_TURNING_OFFSET = .75;
  //   //public static final double REAR_RIGHT_TURNING_OFFSET = .75;

  //   public static final double FRONT_LEFT_TURNING_OFFSET = 0.012360;
  //   public static final double FRONT_RIGHT_TURNING_OFFSET = 0.543194;
  //   public static final double REAR_LEFT_TURNING_OFFSET = 0.744816;
  //   public static final double REAR_RIGHT_TURNING_OFFSET = 0.750468;
  //   public static final double kTrackWidth = 0.38;
  //   public static final double kWheelBase = 0.445;
  // }

    // swerve1

    // public static final class SwerveConstants {

    // public static final double FRONT_LEFT_TURNING_OFFSET = 0.69;//.69;
    // public static final double FRONT_RIGHT_TURNING_OFFSET = 0.71; //.7;
    // public static final double REAR_LEFT_TURNING_OFFSET = 0.36; //36;
    // public static final double REAR_RIGHT_TURNING_OFFSET = 0.98; //.99;
    // public static final double kTrackWidth = 0.449;
    // public static final double kWheelBase = 0.464;
    // }

    // public static final class swerveMeasurments {

    // }

}
