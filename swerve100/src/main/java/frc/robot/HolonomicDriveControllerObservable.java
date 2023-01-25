// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class HolonomicDriveControllerObservable implements Sendable{
  public Pose2d m_poseError = new Pose2d();
  public Rotation2d m_rotationError = new Rotation2d();
  public Pose2d m_poseTolerance = new Pose2d();
  public boolean m_enabled = true;

  public final PIDController m_xController;
  public final PIDController m_yController;
  public final ProfiledPIDController m_thetaController;

  public boolean m_firstRun = true;
  
  public double xFF;
  public double yFF;
  public double thetaFF;
  public double xFeedback;
  public double yFeedback;
  public Rotation2d m_desiredHeading;
  public Pose2d m_currentPose;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  public HolonomicDriveControllerObservable(
      PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
    SmartDashboard.putData("HolonomicDriveController", this);
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param trajectoryPose The desired trajectory pose, as sampled for the current timestep.
   * @param desiredLinearVelocityMetersPerSecond The desired linear velocity.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredHeading) {
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.
    m_currentPose = currentPose;
    m_desiredHeading = desiredHeading;
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_firstRun = false;
    }

    // Calculate feedforward velocities (field-relative).
    xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    thetaFF =
        m_thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    m_poseError = trajectoryPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    // Calculate feedback velocities (based on position error).
    xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
    yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param desiredState The desired trajectory pose, as sampled for the current timestep.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
  public double getxFF() {
    return xFF;
  }
  public double getyFF() {
    return yFF;
  }
  public double getthetaFF() {
    return thetaFF;
  }
  public double getxFeedback() {
    return xFeedback;
  }
  public double getyFeedback() {
    return yFeedback;
  }
  public double getCurrentRotation() {
    if (m_currentPose != null) {
      return m_currentPose.getRotation().getRadians();
    }
    return 0;
  }
  public double getDesiredHeading() {
    if (m_desiredHeading != null) {
      return m_desiredHeading.getRadians();
    }
    return 0;
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HolonomicDriveController");
    builder.addDoubleProperty("xController error", () -> m_xController.getPositionError(), null);
    builder.addDoubleProperty("yController error", () -> m_yController.getPositionError(), null);
    builder.addDoubleProperty("thetaController error", () -> m_thetaController.getPositionError(), null);
    builder.addDoubleProperty("xFF", () -> getxFF(), null);
    builder.addDoubleProperty("yFF", () -> getyFF(), null);
    builder.addDoubleProperty("thetaFF", () -> getthetaFF(), null);
    builder.addDoubleProperty("xFeedback", () -> getxFeedback(), null);
    builder.addDoubleProperty("yFeedback", () -> getyFeedback(), null);
    builder.addDoubleProperty("currentRotation", () -> getCurrentRotation(), null);
    builder.addDoubleProperty("desiredHeading", () -> getDesiredHeading(), null);
  }
}
