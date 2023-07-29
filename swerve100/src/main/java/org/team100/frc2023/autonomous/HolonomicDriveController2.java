package org.team100.frc2023.autonomous;

import org.team100.lib.sensors.RedundantGyro;
import org.team100.lib.subsystems.VeeringCorrection;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HolonomicDriveController2 {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private final RedundantGyro m_gyro;
    private final VeeringCorrection m_veering;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    private boolean m_firstRun = true;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DoublePublisher xFFPublisher = inst.getTable("Holonomic2").getDoubleTopic("xFF").publish();
    DoublePublisher xFBPublisher = inst.getTable("Holonomic2").getDoubleTopic("xFB").publish();
    DoublePublisher yFFPublisher = inst.getTable("Holonomic2").getDoubleTopic("yFF").publish();
    DoublePublisher yFBPublisher = inst.getTable("Holonomic2").getDoubleTopic("yFB").publish();

    public HolonomicDriveController2(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            RedundantGyro gyro) {
        m_gyro = gyro;
        m_veering = new VeeringCorrection(m_gyro);
        m_xController = xController;
        m_yController = yController;
        m_thetaController = thetaController;
        m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
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

    public ChassisSpeeds calculate(
            Pose2d currentPose,
            Trajectory.State desiredState,
            Rotation2d desiredHeading) {

        Pose2d trajectoryPose = desiredState.poseMeters;
        double desiredLinearVelocityMetersPerSecond = desiredState.velocityMetersPerSecond;

        if (m_firstRun) {
            m_thetaController.reset(currentPose.getRotation().getRadians());
            m_firstRun = false;
        }

        double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();

        double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();

        double thetaFF = m_thetaController.calculate(
                currentPose.getRotation().getRadians(), desiredHeading.getRadians());

        xFFPublisher.set(xFF);
        yFFPublisher.set(yFF);

        m_poseError = trajectoryPose.relativeTo(currentPose);
        m_rotationError = desiredHeading.minus(currentPose.getRotation());

        double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());

        xFBPublisher.set(xFeedback);
        yFBPublisher.set(yFeedback);
        Rotation2d rotation2 = m_veering.correct(currentPose.getRotation());
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback, yFF + yFeedback, thetaFF, rotation2);
    }
}
