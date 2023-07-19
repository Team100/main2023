package org.team100.frc2023.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HolonomicDriveController2 {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    private boolean m_firstRun = true;

    public HolonomicDriveController2(
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController) {
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

    /**
     * TODO: combine the cartesian and heading parts of the reference state.
     * 
     * @param currentPose    robot's current pose in field coordinates
     * @param desiredState   cartesian part of the reference
     * @param desiredHeading heading part of the reference
     * @return field-relative twist, meters and radians per second
     */
    public Twist2d calculate(
            Pose2d currentPose,
            Trajectory.State desiredState,
            Rotation2d desiredHeading) {

        Pose2d trajectoryPose = desiredState.poseMeters;
        double desiredLinearVelocityMetersPerSecond = desiredState.velocityMetersPerSecond;

        Rotation2d currentRotation = currentPose.getRotation();
        if (m_firstRun) {
            m_thetaController.reset(currentRotation.getRadians());
            m_firstRun = false;
        }

        double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
        double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
      
        double thetaFF = m_thetaController.calculate(currentRotation.getRadians(), desiredHeading.getRadians());
        xFFPublisher.set(xFF);
        yFFPublisher.set(yFF);
        rotSetpoint.set(m_thetaController.getSetpoint().position);

        m_poseError = trajectoryPose.relativeTo(currentPose);
        m_rotationError = desiredHeading.minus(currentRotation);

        double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
        double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());
        poseXErrorPublisher.set(m_xController.getPositionError());
        poseYErrorPublisher.set(m_yController.getPositionError());
        xFBPublisher.set(xFeedback);
        yFBPublisher.set(yFeedback);

        return new Twist2d(xFF + xFeedback, yFF + yFeedback, thetaFF);
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Holonomic2");
    private final DoublePublisher xFFPublisher = table.getDoubleTopic("xFF").publish();
    private final DoublePublisher xFBPublisher = table.getDoubleTopic("xFB").publish();
    private final DoublePublisher yFFPublisher = table.getDoubleTopic("yFF").publish();
    private final DoublePublisher yFBPublisher = table.getDoubleTopic("yFB").publish();
    private final DoublePublisher poseXErrorPublisher = table.getDoubleTopic("xErr").publish();
    private final DoublePublisher poseYErrorPublisher = table.getDoubleTopic("yErr").publish();
    private final DoublePublisher rotSetpoint = table.getDoubleTopic("Rot Setpoint").publish();

}
