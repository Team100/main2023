package org.team100.lib.controller;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.system.SubRegulator1D;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class HolonomicDriveRegulator {
    private static final double kDt = 0.02;
    SubRegulator1D xRegulator;
    SubRegulator1D yRegulator;
    SubRegulator1D thetaRegulator;

    
    Variance<N2> px = Variance.from2StdDev(.01, .01);
    Variance<N2> py = Variance.from2StdDev(.01, .01);
    Variance<N2> ptheta = Variance.from2StdDev(.316228, .316228);

    RandomVector<N2> xhat_x = new RandomVector<>(VecBuilder.fill(0, 0), px);
    RandomVector<N2> xhat_y = new RandomVector<>(VecBuilder.fill(0, 0), py);
    RandomVector<N2> xhat_theta = new AngularRandomVector<>(VecBuilder.fill(0, 0), ptheta);

    private double xErr = 0;
    private double yErr = 0;
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();

    public HolonomicDriveRegulator(
           SubRegulator1D xRegulator,
           SubRegulator1D yRegulator,
           SubRegulator1D thetaRegulator) {
        this.xRegulator = xRegulator;
        this.yRegulator = yRegulator;
        this.thetaRegulator = thetaRegulator;
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        // final var eTranslate = m_poseError.getTranslation();
        final var eRotate = m_rotationError;
        final var tolTranslate = m_poseTolerance.getTranslation();
        final var tolRotate = m_poseTolerance.getRotation();
        return Math.abs(xErr) < tolTranslate.getX()
                && Math.abs(yErr) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    /**
     * TODO make currentPose a state as well.
     * 
     * @param currentPose  robot's current pose in field coordinates
     * @param desiredState reference state
     * @return field-relative twist, meters and radians per second
     */
    public Twist2d calculate(
            Pose2d currentPose,
            SwerveState desiredState) {
        xhat_x = xRegulator.correctPosition(xhat_x, currentPose.getX());
        xhat_y = yRegulator.correctPosition(xhat_y, currentPose.getY());
        xhat_theta = thetaRegulator.correctPosition(xhat_theta, currentPose.getRotation().getRadians());

        Vector<N2> setpoint_x = xRegulator.getR(desiredState.x());
        Vector<N2> setpoint_y = yRegulator.getR(desiredState.y());
        Vector<N2> setpoint_theta = thetaRegulator.getR(desiredState.theta());

        Vector<N2> rDot_x = xRegulator.getRDot(desiredState.x());
        Vector<N2> rDot_y = yRegulator.getRDot(desiredState.y());
        Vector<N2> rDot_theta = thetaRegulator.getRDot(desiredState.theta());

        Matrix<N1, N1> totalU_x = xRegulator.calculateTotalU(xhat_x, setpoint_x, rDot_x, kDt);
        Matrix<N1, N1> totalU_y = xRegulator.calculateTotalU(xhat_y, setpoint_y, rDot_y, kDt);
        Matrix<N1, N1> totalU_theta = xRegulator.calculateTotalU(xhat_theta, setpoint_theta, rDot_theta, kDt);

        xhat_x = xRegulator.predictState(xhat_x, totalU_x, kDt);
        xhat_y = yRegulator.predictState(xhat_y, totalU_y, kDt);
        xhat_theta = thetaRegulator.predictState(xhat_theta, totalU_theta, kDt);

        return new Twist2d(totalU_x.get(0, 0), totalU_y.get(0, 0), totalU_theta.get(0, 0));
    }
}
