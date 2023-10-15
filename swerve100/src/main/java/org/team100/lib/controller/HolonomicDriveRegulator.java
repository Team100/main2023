package org.team100.lib.controller;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.system.SubRegulator1D;
import org.team100.lib.system.examples.ControllerCartesian1D;
import org.team100.lib.system.examples.ControllerRotary1D;
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

    final Vector<N2> stateTolerance_x = VecBuilder.fill(0.02, 0.02);
    final Vector<N1> controlTolerance_x = VecBuilder.fill(.001);
    private WhiteNoiseVector<N2> wx = WhiteNoiseVector.noise2(0, 0);
    private MeasurementUncertainty<N2> vx = MeasurementUncertainty.for2(0.01, 0.1);
    private ControllerCartesian1D system_x = new ControllerCartesian1D(wx, vx, kDt);
    private SubRegulator1D xRegulator = new SubRegulator1D(system_x, stateTolerance_x, controlTolerance_x);

    final Vector<N2> stateTolerance_y = VecBuilder.fill(0.02, 0.02);
    final Vector<N1> controlTolerance_y = VecBuilder.fill(.001);
    private WhiteNoiseVector<N2> wy = WhiteNoiseVector.noise2(0, 0);
    private MeasurementUncertainty<N2> vy = MeasurementUncertainty.for2(0.01, 0.1);
    private ControllerCartesian1D system_y = new ControllerCartesian1D(wy, vy, kDt);
    private SubRegulator1D yRegulator = new SubRegulator1D(system_y, stateTolerance_y, controlTolerance_y);

    final Vector<N2> stateTolerance_theta = VecBuilder.fill(Math.PI/120, Math.PI/120);
    final Vector<N1> controlTolerance_theta = VecBuilder.fill(.001);
    private WhiteNoiseVector<N2> wtheta = WhiteNoiseVector.noise2(0, 0);
    private MeasurementUncertainty<N2> vtheta = MeasurementUncertainty.for2(0.1, 0.1);
    private ControllerRotary1D system_theta = new ControllerRotary1D(wtheta, vtheta, kDt);;
    private SubRegulator1D thetaRegulator = new SubRegulator1D(system_theta, stateTolerance_theta,
            controlTolerance_theta);

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

    public HolonomicDriveRegulator() {
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    // TODO: fix this;
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
                SwerveState newDesiredState = this.optimize(desiredState, currentPose.getRotation());
        xhat_x = xRegulator.correctPosition(xhat_x, currentPose.getX());
        xhat_y = yRegulator.correctPosition(xhat_y, currentPose.getY());
        xhat_theta = thetaRegulator.correctPosition(xhat_theta, currentPose.getRotation().getRadians());

        Vector<N2> setpoint_x = xRegulator.getR(newDesiredState.x());
        Vector<N2> setpoint_y = yRegulator.getR(newDesiredState.y());
        Vector<N2> setpoint_theta = thetaRegulator.getR(newDesiredState.theta());

        Vector<N2> rDot_x = xRegulator.getRDot(newDesiredState.x());
        Vector<N2> rDot_y = yRegulator.getRDot(newDesiredState.y());
        Vector<N2> rDot_theta = thetaRegulator.getRDot(newDesiredState.theta());

        Matrix<N1, N1> totalU_x = xRegulator.calculateTotalU(xhat_x, setpoint_x, rDot_x, kDt);
        Matrix<N1, N1> totalU_y = xRegulator.calculateTotalU(xhat_y, setpoint_y, rDot_y, kDt);
        Matrix<N1, N1> totalU_theta = xRegulator.calculateTotalU(xhat_theta, setpoint_theta, rDot_theta, kDt);

        xhat_x = xRegulator.predictState(xhat_x, totalU_x, kDt);
        xhat_y = yRegulator.predictState(xhat_y, totalU_y, kDt);
        xhat_theta = thetaRegulator.predictState(xhat_theta, totalU_theta, kDt);

        return new Twist2d(totalU_x.get(0, 0), totalU_y.get(0, 0), totalU_theta.get(0, 0));
    }

    public SwerveState optimize(SwerveState desiredState, Rotation2d currentAngle) {
        double delta = desiredState.theta().x()-currentAngle.getRadians();
        if (Math.abs(delta) > Math.PI/2) {
            return new SwerveState(
                desiredState.x(),
                desiredState.y(),
                new State100(desiredState.theta().x()+Math.PI,desiredState.theta().v(),desiredState.theta().a()));
        } else {
            return desiredState;
        }
    }

}
