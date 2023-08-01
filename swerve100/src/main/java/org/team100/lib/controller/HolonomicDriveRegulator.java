package org.team100.lib.controller;

import org.opencv.core.Point;
import org.team100.lib.estimator.ExtrapolatingEstimator;
import org.team100.lib.estimator.PointEstimator;
import org.team100.lib.fusion.LinearPooling;
import org.team100.lib.fusion.VarianceWeightedLinearPooling;
import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;
import org.team100.lib.motion.drivetrain.SwerveState;
import org.team100.lib.system.NonlinearSystemLoop;
import org.team100.lib.system.examples.FrictionCartesian1D;
import org.team100.lib.system.examples.FrictionRotary1D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HolonomicDriveRegulator {
    private static final double kDt = 0.02;

    final Vector<N2> xTolerance = VecBuilder.fill(0.01, 0.2);
    final Vector<N2> yTolerance = VecBuilder.fill(0.01, 0.2);
    final Vector<N2> thetaTolerance = VecBuilder.fill(0.01, 0.2);
    final Vector<N1> controlTolerance = VecBuilder.fill(12.0);

    WhiteNoiseVector<N2> w_translation = WhiteNoiseVector.noise2(0, 0);
    WhiteNoiseVector<N2> w_theta = WhiteNoiseVector.noise2(0, 0);

    MeasurementUncertainty<N2> v_translation = MeasurementUncertainty.for2(.1, .1);
    MeasurementUncertainty<N2> v_theta = MeasurementUncertainty.for2(.1, .1);

    FrictionCartesian1D x_system = new FrictionCartesian1D(w_translation, v_translation);
    FrictionCartesian1D y_system = new FrictionCartesian1D(w_translation, v_translation);
    FrictionRotary1D theta_system = new FrictionRotary1D(w_theta, v_theta);

    GainCalculator<N2, N1, N2> gcx = new GainCalculator<>(x_system, xTolerance, controlTolerance, kDt);
    GainCalculator<N2, N1, N2> gcy = new GainCalculator<>(y_system, yTolerance, controlTolerance, kDt);
    GainCalculator<N2, N1, N2> gctheta = new GainCalculator<>(theta_system, thetaTolerance, controlTolerance, kDt);

    Matrix<N1, N2> Kx = gcx.getK();
    Matrix<N1, N2> Ky = gcy.getK();
    Matrix<N1, N2> Ktheta = gctheta.getK();

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController;

    FeedbackControl<N2, N1, N2> xController = new FeedbackControl<>(x_system, Kx);
    FeedbackControl<N2, N1, N2> yController = new FeedbackControl<>(y_system, Ky);
    FeedbackControl<N2, N1, N2> thetaController = new FeedbackControl<>(theta_system, Ktheta);

    ExtrapolatingEstimator<N2, N1, N2> x_predictor = new ExtrapolatingEstimator<>(x_system);
    ExtrapolatingEstimator<N2, N1, N2> y_predictor = new ExtrapolatingEstimator<>(y_system);
    ExtrapolatingEstimator<N2, N1, N2> theta_predictor = new ExtrapolatingEstimator<>(theta_system);

    PointEstimator<N2, N1, N2> x_PointEstimator = new PointEstimator<>(x_system);
    PointEstimator<N2, N1, N2> y_PointEstimator = new PointEstimator<>(y_system);
    PointEstimator<N2, N1, N2> theta_PointEstimator = new PointEstimator<>(theta_system);

    LinearPooling<N2> pooling = new VarianceWeightedLinearPooling<>();

    InversionFeedforward<N2, N1, N2> xFF = new InversionFeedforward<>(x_system);
    InversionFeedforward<N2, N1, N2> yFF = new InversionFeedforward<>(y_system);
    InversionFeedforward<N2, N1, N2> thetaFF = new InversionFeedforward<>(theta_system);

    NonlinearSystemLoop<N2, N1, N2> xLoop = new NonlinearSystemLoop<>(x_system, x_predictor, x_PointEstimator, pooling,
            xController, xFF);
    NonlinearSystemLoop<N2, N1, N2> yLoop = new NonlinearSystemLoop<>(y_system, y_predictor, y_PointEstimator, pooling,
            yController, yFF);
    NonlinearSystemLoop<N2, N1, N2> thetaLoop = new NonlinearSystemLoop<>(theta_system, theta_predictor,
            theta_PointEstimator, pooling, thetaController, thetaFF);

    Variance<N2> px = Variance.from2StdDev(.01, .01);
    Variance<N2> py = Variance.from2StdDev(.01, .01);
    Variance<N2> ptheta = Variance.from2StdDev(.316228, .316228);

    RandomVector<N2> xhat_x = new AngularRandomVector<>(VecBuilder.fill(0, 0), px);
    RandomVector<N2> xhat_y = new AngularRandomVector<>(VecBuilder.fill(0, 0), py);
    RandomVector<N2> xhat_theta = new AngularRandomVector<>(VecBuilder.fill(0, 0), ptheta);

    private double xErr = 0;
    private double yErr = 0;
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();

    public HolonomicDriveRegulator(
            DriveControllers controllers) {
        m_xController = controllers.xController;
        m_yController = controllers.yController;
        m_thetaController = controllers.thetaController;
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
        xhat_x = xLoop.correct(xhat_x, x_system.position(currentPose.getX()));
        xhat_y = yLoop.correct(xhat_y, y_system.position(currentPose.getY()));
        xhat_theta = thetaLoop.correct(xhat_theta, theta_system.position(currentPose.getRotation().getRadians()));

        Vector<N2> setpoint_x = VecBuilder.fill(desiredState.x().x(), desiredState.x().v());
        Vector<N2> setpoint_y = VecBuilder.fill(desiredState.y().x(), desiredState.y().v());
        Vector<N2> setpoint_theta = VecBuilder.fill(desiredState.theta().x(), desiredState.theta().v());

        Vector<N2> rDot_x = VecBuilder.fill(desiredState.x().v(), desiredState.x().a());
        Vector<N2> rDot_y = VecBuilder.fill(desiredState.y().v(), desiredState.y().a());
        Vector<N2> rDot_theta = VecBuilder.fill(desiredState.theta().v(), desiredState.theta().a());

        Matrix<N1, N1> totalU_x = xLoop.calculateTotalU(xhat_x, setpoint_x, rDot_x, kDt);
        Matrix<N1, N1> totalU_y = xLoop.calculateTotalU(xhat_y, setpoint_y, rDot_y, kDt);
        Matrix<N1, N1> totalU_theta = xLoop.calculateTotalU(xhat_theta, setpoint_theta, rDot_theta, kDt);

        xhat_x = xLoop.predictState(xhat_x, totalU_x, kDt);
        xhat_y = yLoop.predictState(xhat_y, totalU_y, kDt);
        xhat_theta = thetaLoop.predictState(xhat_theta, totalU_theta, kDt);

        return new Twist2d(totalU_x.get(0, 0), totalU_y.get(0, 0), totalU_theta.get(0, 0));
    }

    public void setGains(PidGains cartesian, PidGains rotation) {
        m_xController.setPID(cartesian.p, cartesian.i, cartesian.d);
        m_yController.setPID(cartesian.p, cartesian.i, cartesian.d);
        m_thetaController.setPID(rotation.p, rotation.i, rotation.d);
    }

    public void setIRange(double cartesian) {
        m_xController.setIntegratorRange(-1.0 * cartesian, cartesian);
        m_yController.setIntegratorRange(-1.0 * cartesian, cartesian);
    }

    public void setTolerance(double cartesian, double rotation) {
        m_xController.setTolerance(cartesian);
        m_yController.setTolerance(cartesian);
        m_thetaController.setTolerance(rotation);
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Holonomic2");

    private final DoublePublisher xSetPublisher = table.getDoubleTopic("xSet").publish();
    private final DoublePublisher xFFPublisher = table.getDoubleTopic("xFF").publish();
    private final DoublePublisher xFBPublisher = table.getDoubleTopic("xFB").publish();
    private final DoublePublisher poseXErrorPublisher = table.getDoubleTopic("xErr").publish();

    private final DoublePublisher ySetPublisher = table.getDoubleTopic("ySet").publish();
    private final DoublePublisher yFFPublisher = table.getDoubleTopic("yFF").publish();
    private final DoublePublisher yFBPublisher = table.getDoubleTopic("yFB").publish();
    private final DoublePublisher poseYErrorPublisher = table.getDoubleTopic("yErr").publish();

    private final DoublePublisher thetaSetPublisher = table.getDoubleTopic("thetaSet").publish();
    private final DoublePublisher thetaFFPublisher = table.getDoubleTopic("thetaFF").publish();
    private final DoublePublisher thetaFBPublisher = table.getDoubleTopic("thetaFB").publish();
    private final DoublePublisher thetaErrorPublisher = table.getDoubleTopic("thetaErr").publish();

}
