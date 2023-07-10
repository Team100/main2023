package org.team100.lib.sensors;

import java.util.function.Supplier;

import org.team100.lib.util.Unroller;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Combine mag and gyro with Kalman filter.
 * 
 * Mag is NED, gyro is NWU, we produce NWU.
 * 
 * Measurements are *unrolled* because I couldn't find a way to make the KF
 * understand wrapping.
 * 
 * NOTE: it is probably not possible to use this on a robot, because of the
 * effect of motor currents.
 */
public class FusedHeading implements Supplier<Rotation2d>, Sendable {
    private static final double kDtSec = 0.02;
    // State is [position, velocity]
    private static final Matrix<N2, N2> kA = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, 0);
    // Input is torque (unused for now)
    private static final Matrix<N2, N1> kB = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1);

    // private static final Pair<Matrix<N2, N2>, Matrix<N2, N1>> discABpair =
    // Discretization.discretizeAB(kA, kB, kDtSec);
    // private static final Matrix<N2, N2> kDiscA = discABpair.getFirst();

    private static final Matrix<N2, N2> kDiscA = Discretization.discretizeA(kA, kDtSec);

    // Output is [position, velocity] as measured by mag, gyro; accel is unmeasured.
    private static final Matrix<N2, N2> kC = Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1);
    // feedthrough is zero
    private static final Matrix<N2, N1> kD = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0);
    // State position stdev
    // State velocity stdev
    private static final Matrix<N2, N1> kStateStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(1, 1);
    // Compass is pretty noisy, 0.1 radians
    // Gyro stdev is extremely low, 0.005 radians/sec
    private static final Matrix<N2, N1> kOutputStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(0.01, 0.001);
    // For now there is no Input.
    private static final Matrix<N1, N1> kControlInput = Matrix.mat(Nat.N1(), Nat.N1()).fill(0);

    private final Unroller m_mag;
    private final LSM6DSOX_I2C m_gyro;
    // 2 states, 1 input, 2 outputs
    private final LinearSystem<N2, N1, N2> m_system;
    private final KalmanFilter<N2, N1, N2> m_filter;

    public FusedHeading() {
        m_mag = new Unroller(new LIS3MDL_I2C());
        m_gyro = new LSM6DSOX_I2C();
        m_system = new LinearSystem<N2, N1, N2>(kA, kB, kC, kD);
        m_filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), m_system, kStateStdDevs, kOutputStdDevs, kDtSec);
        Matrix<N2, N2> K = m_filter.getK();
        DriverStationJNI.sendConsoleLine(K.toString());
        DriverStationJNI.sendConsoleLine(String.format("K = [%10.5f %10.5f \n     %10.5f %10.5f]\n",
                K.get(0, 0), K.get(0, 1), K.get(1, 0), K.get(1, 1)));
        DriverStationJNI.sendConsoleLine(String.format("discA = [%10.5f %10.5f \n         %10.5f %10.5f]\n",
                kDiscA.get(0, 0),
                kDiscA.get(0, 1),
                kDiscA.get(1, 0),
                kDiscA.get(1, 1)));
        SmartDashboard.putData("heading", this);
    }

    private double getNEDMagRadians() {
        return m_mag.get().getRadians();
    }

    private double getNWUMagRadians() {
        return -1.0 * getNEDMagRadians();
    }

    private double getNWUGyroRadiansPerSec() {
        return m_gyro.getRate();
    }

    private double m_nwuMagRadians;
    private double m_nwuGyroRadiansPerSec;

    /**
     * Observations are [NWU radians, NWU radians/sec].
     */
    public Matrix<N2, N1> getObservations() {
        m_nwuMagRadians = getNWUMagRadians();
        m_nwuGyroRadiansPerSec = getNWUGyroRadiansPerSec();
        return Matrix.mat(Nat.N2(), Nat.N1()).fill(m_nwuMagRadians, m_nwuGyroRadiansPerSec);
    }

    public void reset() {
        Matrix<N2, N1> obs = getObservations();
        m_filter.setXhat(0, obs.get(0, 0));
        m_filter.setXhat(1, obs.get(1, 0));
    }

    public Matrix<N2, N1> getState() {
        return m_filter.getXhat();
    }

    public double m_posObs;
    public double m_velObs;
    public double m_posPred;
    public double m_velPred;
    public double m_posCorr;
    public double m_velCorr;
    public double m_posPredDiff;
    public double m_velPredDiff;
    public double m_posCorrDiff;
    public double m_velCorrDiff;
    public double m_posDiff;
    public double m_velDiff;

    /**
     * Yaw in radians referenced to magnetic north, NWU orientation.
     * 
     * This measurement is WRAPPED unlike everything else in this class, because the
     * gyro consumers expect it to be.
     */
    @Override
    public Rotation2d get() {
        // DataLogManager.log("FusedHeading.get()");

        // what is the predict step doing?
        // var discABpair = Discretization.discretizeAB(kA, kB, kDtSec);
        var discA = Discretization.discretizeA(kA, kDtSec);
        // HAL.sendConsoleLine(String.format("discA = [%10.5f %10.5f \n %10.5f
        // %10.5f]\n",
        // kDiscA.get(0, 0),
        // kDiscA.get(0, 1),
        // kDiscA.get(1, 0),
        // kDiscA.get(1, 1)));
        Matrix<N2, N1> state = m_filter.getXhat();
        Matrix<N2, N1> newState = discA.times(state);
        Matrix<N2, N1> stateDiff = newState.minus(state);
        m_posPredDiff = stateDiff.get(0, 0);
        m_velPredDiff = stateDiff.get(1, 0);

        // do the real prediction/correction.
        Matrix<N2, N1> obs = getObservations();
        m_posObs = obs.get(0, 0);
        m_velObs = obs.get(1, 0);

        m_filter.predict(kControlInput, kDtSec);
        state = m_filter.getXhat();
        m_posPred = state.get(0, 0);
        m_velPred = state.get(1, 0);

        // so the correction takes the observation and compares them to the state.
        Matrix<N2, N1> diffs = obs.minus(kC.times(state));
        m_posDiff = diffs.get(0, 0);
        m_velDiff = diffs.get(1, 0);

        Matrix<N2, N1> preCorrState = state;
        m_filter.correct(kControlInput, obs);
        state = m_filter.getXhat();
        m_posCorr = state.get(0, 0);
        m_velCorr = state.get(1, 0);
        Matrix<N2, N1> corrDiff = state.minus(preCorrState);
        m_posCorrDiff = corrDiff.get(0, 0);
        m_velCorrDiff = corrDiff.get(1, 0);
        return new Rotation2d(MathUtil.angleModulus(m_posCorr));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("position observed", () -> m_posObs, null);
        builder.addDoubleProperty("velocity observed", () -> m_velObs, null);
        builder.addDoubleProperty("position predicted", () -> m_posPred, null);
        builder.addDoubleProperty("velocity predicted", () -> m_velPred, null);
        builder.addDoubleProperty("position corrected", () -> m_posCorr, null);
        builder.addDoubleProperty("velocity corrected", () -> m_velCorr, null);
        builder.addDoubleProperty("position predicted diff", () -> m_posPredDiff, null);
        builder.addDoubleProperty("velocity predicted diff", () -> m_velPredDiff, null);
        builder.addDoubleProperty("position correction diff", () -> m_posCorrDiff, null);
        builder.addDoubleProperty("velocity correction diff", () -> m_velCorrDiff, null);
        builder.addDoubleProperty("position diff", () -> m_posDiff, null);
        builder.addDoubleProperty("velocity diff", () -> m_velDiff, null);

        // builder.addDoubleProperty("unrolled mag nwu radians", () -> m_nwuMagRadians,
        // null);
        // builder.addDoubleProperty("gyro rate nwu radians per sec", () ->
        // m_nwuGyroRadiansPerSec, null);
    }
}
