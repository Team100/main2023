package org.team100.frc2023.subsystems.arm;

import org.team100.lib.motion.arm.ArmKinematics;
import org.team100.lib.motor.FRCNEO;
import org.team100.frc2023.LQRManager;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.State100;
import org.team100.lib.motion.arm.ArmAngles;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Arm servo. Subsystem runs *all the time*, commands feed it references.
 */
public class ArmSubsystem extends Subsystem implements ArmInterface {
    private static class Noop extends Subsystem implements ArmInterface {

        @Override
        public Subsystem subsystem() {
            return this;
        }

		@Override
		public boolean getCubeMode() {
			return false;
		}

		@Override
		public void setCubeMode(boolean b) {			
		}

        @Override
        public void setReference(ArmAngles reference) {
            
        }

        @Override
        public ArmAngles getMeasurement() {
            return new ArmAngles(0,0);
        }

        @Override
        public void setControlNormal() {
        }

        @Override
        public void setControlSafe() {            
        }

        @Override
        public void close() {            
        }

        @Override
        public void setUpperSpeed(double x){

        }

        @Override
        public void setLowerSpeed(double x){

        }

    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public ArmInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                    return new ArmSubsystem();
                default:
                    return new Noop();
            }
        }
    }

    public static class Config {
        public double softStop = -0.594938;
        public double kUpperArmLengthM = 0.92;
        public double kLowerArmLengthM = 0.93;
        public double filterTimeConstantS = 0.06; // TODO: tune the time constant
        public double filterPeriodS = 0.02;
        public double safeP = 2.5;
        public double safeI = 0;
        public double safeD = 0;
        public double normalLowerP = 0.3;
        public double normalLowerI = 0;
        public double normalLowerD = 0.1;
        public double normalUpperP = 4;
        public double normalUpperI = 0.2;
        public double normalUpperD = 0.05;
        public double tolerance = 0.001;
    }

    // TODO: do something with this
    public static class ArmState {
        private final State100 m_lower;
        private final State100 m_upper;

        public ArmState(State100 lower, State100 upper) {
            m_lower = lower;
            m_upper = upper;
        }

        public State100 lower() {
            return m_lower;
        }

        public State100 upper() {
            return m_upper;
        }
    }

    private final Config m_config = new Config();

    /** Kinematics in meters. Coordinates are x up, y forward. */
    private final ArmKinematics m_armKinematicsM;
    private final LinearFilter m_lowerMeasurementFilter;
    private final LinearFilter m_upperMeasurementFilter;
    private final PIDController m_lowerController;
    private final PIDController m_upperController;
    private final FRCNEO lowerArmMotor;
    private final FRCNEO upperArmMotor;
    private final AnalogInput lowerArmInput;
    private final AnalogInput upperArmInput;
    private final AnalogEncoder lowerArmEncoder;
    private final AnalogEncoder upperArmEncoder;
    private final double kLowerArmGearing = 1;
    private final double kUpperArmGearing = 1; //TODO add arm gear ratios
    private final TrapezoidProfile.Constraints m_constraints1 = new TrapezoidProfile.Constraints(8*Math.PI,4*Math.PI); //TODO add arm max accel and speed
    private final TrapezoidProfile.Constraints m_constraints2 = new TrapezoidProfile.Constraints(8*Math.PI,4*Math.PI);
    private final LinearSystem<N2, N1, N1> m_armPlant1 = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.43, kLowerArmGearing);//TODO add arm moment of inertia
    private final LinearSystem<N2, N1, N1> m_armPlant2 = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 0.43, kUpperArmGearing);
    private final KalmanFilter<N2, N1, N1> m_observer1 = new KalmanFilter<>(Nat.N2(), Nat.N1(),m_armPlant1,VecBuilder.fill(0.015, 0.17), VecBuilder.fill(2*Math.PI/(42*45)), 0.020);//TODO TUNE THESE VALUES
    private final KalmanFilter<N2, N1, N1> m_observer2 = new KalmanFilter<>(Nat.N2(), Nat.N1(),m_armPlant2,VecBuilder.fill(0.015, 0.17), VecBuilder.fill(2*Math.PI/(42*45)), 0.020);
    private final LinearQuadraticRegulator<N2, N1, N1>  m_controller1 = new LinearQuadraticRegulator<>(m_armPlant1, VecBuilder.fill(Units.degreesToRadians(0.01), Units.degreesToRadians(1)), VecBuilder.fill(12),0.02); //TODO tune these values
    private final LinearQuadraticRegulator<N2, N1, N1>  m_controller2 = new LinearQuadraticRegulator<>(m_armPlant2, VecBuilder.fill(Units.degreesToRadians(0.01), Units.degreesToRadians(1)), VecBuilder.fill(12),0.02);
    private final LQRManager lowerLQRController = new LQRManager(m_armPlant1,m_observer1,m_controller1,m_constraints1);
    private final LQRManager upperLQRController = new LQRManager(m_armPlant2,m_observer2,m_controller2,m_constraints2);
    // TODO: move this somewhere else
    private boolean cubeMode = true;
    public void setCubeMode(boolean mode) {
        cubeMode = mode;
    }
    @Override
    public boolean getCubeMode() {
        return cubeMode;
    }
    private ArmAngles m_reference;

    private ArmSubsystem() {
        m_armKinematicsM = new ArmKinematics(m_config.kLowerArmLengthM, m_config.kUpperArmLengthM);
        m_lowerMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_upperMeasurementFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        m_lowerController = new PIDController(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController = new PIDController(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
        m_lowerController.setTolerance(m_config.tolerance);
        m_upperController.setTolerance(m_config.tolerance);

        lowerArmMotor = new FRCNEO.FRCNEOBuilder(43)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(40)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .build();

        upperArmMotor = new FRCNEO.FRCNEOBuilder(42)
                .withInverted(false)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(40)
                .withPeakOutputForward(0.5)
                .withPeakOutputReverse(-0.5)
                .withNeutralMode(IdleMode.kBrake)
                .withForwardSoftLimitEnabled(false)
                .build();

        lowerArmInput = new AnalogInput(6);
        lowerArmEncoder = new AnalogEncoder(lowerArmInput);
        upperArmInput = new AnalogInput(5);
        upperArmEncoder = new AnalogEncoder(upperArmInput);
        lowerLQRController.m_loop.reset(VecBuilder.fill(lowerArmEncoder.getDistance(), 0));
        upperLQRController.m_loop.reset(VecBuilder.fill(upperArmEncoder.getDistance(), 0));
        lowerLQRController.m_lastProfiledReference = new TrapezoidProfile.State(lowerArmEncoder.getDistance(), 0);
        upperLQRController.m_lastProfiledReference = new TrapezoidProfile.State(upperArmEncoder.getDistance(), 0);
        m_reference = getMeasurement();

        SmartDashboard.putData("Arm Subsystem", this);
    }

    @Override
    public void periodic() {;
        ArmAngles measurement = getMeasurement();
        double u1 = lowerLQRController.calculate(measurement.th1, m_reference.th1, 0);
        double u2 = upperLQRController.calculate(measurement.th2, m_reference.th2, 0);
        lowerArmMotor.setVoltage(soften(u1));
        upperArmMotor.setVoltage(u2);
    }

    /**
     * Update the reference to track. This might come from a trajectory sampler or
     * from manual control.
     */
    public void setReference(ArmAngles reference) {
        m_reference = reference;
    }

    /** Measure the arm position, smoothed with single-pole IIR low-pass filter. */
    public ArmAngles getMeasurement() {
        return new ArmAngles(
                m_lowerMeasurementFilter.calculate(getLowerArm()),
                m_upperMeasurementFilter.calculate(getUpperArm()));
    }

    public void setControlNormal() {
        m_lowerController.setPID(m_config.normalLowerP, m_config.normalLowerI, m_config.normalLowerD);
        m_upperController.setPID(m_config.normalUpperP, m_config.normalUpperI, m_config.normalUpperD);
    }

    /**
     * Safe control parameters are gentler, intended to keep from crashing into the
     * safe-position hard-stop.
     */
    public void setControlSafe() {
        m_lowerController.setPID(m_config.safeP, m_config.safeI, m_config.safeD);
        m_upperController.setPID(m_config.safeP, m_config.safeI, m_config.safeD);
    }

    public void close() {
        lowerArmMotor.close();
        upperArmMotor.close();
        lowerArmEncoder.close();
        upperArmEncoder.close();
        lowerArmInput.close();
        upperArmInput.close();
    }

    ///////////////////////////////////////////////////////////////////////////////

    /** Stop the motor at the soft stop. */
    private double soften(double x) {
        if (softBand(x))
            return 0;
        return x;
    }

    /** Should the motor stop? */
    private boolean softBand(double x) {
        if (getLowerArm() <= m_config.softStop && x < 0)
            return false;
        return true;
    }

    /** Lower arm angle (in radians) 0 up, positive forward. */
    private double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
        return (-1.0 * x) * Math.PI / 180;
    }

    /** Upper arm angle (radians), 0 up, positive forward. */
    private double getUpperArm() {
        double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 360;
        return x * Math.PI / 180;
    }

    public void setUpperSpeed(double x){

        upperArmMotor.drivePercentOutput(x);

    }

    public void setLowerSpeed(double x){

        lowerArmMotor.drivePercentOutput(x);

    }

    private double getUpperArmDegrees() {
        return getUpperArm() * 180 / Math.PI;
    }

    private double getLowerArmDegrees() {
        return getLowerArm() * 180 / Math.PI;
    }

    @Override
    public Subsystem subsystem() {
        return this;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ARM X", () -> m_armKinematicsM.forward(getMeasurement()).getX(), null);
        builder.addDoubleProperty("ARM Y", () -> m_armKinematicsM.forward(getMeasurement()).getY(), null);
        builder.addDoubleProperty("Upper Arm Absolute Angle", () -> upperArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Lower Arm Absolute Angle", () -> lowerArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Upper Arm Absolute Radians", () -> getUpperArm(), null);
        builder.addDoubleProperty("Lower Arm Absolute Radians", () -> getLowerArm(), null);
        builder.addDoubleProperty("Upper Arm Absolute Degrees", () -> getUpperArmDegrees(), null);
        builder.addDoubleProperty("Lower Arm Absolute Degrees", () -> getLowerArmDegrees(), null);
        builder.addBooleanProperty("Cube Mode", () -> cubeMode, null);
        builder.addDoubleProperty("Upper Angle Setpoint", () -> m_reference.th2, null);
        builder.addDoubleProperty("Lower Angle Setpoint", () -> m_reference.th1, null);
    }
}