/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRCLib.Motors;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An abstraction for the Talon FX for debugging information
 */
public class FRCNEOCTREStyle implements Sendable {
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Text View");
        builder.addDoubleProperty("EncoderPosition", this::getSelectedSensorPosition, null);
        builder.addDoubleProperty("EncoderSpeed", this::getSensorVelocity, null);
        builder.addBooleanProperty("Fwd Limit", this.fwdLimitSwitch::isPressed, null);
        builder.addBooleanProperty("Rev Limit", this.revLimitSwitch::isPressed, null);
        builder.addDoubleProperty("current", motor::getOutputCurrent, null);
        builder.addBooleanProperty("Inverted", motor::getInverted, null);
        //builder.addStringProperty("Control Mode", () -> motor.getControlMode().toString(), null);
        // Output voltage not available
    }

    public void reset() {
        this.motor.restoreFactoryDefaults();
    }

    public void driveVelocity(double velocity) {
        this.closedLoop.setReference(velocity, ControlType.kVelocity);
    }

    public void drivePercentOutput(double percentOutput) {
        this.motor.set(percentOutput);
    }

    public void setSensorPosition(int position){
        this.motor.getEncoder().setPosition(position);
    }

    public void driveMotionMagic(double setpoint) {
        this.closedLoop.setReference(setpoint, ControlType.kSmartMotion);
    }

    public void drivePosition(double setpoint) {
        this.closedLoop.setReference(setpoint, ControlType.kPosition);
    }

    public void driveCurrent(double current) {
        this.closedLoop.setReference(current, ControlType.kCurrent);
    }

    public int getSensorVelocity() {
        return (int)this.motor.getEncoder().getVelocity();
    }

    public int getSelectedSensorPosition() {
        return (int)this.motor.getEncoder().getPosition();
    }

    public void updateSmartDashboard() {
        if (this.isSmartDashboardPutEnabled()) {

            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/percentOutput",
                    this.motor.getAppliedOutput());

            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/allowableClosedLoopError",
                    this.getAllowableClosedLoopError());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/auxPIDPolarity", this.isAuxPIDPolarity());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/canID", this.getCanID());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/closedLoopRampRate",
                    this.getClosedLoopRampRate());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/currentLimit", this.getCurrentLimit());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/currentLimitEnabled",
                    this.isCurrentLimitEnabled());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/feedbackPort", this.getFeedbackPort());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/forwardSoftLimitEnabled",
                    this.isForwardSoftLimitEnabled());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/forwardSoftLimitThreshold",
                    this.getForwardSoftLimitThreshold());
            SmartDashboard.putString(this.getSmartDashboardPath() + "/inverted", this.getInvertType().toString());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/kP", this.getkP());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/kI", this.getkI());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/kD", this.getkD());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/kF", this.getkF());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/motionAcceleration",
                    this.getMotionAcceleration());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/motionCruiseVelocity",
                    this.getMotionCruiseVelocity());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/motionCurveStrength",
                    this.getMotionCurveStrength());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/motionProfileTrajectoryPeriod",
                    this.motionProfileTrajectoryPeriod);
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/neutralDeadband", this.getNeutralDeadband());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/neutralDeadband",
                    (this.getNeutralMode() == NeutralMode.Coast) ? false : true);
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/openLoopRampRate", this.getOpenLoopRampRate());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/peakOutputForward", this.getPeakOutputForward());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/peakOutputReverse", this.getPeakOutputReverse());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/reverseSoftLimitEnabled",
                    this.isReverseSoftLimitEnabled());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/reverseSoftLimitThreshold",
                    this.getReverseSoftLimitThreshold());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/sensorPhase", this.isSensorPhase());
            SmartDashboard.putString(this.getSmartDashboardPath() + "/SmartDashboardPath",
                    this.getSmartDashboardPath());
            if (Robot.isReal()) {
                SmartDashboard.putNumber(this.getSmartDashboardPath() + "/timeout", this.getTimeout());
                SmartDashboard.putString(this.getSmartDashboardPath() + "/velocityMeasurementPeriod",
                        this.getVelocityMeasurementPeriod().toString());
                SmartDashboard.putNumber(this.getSmartDashboardPath() + "/velocityMeasurementWindow",
                        this.getVelocityMeasurementWindow());
            }

        }

    }

    ///////////////////////////////////////////////////////////////////////////
    /**
     * A direct reference to the CANSparkMax motor, designed for direct control
     */
    public CANSparkMax motor;

    /**
     * A direct reference to the CANSparkMax motor, designed for direct control
     */
    public SparkMaxPIDController closedLoop;

    /**
     * The master that will be followed
     */
    public FRCNEOCTREStyle master;
    ///////////////////////////////////////////////////////////////////////////

    /**
     * Limit switches
     */
    private SparkMaxLimitSwitch fwdLimitSwitch;
    private SparkMaxLimitSwitch revLimitSwitch;

    /**
     * The Can ID of the selected motor
     */
    private int canID;

    /**
     * The inversion of the motor
     *
     * Uses CTRE InvertType
     */
    private InvertType invertType;

    /**
     * The feedback port of the motor Default is 0
     */
    private int feedbackPort = 0;

    /**
     * The timeout of the motor in ms
     *
     * Default is 10
     */
    private int timeout = 10;

    /**
     * The sensor phase of the motor
     *
     * true inverts the encoder signal
     */
    private boolean sensorPhase;

    /**
     * The kP value of the motor's PID controller
     */
    private double kP;

    /**
     * The kI value of the motor's PID controller
     */
    private double kI;

    /**
     * The kD value of the motor's PID controller
     */
    private double kD;

    /**
     * The kF value of the motor's PID controller
     */
    private double kF;

    /**
     * The acceptable closed loop error in ticks
     */
    private int allowableClosedLoopError;

    /**
     * Is a current limit enabled
     *
     * a currentLimit must be set if this is true
     */
    private boolean currentLimitEnabled;

    /**
     * The current limit set (amps)
     *
     * currentLimitEnabled must be set for this to activate
     */
    private int currentLimit;

    /**
     * The neutral mode of the motor controller
     */
    private NeutralMode neutralMode;

    /**
     * Should a smartDashboardPut be enabled
     *
     * true will put to SmartDashboard
     */
    private boolean smartDashboardPutEnabled;

    /**
     * The path that the motor controller should report to
     */
    private String smartDashboardPath;

    /**
     * The ramp rate when controlled in open loop
     */
    private double openLoopRampRate;

    /**
     * The ramp rate when controlled in closed loop
     */
    private double closedLoopRampRate;

    /**
     * The forward peak output
     */
    private double peakOutputForward;

    /**
     * The reverse peak output
     */
    private double peakOutputReverse;

    /**
     * The neutral deadband
     */
    private double neutralDeadband;

    /**
     * The measurement period (in ms) for velocity control
     */
    private SensorVelocityMeasPeriod velocityMeasurementPeriod;

    /**
     * The measurement window for the velocity control
     */
    private int velocityMeasurementWindow;

    /**
     * Is a forward soft limit enabled
     */
    private boolean forwardSoftLimitEnabled;

    /**
     * The forward soft limit set
     */
    private int forwardSoftLimitThreshold;

    /**
     * Is a reverse soft limit enabled
     */
    private boolean reverseSoftLimitEnabled;

    /**
     * The reverse soft limit set
     */
    private int reverseSoftLimitThreshold;

    /**
     * Is auxiliary polarity enabled
     */
    private boolean auxPIDPolarity;

    /**
     * The motion cruise velocity set
     */
    private int motionCruiseVelocity;

    /**
     * The motion acceleration set
     */
    private int motionAcceleration;

    /**
     * The strength of the motion curve
     */
    private int motionCurveStrength;

    /**
     * The period set when using motion profiles
     */
    private int motionProfileTrajectoryPeriod;

    public void updatePIDController() {
        closedLoop.setP(this.getkP());
        closedLoop.setI(this.getkI());
        closedLoop.setD(this.getkD());
        closedLoop.setFF(this.getkF());

        System.out.println("Wrote PID TO " + canID + ", KF VALUE " + this.getkF());
    }

    public FRCNEOCTREStyle configure() {
        motor = new CANSparkMax(this.getCanID(), MotorType.kBrushless);

        closedLoop = motor.getPIDController();

        fwdLimitSwitch = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        revLimitSwitch = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

        motor.restoreFactoryDefaults();

        switch (this.invertType) {
            case InvertMotorOutput:
                motor.setInverted(true);
                break;
            case None:
                motor.setInverted(false);
                break;
            case FollowMaster:
                motor.setInverted(master.invertType == InvertType.InvertMotorOutput || master.invertType == InvertType.OpposeMaster);
                break;
            case OpposeMaster:
                motor.setInverted(master.invertType == InvertType.None || master.invertType == InvertType.FollowMaster);
                break;
        }
        System.out.println("Configuring Inverted");

        if (this.isForwardSoftLimitEnabled()) {
            motor.enableSoftLimit(SoftLimitDirection.kForward, this.isForwardSoftLimitEnabled());
            motor.setSoftLimit(SoftLimitDirection.kForward, this.getForwardSoftLimitThreshold());
            System.out.println("Configuring forward soft limit");
        }

        if (this.getMotionAcceleration() != 0) {
            closedLoop.setSmartMotionMaxAccel(this.getMotionAcceleration(), 0);
            closedLoop.setSmartMotionMaxVelocity(this.getMotionCruiseVelocity(), 0);
            System.out.println("Configuring acceleration");
        }

        if (this.getNeutralMode() != null) {
            if (this.getNeutralMode() == NeutralMode.Coast) {
                motor.setIdleMode(IdleMode.kCoast);
            } else if (this.getNeutralMode() == NeutralMode.Brake) {
                motor.setIdleMode(IdleMode.kBrake);
            }
            System.out.println("Setting Neutral Mode");
        }

        if (this.getOpenLoopRampRate() != 0) {
            motor.setOpenLoopRampRate(this.getOpenLoopRampRate());
            System.out.println("Setting Open Loop Ramp Rate");

        }

        if (this.getPeakOutputForward() != 0 || this.getPeakOutputReverse() != 0) {
            closedLoop.setOutputRange(this.getPeakOutputReverse(), this.getPeakOutputForward());
            System.out.println("Setting Peak Output");
        }

        if (this.isReverseSoftLimitEnabled()) {
            motor.enableSoftLimit(SoftLimitDirection.kReverse, this.isReverseSoftLimitEnabled());
            motor.setSoftLimit(SoftLimitDirection.kReverse, this.getReverseSoftLimitThreshold());
            System.out.println("setting reverse soft limit enabled");
        }

        if (this.isSensorPhase()) {
            motor.getEncoder().setInverted(this.isSensorPhase());
            System.out.println("setting sensor phase");
        }

        if (this.getVelocityMeasurementPeriod() != null || this.getVelocityMeasurementWindow() != 0) {
            int velocityPeriodInt = this.getVelocityMeasurementPeriod().value;
            motor.getEncoder().setMeasurementPeriod(velocityPeriodInt);
            motor.getEncoder().setAverageDepth(this.getVelocityMeasurementWindow());
            System.out.println("Setting Velocity Measurement Period");
        }

        System.out.println("MOTOR " + canID + ", KF "+ this.getkF());

        if (this.getkP() != 0 || this.getkI() != 0 || this.getkD() != 0 || this.getkF() != 0) {
            updatePIDController();
            System.out.println("Setting PID Controller");
        }

        if (this.master != null) {
            motor.follow(master.motor, motor.getInverted());
        }

        return this;
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public void setMotor(CANSparkMax motor) {
        this.motor = motor;
    }

    public int getCanID() {
        return canID;
    }

    public void setCanID(int canID) {
        this.canID = canID;
    }

    public boolean isInvertedWithType() {
        return invertType != InvertType.None;
    }

    public InvertType getInvertType() {
        return this.invertType;
    }

    public void setInverted(InvertType inverted) {
        this.invertType = inverted;
    }

    public int getFeedbackPort() {
        return feedbackPort;
    }

    public void setFeedbackPort(int feedbackPort) {
        this.feedbackPort = feedbackPort;
    }

    public int getTimeout() {
        return timeout;
    }

    public void setTimeout(int timeout) {
        this.timeout = timeout;
    }

    public boolean isSensorPhase() {
        return sensorPhase;
    }

    public void setSensorPhase(boolean sensorPhase) {
        this.sensorPhase = sensorPhase;
    }

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
        this.updatePIDController();
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
        this.updatePIDController();
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
        this.updatePIDController();
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
        this.updatePIDController();
    }

    public int getAllowableClosedLoopError() {
        return allowableClosedLoopError;
    }

    public void setAllowableClosedLoopError(int allowableClosedLoopError) {
        this.allowableClosedLoopError = allowableClosedLoopError;
    }

    public boolean isCurrentLimitEnabled() {
        return currentLimitEnabled;
    }

    public void setCurrentLimitEnabled(boolean currentLimitEnabled) {
        this.currentLimitEnabled = currentLimitEnabled;
    }

    public int getCurrentLimit() {
        return currentLimit;
    }

    public void setCurrentLimit(int currentLimit) {
        this.currentLimit = currentLimit;
    }

    public NeutralMode getNeutralMode() {
        return neutralMode;
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        this.neutralMode = neutralMode;
    }

    public boolean isSmartDashboardPutEnabled() {
        return smartDashboardPutEnabled;
    }

    public void setSmartDashboardPutEnabled(boolean smartDashboardPutEnabled) {
        this.smartDashboardPutEnabled = smartDashboardPutEnabled;
    }

    public String getSmartDashboardPath() {
        return smartDashboardPath;
    }

    public void setSmartDashboardPath(String smartDashboardPath) {
        this.smartDashboardPath = smartDashboardPath;
    }

    public double getOpenLoopRampRate() {
        return openLoopRampRate;
    }

    public void setOpenLoopRampRate(double openLoopRampRate) {
        this.openLoopRampRate = openLoopRampRate;
    }

    public double getClosedLoopRampRate() {
        return closedLoopRampRate;
    }

    public void setClosedLoopRampRate(double closedLoopRampRate) {
        this.closedLoopRampRate = closedLoopRampRate;
    }

    public double getPeakOutputForward() {
        return peakOutputForward;
    }

    public void setPeakOutputForward(double peakOutputForward) {
        this.peakOutputForward = peakOutputForward;
    }

    public double getPeakOutputReverse() {
        return peakOutputReverse;
    }

    public void setPeakOutputReverse(double peakOutputReverse) {
        this.peakOutputReverse = peakOutputReverse;
    }

    public double getNeutralDeadband() {
        return neutralDeadband;
    }

    public void setNeutralDeadband(double neutralDeadband) {
        this.neutralDeadband = neutralDeadband;
    }

    public SensorVelocityMeasPeriod getVelocityMeasurementPeriod() {
        return velocityMeasurementPeriod;
    }

    public void setVelocityMeasurementPeriod(SensorVelocityMeasPeriod velocityMeasurementPeriod) {
        this.velocityMeasurementPeriod = velocityMeasurementPeriod;
    }

    public int getVelocityMeasurementWindow() {
        return velocityMeasurementWindow;
    }

    public void setVelocityMeasurementWindow(int velocityMeasurementWindow) {
        this.velocityMeasurementWindow = velocityMeasurementWindow;
    }

    public boolean isForwardSoftLimitEnabled() {
        return forwardSoftLimitEnabled;
    }

    public void setForwardSoftLimitEnabled(boolean forwardSoftLimitEnabled) {
        this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
    }

    public int getForwardSoftLimitThreshold() {
        return forwardSoftLimitThreshold;
    }

    public void setForwardSoftLimitThreshold(int forwardSoftLimitThreshold) {
        this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
    }

    public boolean isReverseSoftLimitEnabled() {
        return reverseSoftLimitEnabled;
    }

    public void setReverseSoftLimitEnabled(boolean reverseSoftLimitEnabled) {
        this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
    }

    public int getReverseSoftLimitThreshold() {
        return reverseSoftLimitThreshold;
    }

    public void setReverseSoftLimitThreshold(int reverseSoftLimitThreshold) {
        this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
    }

    public boolean isAuxPIDPolarity() {
        return auxPIDPolarity;
    }

    public void setAuxPIDPolarity(boolean auxPIDPolarity) {
        this.auxPIDPolarity = auxPIDPolarity;
    }

    public int getMotionCruiseVelocity() {
        return motionCruiseVelocity;
    }

    public void setMotionCruiseVelocity(int motionCruiseVelocity) {
        this.motionCruiseVelocity = motionCruiseVelocity;
    }

    public int getMotionAcceleration() {
        return motionAcceleration;
    }

    public void setMotionAcceleration(int motionAcceleration) {
        this.motionAcceleration = motionAcceleration;
    }

    public int getMotionCurveStrength() {
        return motionCurveStrength;
    }

    public void setMotionCurveStrength(int motionCurveStrength) {
        this.motionCurveStrength = motionCurveStrength;
    }

    public int getMotionProfileTrajectoryPeriod() {
        return motionProfileTrajectoryPeriod;
    }

    public void setMotionProfileTrajectoryPeriod(int motionProfileTrajectoryPeriod) {
        this.motionProfileTrajectoryPeriod = motionProfileTrajectoryPeriod;
    }

    public void setMaster(FRCNEOCTREStyle master) {
        this.master = master;
    }

    public static final class FRCNEOCTREStyleBuilder {
        private int canID;
        private InvertType invertType = InvertType.None;
        private int feedbackPort = 0;
        private int timeout = 10;
        private boolean sensorPhase = false;
        private double kP = 0.0;
        private double kI = 0.0;
        private double kD = 0.0;
        private double kF = 0.0;
        private int allowableClosedLoopError = 0;
        private boolean currentLimitEnabled = false;
        private int currentLimit = 0;
        private NeutralMode neutralMode = NeutralMode.Coast;
        private boolean smartDashboardPutEnabled = false;
        private String smartDashboardPath;
        private double openLoopRampRate = 0;
        private double closedLoopRampRate = 0;
        private double peakOutputForward = 1.0;
        private double peakOutputReverse = -1.0;
        private double neutralDeadband = 0.04;
        private SensorVelocityMeasPeriod velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;// ??
        private int velocityMeasurementWindow = 64;
        private boolean forwardSoftLimitEnabled = false;
        private int forwardSoftLimitThreshold = 0;
        private boolean reverseSoftLimitEnabled = false;
        private int reverseSoftLimitThreshold = 0;
        private boolean auxPIDPolarity = false;
        private int motionCruiseVelocity = 0;
        private int motionAcceleration = 0;
        private int motionCurveStrength = 0;
        private int motionProfileTrajectoryPeriod = 0;
        private FRCNEOCTREStyle master;

        public FRCNEOCTREStyleBuilder(int canID) {
            this.canID = canID;
            this.smartDashboardPath = "NEO_" + canID;
        }

        // public static FRCNEOBuilder aFRCNEO() {
        // return new FRCNEOBuilder();
        // }

        public FRCNEOCTREStyleBuilder withCanID(int canID) {
            this.canID = canID;
            return this;
        }

        public FRCNEOCTREStyleBuilder withInverted(InvertType invert) {
            this.invertType = invert;
            return this;
        }

        public FRCNEOCTREStyleBuilder withFeedbackPort(int feedbackPort) {
            this.feedbackPort = feedbackPort;
            return this;
        }

        public FRCNEOCTREStyleBuilder withTimeout(int timeout) {
            this.timeout = timeout;
            return this;
        }

        public FRCNEOCTREStyleBuilder withSensorPhase(boolean sensorPhase) {
            this.sensorPhase = sensorPhase;
            return this;
        }

        public FRCNEOCTREStyleBuilder withKP(double kP) {
            this.kP = kP;
            return this;
        }

        public FRCNEOCTREStyleBuilder withKI(double kI) {
            this.kI = kI;
            return this;
        }

        public FRCNEOCTREStyleBuilder withKD(double kD) {
            this.kD = kD;
            return this;
        }

        public FRCNEOCTREStyleBuilder withKF(double kF) {
            this.kF = kF;
            return this;
        }

        public FRCNEOCTREStyleBuilder withAllowableClosedLoopError(int allowableClosedLoopError) {
            this.allowableClosedLoopError = allowableClosedLoopError;
            return this;
        }

        public FRCNEOCTREStyleBuilder withCurrentLimitEnabled(boolean currentLimitEnabled) {
            this.currentLimitEnabled = currentLimitEnabled;
            return this;
        }

        public FRCNEOCTREStyleBuilder withCurrentLimit(int currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public FRCNEOCTREStyleBuilder withNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
            return this;
        }

        public FRCNEOCTREStyleBuilder withSmartDashboardPutEnabled(boolean smartDashboardPutEnabled) {
            this.smartDashboardPutEnabled = smartDashboardPutEnabled;
            return this;
        }

        public FRCNEOCTREStyleBuilder withSmartDashboardPath(String smartDashboardPath) {
            this.smartDashboardPath = smartDashboardPath;
            return this;
        }

        public FRCNEOCTREStyleBuilder withOpenLoopRampRate(double openLoopRampRate) {
            this.openLoopRampRate = openLoopRampRate;
            return this;
        }

        public FRCNEOCTREStyleBuilder withClosedLoopRampRate(double closedLoopRampRate) {
            this.closedLoopRampRate = closedLoopRampRate;
            return this;
        }

        public FRCNEOCTREStyleBuilder withPeakOutputForward(double peakOutputForward) {
            this.peakOutputForward = peakOutputForward;
            return this;
        }

        public FRCNEOCTREStyleBuilder withPeakOutputReverse(double peakOutputReverse) {
            this.peakOutputReverse = peakOutputReverse;
            return this;
        }

        public FRCNEOCTREStyleBuilder withNeutralDeadband(double neutralDeadband) {
            this.neutralDeadband = neutralDeadband;
            return this;
        }

        public FRCNEOCTREStyleBuilder withVelocityMeasurementPeriod(SensorVelocityMeasPeriod velocityMeasurementPeriod) {
            this.velocityMeasurementPeriod = velocityMeasurementPeriod;
            return this;
        }

        public FRCNEOCTREStyleBuilder withVelocityMeasurementWindow(int velocityMeasurementWindow) {
            this.velocityMeasurementWindow = velocityMeasurementWindow;
            return this;
        }

        public FRCNEOCTREStyleBuilder withForwardSoftLimitEnabled(boolean forwardSoftLimitEnabled) {
            this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
            return this;
        }

        public FRCNEOCTREStyleBuilder withForwardSoftLimitThreshold(int forwardSoftLimitThreshold) {
            this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
            return this;
        }

        public FRCNEOCTREStyleBuilder withReverseSoftLimitEnabled(boolean reverseSoftLimitEnabled) {
            this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
            return this;
        }

        public FRCNEOCTREStyleBuilder withReverseSoftLimitThreshold(int reverseSoftLimitThreshold) {
            this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
            return this;
        }

        public FRCNEOCTREStyleBuilder withAuxPIDPolarity(boolean auxPIDPolarity) {
            this.auxPIDPolarity = auxPIDPolarity;
            return this;
        }

        public FRCNEOCTREStyleBuilder withMotionCruiseVelocity(int motionCruiseVelocity) {
            this.motionCruiseVelocity = motionCruiseVelocity;
            return this;
        }

        public FRCNEOCTREStyleBuilder withMotionAcceleration(int motionAcceleration) {
            this.motionAcceleration = motionAcceleration;
            return this;
        }

        public FRCNEOCTREStyleBuilder withMotionCurveStrength(int motionCurveStrength) {
            this.motionCurveStrength = motionCurveStrength;
            return this;
        }

        public FRCNEOCTREStyleBuilder withMotionProfileTrajectoryPeriod(int motionProfileTrajectoryPeriod) {
            this.motionProfileTrajectoryPeriod = motionProfileTrajectoryPeriod;
            return this;
        }

        public FRCNEOCTREStyleBuilder withMaster(FRCNEOCTREStyle master) {
            this.master = master;
            return this;
        }

        public FRCNEOCTREStyle build() {
            FRCNEOCTREStyle fRCNEO = new FRCNEOCTREStyle();
            fRCNEO.setCanID(canID);
            fRCNEO.setInverted(invertType);
            fRCNEO.setFeedbackPort(feedbackPort);
            fRCNEO.setTimeout(timeout);
            fRCNEO.setSensorPhase(sensorPhase);
            fRCNEO.setAllowableClosedLoopError(allowableClosedLoopError);
            fRCNEO.setCurrentLimitEnabled(currentLimitEnabled);
            fRCNEO.setCurrentLimit(currentLimit);
            fRCNEO.setNeutralMode(neutralMode);
            fRCNEO.setSmartDashboardPutEnabled(smartDashboardPutEnabled);
            fRCNEO.setSmartDashboardPath(smartDashboardPath);
            fRCNEO.setOpenLoopRampRate(openLoopRampRate);
            fRCNEO.setClosedLoopRampRate(closedLoopRampRate);
            fRCNEO.setPeakOutputForward(peakOutputForward);
            fRCNEO.setPeakOutputReverse(peakOutputReverse);
            fRCNEO.setNeutralDeadband(neutralDeadband);
            fRCNEO.setVelocityMeasurementPeriod(velocityMeasurementPeriod);
            fRCNEO.setVelocityMeasurementWindow(velocityMeasurementWindow);
            fRCNEO.setForwardSoftLimitEnabled(forwardSoftLimitEnabled);
            fRCNEO.setForwardSoftLimitThreshold(forwardSoftLimitThreshold);
            fRCNEO.setReverseSoftLimitEnabled(reverseSoftLimitEnabled);
            fRCNEO.setReverseSoftLimitThreshold(reverseSoftLimitThreshold);
            fRCNEO.setAuxPIDPolarity(auxPIDPolarity);
            fRCNEO.setMotionCruiseVelocity(motionCruiseVelocity);
            fRCNEO.setMotionAcceleration(motionAcceleration);
            fRCNEO.setMotionCurveStrength(motionCurveStrength);
            fRCNEO.setMotionProfileTrajectoryPeriod(motionProfileTrajectoryPeriod);
            fRCNEO.setMaster(master);
            fRCNEO.kF = this.kF;
            fRCNEO.kD = this.kD;
            fRCNEO.kI = this.kI;
            fRCNEO.kP = this.kP;
            return fRCNEO.configure();
        }
    }

}