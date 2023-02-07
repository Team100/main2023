/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.FRCLib.Motors;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An abstraction for the Talon FX for debugging information
 */
public class FRCTalonFX implements Sendable {
    
    /**
     * Tells Talon FX which motor controller to follow
     * 
     * Send null to stop following
     *  
     * @param master motor to follow
     * 
     * @deprecated use FRCTalonFXBuilder.withMaster() instead.  
     */
    @Deprecated(forRemoval = true)
    public void follow(FRCTalonFX master) {
        this.master = master;
        this.motor.follow(this.master.motor);

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Text View");
        builder.addDoubleProperty("EncoderPosition", this::getSelectedSensorPosition, null);
        builder.addDoubleProperty("EncoderSpeed", this::getSensorVelocity, null);
        builder.addDoubleProperty("Fwd Limit", this.m_sensorCollection::isFwdLimitSwitchClosed, null);
        builder.addDoubleProperty("Rev Limit", this.m_sensorCollection::isRevLimitSwitchClosed, null);
        builder.addDoubleProperty("current", motor::getStatorCurrent, null);
        builder.addBooleanProperty("Inverted", motor::getInverted, null);
        builder.addStringProperty("Control Mode", () -> motor.getControlMode().toString(), null);
        builder.addDoubleProperty("Voltage Output", motor::getMotorOutputVoltage, null);
    }

    public void reset() {
        this.motor.configFactoryDefault();
    }

    public void driveVelocity(double velocity) {
        this.motor.set(ControlMode.Velocity, velocity);
    }

    public void drivePercentOutput(double percentOutput) {
        this.motor.set(ControlMode.PercentOutput, percentOutput);

    }

    public void setSensorPosition(int position){
        this.motor.setSelectedSensorPosition(position);
    }

    public void driveMotionMagic(double setpoint) {
        this.motor.set(ControlMode.MotionProfile, setpoint);
    }

    public void drivePosition(double setpoint) {
        this.motor.set(ControlMode.Position, setpoint);
    }

    public void driveCurrent(double current) {
        this.motor.set(ControlMode.Current, current);
    }

    public double getSensorVelocity() {
        return this.motor.getSelectedSensorVelocity();
    }

    public double getSelectedSensorPosition() {
        return this.motor.getSelectedSensorPosition();
    }

    public void updateSmartDashboard() {
        if (this.isSmartDashboardPutEnabled()) {

            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/percentOutput",
                    this.motor.getMotorOutputPercent());

            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/allowableClosedLoopError",
                    this.getAllowableClosedLoopError());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/auxPIDPolarity", this.isAuxPIDPolarity());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/canID", this.getCanID());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/closedLoopRampRate",
                    this.getClosedLoopRampRate());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/currentLimit", this.getCurrentLimit());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/currentLimitEnabled",
                    this.isCurrentLimitEnabled());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/feedbackNotContinuous",
                    this.isFeedbackNotContinuous());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/feedbackPort", this.getFeedbackPort());
            SmartDashboard.putBoolean(this.getSmartDashboardPath() + "/forwardSoftLimitEnabled",
                    this.isForwardSoftLimitEnabled());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/forwardSoftLimitThreshold",
                    this.getForwardSoftLimitThreshold());
            SmartDashboard.putString(this.getSmartDashboardPath() + "/invertType", this.getInvertType().toString());
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
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/nominalOutputForward",
                    this.getNominalOutputForward());
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/nominalOutputReverse",
                    this.getNominalOutputReverse());
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
            SmartDashboard.putNumber(this.getSmartDashboardPath() + "/statusFrame", this.getStatusFrame());
            if (Robot.isReal()) {
                SmartDashboard.putString(this.getSmartDashboardPath() + "/statusFrameType",
                        this.getStatusFrameType().toString());
                SmartDashboard.putNumber(this.getSmartDashboardPath() + "/timeout", this.getTimeout());
                SmartDashboard.putString(this.getSmartDashboardPath() + "/velocityMeasurementPeriod",
                        this.getVelocityMeasurementPeriod().toString());
                SmartDashboard.putNumber(this.getSmartDashboardPath() + "/velocityMeasurementWindow",
                        this.getVelocityMeasurementWindow());
                SmartDashboard.putNumber(this.getSmartDashboardPath() + "/voltageCompensationSaturation",
                        this.getVoltageCompensationSaturation());
            }

        }

    }

    ///////////////////////////////////////////////////////////////////////////
    /**
     * A direct reference to the TalonSRX motor, designed for direct control
     */
    public WPI_TalonFX motor;

    /**
     * The master that will be followed
     */
    public FRCTalonFX master;
    ///////////////////////////////////////////////////////////////////////////

    public TalonFXSensorCollection m_sensorCollection;
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
     * The type of status frame
     */
    private StatusFrameEnhanced statusFrameType;

    /**
     * The status frame of the motor
     */
    private int statusFrame;

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
     * The forward nominal output
     */
    private double nominalOutputForward;

    /**
     * The reverse nominal output
     */
    private double nominalOutputReverse;

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
     * The saturation for voltage compensation
     */
    private double voltageCompensationSaturation;

    /**
     * The measurement period for velocity control
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

    /**
     * Is continuous or discontinuous feedback enabled
     */
    private boolean feedbackNotContinuous;

    public void updatePIDController() {
        motor.config_kP(0, this.getkP());
        motor.config_kI(0, this.getkI());
        motor.config_kD(0, this.getkD());
        motor.config_kF(0, this.getkF());

    

        System.out.println("Wrote PID TO "+motor.getDeviceID() + " KF VALUE " + this.getkF());

    }

    public FRCTalonFX configure() {
        motor = new WPI_TalonFX(this.getCanID());

        m_sensorCollection = motor.getSensorCollection();
        motor.configFactoryDefault();
        motor.setSafetyEnabled(false);

        motor.selectProfileSlot(0, 0);
        motor.setSafetyEnabled(false);
        if (this.isInvertedWithType()) {
            motor.setInverted(this.invertType);
            System.out.println("Configuring Inverted");
        }
        if (this.isCurrentLimitEnabled()) {

        }
        if (this.isFeedbackNotContinuous()) {
            motor.configFeedbackNotContinuous(this.isFeedbackNotContinuous(), this.getTimeout());
            System.out.println("Configuring Feedback Continuity");
        }

        if (this.isForwardSoftLimitEnabled()) {
            motor.configForwardSoftLimitEnable(this.isForwardSoftLimitEnabled());
            motor.configForwardSoftLimitThreshold(this.getForwardSoftLimitThreshold());
            System.out.println("Configuring forward soft limit");
        }
        if (this.getMotionAcceleration() != 0) {
            motor.configMotionAcceleration(this.getMotionAcceleration());
            motor.configMotionCruiseVelocity(this.getMotionCruiseVelocity());
            System.out.println("Configuring acceleration");
        }
        if (this.getNeutralMode() != null) {
            motor.setNeutralMode(this.getNeutralMode());
            System.out.println("Setting Neutral Mode");
        }
        if (this.getNominalOutputForward() != 0 || this.getNominalOutputReverse() != 0) {
            motor.configNominalOutputForward(this.getNominalOutputForward());
            motor.configNominalOutputReverse(this.getNominalOutputReverse());
            System.out.println("Setting Nominal Output");
        }

        if (this.getOpenLoopRampRate() != 0) {
            motor.configOpenloopRamp(this.getOpenLoopRampRate());
            System.out.println("Setting Open Loop Ramp Rate");

        }

        if (this.getPeakOutputForward() != 0 || this.getPeakOutputReverse() != 0) {
            motor.configPeakOutputForward(this.getPeakOutputForward());
            motor.configPeakOutputReverse(this.getPeakOutputReverse());
            System.out.println("Setting Peak Output");
        }

        if (this.isReverseSoftLimitEnabled()) {
            motor.configReverseSoftLimitEnable(this.isReverseSoftLimitEnabled());
            motor.configReverseSoftLimitThreshold(this.getReverseSoftLimitThreshold());
            System.out.println("setting reverse soft limit enabled");
        }
        if (this.isSensorPhase()) {
            motor.setSensorPhase(this.isSensorPhase());
            System.out.println("setting sensor phase");

        }
        if (this.getStatusFrame() != 0) {
            motor.setStatusFramePeriod(this.getStatusFrameType(), this.getStatusFrame());
            System.out.println("Setting Frame Period");

        }
        if (this.getVelocityMeasurementPeriod() != null || this.getVelocityMeasurementWindow() != 0) {
            motor.configVelocityMeasurementPeriod(this.getVelocityMeasurementPeriod());
            motor.configVelocityMeasurementWindow(this.getVelocityMeasurementWindow());
            System.out.println("Setting Velocity Measurement Period");
        }
        if (this.getVoltageCompensationSaturation() != 0) {
            motor.configVoltageCompSaturation(this.getVoltageCompensationSaturation());
            System.out.println("Setting Saturation");

        }
        System.out.println("MOTOR "+this.motor.getDeviceID()+ "KF "+ this.getkF());
        if (this.getkP() != 0 || this.getkI() != 0 || this.getkD() != 0 || this.getkF() != 0) {
            updatePIDController();
            System.out.println("Setting PID Controller");
        }
        if (this.master != null) {
            motor.follow(master.motor);
        }
        return this;
    }

    public WPI_TalonFX getMotor() {
        return motor;
    }

    public void setMotor(WPI_TalonFX motor) {
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

    public StatusFrameEnhanced getStatusFrameType() {
        return statusFrameType;
    }

    public void setStatusFrameType(StatusFrameEnhanced statusFrameType) {
        this.statusFrameType = statusFrameType;
    }

    public int getStatusFrame() {
        return statusFrame;
    }

    public void setStatusFrame(int statusFrame) {
        this.statusFrame = statusFrame;
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

    public double getNominalOutputForward() {
        return nominalOutputForward;
    }

    public void setNominalOutputForward(double nominalOutputForward) {
        this.nominalOutputForward = nominalOutputForward;
    }

    public double getNominalOutputReverse() {
        return nominalOutputReverse;
    }

    public void setNominalOutputReverse(double nominalOutputReverse) {
        this.nominalOutputReverse = nominalOutputReverse;
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

    public double getVoltageCompensationSaturation() {
        return voltageCompensationSaturation;
    }

    public void setVoltageCompensationSaturation(double voltageCompensationSaturation) {
        this.voltageCompensationSaturation = voltageCompensationSaturation;
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

    public boolean isFeedbackNotContinuous() {
        return feedbackNotContinuous;
    }

    public void setFeedbackNotContinuous(boolean feedbackNotContinuous) {
        this.feedbackNotContinuous = feedbackNotContinuous;
    }

    public void setMaster(FRCTalonFX master) {
        this.master = master;
    }

    public static final class FRCTalonFXBuilder {
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
        private StatusFrameEnhanced statusFrameType = StatusFrameEnhanced.Status_3_Quadrature;
        private int statusFrame = 0;
        private boolean currentLimitEnabled = false;
        private int currentLimit = 0;
        private NeutralMode neutralMode = NeutralMode.Coast;
        private boolean smartDashboardPutEnabled = false;
        private String smartDashboardPath;
        private double openLoopRampRate = 0;
        private double closedLoopRampRate = 0;
        private double nominalOutputForward = 0;
        private double nominalOutputReverse = 0;
        private double peakOutputForward = 1.0;
        private double peakOutputReverse = -1.0;
        private double neutralDeadband = 0.04;
        private double voltageCompensationSaturation = 0;
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
        private boolean feedbackNotContinuous = false;
        private FRCTalonFX master;

        public FRCTalonFXBuilder(int canID) {
            this.canID = canID;
            this.smartDashboardPath = "TalonFX_" + canID;
        }

        // public static FRCTalonFXBuilder aFRCTalonFX() {
        // return new FRCTalonFXBuilder();
        // }

        public FRCTalonFXBuilder withCanID(int canID) {
            this.canID = canID;
            return this;
        }

        public FRCTalonFXBuilder withInverted(InvertType invert) {
            this.invertType = invert;
            return this;
        }

        public FRCTalonFXBuilder withFeedbackPort(int feedbackPort) {
            this.feedbackPort = feedbackPort;
            return this;
        }

        public FRCTalonFXBuilder withTimeout(int timeout) {
            this.timeout = timeout;
            return this;
        }

        public FRCTalonFXBuilder withSensorPhase(boolean sensorPhase) {
            this.sensorPhase = sensorPhase;
            return this;
        }

        public FRCTalonFXBuilder withKP(double kP) {
            this.kP = kP;
            return this;
        }

        public FRCTalonFXBuilder withKI(double kI) {
            this.kI = kI;
            return this;
        }

        public FRCTalonFXBuilder withKD(double kD) {
            this.kD = kD;
            return this;
        }

        public FRCTalonFXBuilder withKF(double kF) {
            this.kF = kF;
            return this;
        }

        public FRCTalonFXBuilder withAllowableClosedLoopError(int allowableClosedLoopError) {
            this.allowableClosedLoopError = allowableClosedLoopError;
            return this;
        }

        public FRCTalonFXBuilder withStatusFrameType(StatusFrameEnhanced statusFrameType) {
            this.statusFrameType = statusFrameType;
            return this;
        }

        public FRCTalonFXBuilder withStatusFrame(int statusFrame) {
            this.statusFrame = statusFrame;
            return this;
        }

        public FRCTalonFXBuilder withCurrentLimitEnabled(boolean currentLimitEnabled) {
            this.currentLimitEnabled = currentLimitEnabled;
            return this;
        }

        public FRCTalonFXBuilder withCurrentLimit(int currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public FRCTalonFXBuilder withNeutralMode(NeutralMode neutralMode) {
            this.neutralMode = neutralMode;
            return this;
        }

        public FRCTalonFXBuilder withSmartDashboardPutEnabled(boolean smartDashboardPutEnabled) {
            this.smartDashboardPutEnabled = smartDashboardPutEnabled;
            return this;
        }

        public FRCTalonFXBuilder withSmartDashboardPath(String smartDashboardPath) {
            this.smartDashboardPath = smartDashboardPath;
            return this;
        }

        public FRCTalonFXBuilder withOpenLoopRampRate(double openLoopRampRate) {
            this.openLoopRampRate = openLoopRampRate;
            return this;
        }

        public FRCTalonFXBuilder withClosedLoopRampRate(double closedLoopRampRate) {
            this.closedLoopRampRate = closedLoopRampRate;
            return this;
        }

        public FRCTalonFXBuilder withNominalOutputForward(double nominalOutputForward) {
            this.nominalOutputForward = nominalOutputForward;
            return this;
        }

        public FRCTalonFXBuilder withNominalOutputReverse(double nominalOutputReverse) {
            this.nominalOutputReverse = nominalOutputReverse;
            return this;
        }

        public FRCTalonFXBuilder withPeakOutputForward(double peakOutputForward) {
            this.peakOutputForward = peakOutputForward;
            return this;
        }

        public FRCTalonFXBuilder withPeakOutputReverse(double peakOutputReverse) {
            this.peakOutputReverse = peakOutputReverse;
            return this;
        }

        public FRCTalonFXBuilder withNeutralDeadband(double neutralDeadband) {
            this.neutralDeadband = neutralDeadband;
            return this;
        }

        public FRCTalonFXBuilder withVoltageCompensationSaturation(double voltageCompensationSaturation) {
            this.voltageCompensationSaturation = voltageCompensationSaturation;
            return this;
        }

        public FRCTalonFXBuilder withVelocityMeasurementPeriod(SensorVelocityMeasPeriod velocityMeasurementPeriod) {
            this.velocityMeasurementPeriod = velocityMeasurementPeriod;
            return this;
        }

        public FRCTalonFXBuilder withVelocityMeasurementWindow(int velocityMeasurementWindow) {
            this.velocityMeasurementWindow = velocityMeasurementWindow;
            return this;
        }

        public FRCTalonFXBuilder withForwardSoftLimitEnabled(boolean forwardSoftLimitEnabled) {
            this.forwardSoftLimitEnabled = forwardSoftLimitEnabled;
            return this;
        }

        public FRCTalonFXBuilder withForwardSoftLimitThreshold(int forwardSoftLimitThreshold) {
            this.forwardSoftLimitThreshold = forwardSoftLimitThreshold;
            return this;
        }

        public FRCTalonFXBuilder withReverseSoftLimitEnabled(boolean reverseSoftLimitEnabled) {
            this.reverseSoftLimitEnabled = reverseSoftLimitEnabled;
            return this;
        }

        public FRCTalonFXBuilder withReverseSoftLimitThreshold(int reverseSoftLimitThreshold) {
            this.reverseSoftLimitThreshold = reverseSoftLimitThreshold;
            return this;
        }

        public FRCTalonFXBuilder withAuxPIDPolarity(boolean auxPIDPolarity) {
            this.auxPIDPolarity = auxPIDPolarity;
            return this;
        }

        public FRCTalonFXBuilder withMotionCruiseVelocity(int motionCruiseVelocity) {
            this.motionCruiseVelocity = motionCruiseVelocity;
            return this;
        }

        public FRCTalonFXBuilder withMotionAcceleration(int motionAcceleration) {
            this.motionAcceleration = motionAcceleration;
            return this;
        }

        public FRCTalonFXBuilder withMotionCurveStrength(int motionCurveStrength) {
            this.motionCurveStrength = motionCurveStrength;
            return this;
        }

        public FRCTalonFXBuilder withMotionProfileTrajectoryPeriod(int motionProfileTrajectoryPeriod) {
            this.motionProfileTrajectoryPeriod = motionProfileTrajectoryPeriod;
            return this;
        }

        public FRCTalonFXBuilder withFeedbackNotContinuous(boolean feedbackNotContinuous) {
            this.feedbackNotContinuous = feedbackNotContinuous;
            return this;
        }

        public FRCTalonFXBuilder withMaster(FRCTalonFX master) {
            this.master = master;
            return this;
        }

        public FRCTalonFX build() {
            FRCTalonFX fRCTalonFX = new FRCTalonFX();
            fRCTalonFX.setCanID(canID);
            fRCTalonFX.setInverted(invertType);
            fRCTalonFX.setFeedbackPort(feedbackPort);
            fRCTalonFX.setTimeout(timeout);
            fRCTalonFX.setSensorPhase(sensorPhase);
            fRCTalonFX.setAllowableClosedLoopError(allowableClosedLoopError);
            fRCTalonFX.setStatusFrameType(statusFrameType);
            fRCTalonFX.setStatusFrame(statusFrame);
            fRCTalonFX.setCurrentLimitEnabled(currentLimitEnabled);
            fRCTalonFX.setCurrentLimit(currentLimit);
            fRCTalonFX.setNeutralMode(neutralMode);
            fRCTalonFX.setSmartDashboardPutEnabled(smartDashboardPutEnabled);
            fRCTalonFX.setSmartDashboardPath(smartDashboardPath);
            fRCTalonFX.setOpenLoopRampRate(openLoopRampRate);
            fRCTalonFX.setClosedLoopRampRate(closedLoopRampRate);
            fRCTalonFX.setNominalOutputForward(nominalOutputForward);
            fRCTalonFX.setNominalOutputReverse(nominalOutputReverse);
            fRCTalonFX.setPeakOutputForward(peakOutputForward);
            fRCTalonFX.setPeakOutputReverse(peakOutputReverse);
            fRCTalonFX.setNeutralDeadband(neutralDeadband);
            fRCTalonFX.setVoltageCompensationSaturation(voltageCompensationSaturation);
            fRCTalonFX.setVelocityMeasurementPeriod(velocityMeasurementPeriod);
            fRCTalonFX.setVelocityMeasurementWindow(velocityMeasurementWindow);
            fRCTalonFX.setForwardSoftLimitEnabled(forwardSoftLimitEnabled);
            fRCTalonFX.setForwardSoftLimitThreshold(forwardSoftLimitThreshold);
            fRCTalonFX.setReverseSoftLimitEnabled(reverseSoftLimitEnabled);
            fRCTalonFX.setReverseSoftLimitThreshold(reverseSoftLimitThreshold);
            fRCTalonFX.setAuxPIDPolarity(auxPIDPolarity);
            fRCTalonFX.setMotionCruiseVelocity(motionCruiseVelocity);
            fRCTalonFX.setMotionAcceleration(motionAcceleration);
            fRCTalonFX.setMotionCurveStrength(motionCurveStrength);
            fRCTalonFX.setMotionProfileTrajectoryPeriod(motionProfileTrajectoryPeriod);
            fRCTalonFX.setFeedbackNotContinuous(feedbackNotContinuous);
            fRCTalonFX.setMaster(master);
            fRCTalonFX.kF = this.kF;
            fRCTalonFX.kD = this.kD;
            fRCTalonFX.kI = this.kI;
            fRCTalonFX.kP = this.kP;
            return fRCTalonFX.configure();
        }
    }

}