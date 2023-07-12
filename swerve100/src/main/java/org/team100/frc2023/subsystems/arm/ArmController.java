package org.team100.frc2023.subsystems.arm;

import org.team100.lib.motors.FRCNEO;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmController extends SubsystemBase {
    public static final double coneSubVal = 1.308745;
    public static final double softStop = -0.594938;

    // private static final double kArmMaxXCoordinate = 1.4;
    // private static final double kArmMaxYCoordinate = 1.4;

    private double xSetpoint = 1;
    private double ySetpoint = 1;

    public boolean cubeMode = true;

    private double upperAngleSetpoint = 0;
    private double lowerAngleSetpoint = 0;

    // Lower arm objects
    public final ArmSegment lowerArmSegment;
    private final FRCNEO lowerArmMotor;
    private final AnalogEncoder lowerArmEncoder = new AnalogEncoder(6);

    // Upper arm objects
    public final ArmSegment upperArmSegment;
    private final FRCNEO upperArmMotor;
    private final AnalogEncoder upperArmEncoder = new AnalogEncoder(5);

    public ArmController() {
        // TrapezoidProfile.Const raints constraints = new TrapezoidProfile.Constraints(
        // 0.3, // velocity rad/s
        // 3.0 // accel rad/s^2
        // );
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

        lowerArmSegment = new ArmSegment(this::getLowerArm, lowerArmMotor, "Lower Motor");
        upperArmSegment = new ArmSegment(this::getUpperArm, upperArmMotor, "Upper Motor");

        Translation2d initial = ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());

        xSetpoint = initial.getX();
        ySetpoint = initial.getY();

        SmartDashboard.putData("Arm Subsystem", this);

    }

    /**
     * Uses x and y velocity from joystick to control the arm. Should be used for
     * manual control
     * <p>
     * The coordinate grid is cartesian but rotated 90 degrees CCW.
     * Positive X is upwards and positive Y is forward.
     * </p>
     * 
     * @param x vertical axis coordinate
     * @param y horizontal axis coordinate
     */
    public ArmAngles manualSetpoint(XboxController m_driverController) {

        double dx = -m_driverController.getLeftY();
        double dy = m_driverController.getRightX();

        if (Math.abs(dx) <= 0.15) {
            dx = 0;
        } else {
            dx = (dx - 0.15) / 0.85;
        }

        if (Math.abs(dy) <= 0.15) {
            dy = 0;
        } else {
            dy = (dy - 0.15) / 0.85;
        }

        Translation2d current = ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());

        dx *= 0.2;
        dy *= 0.2;

        double x = current.getX() + dx;
        double y = current.getY() + dy;

        double[] coords = { x, y };

        double[] angles = ArmKinematics.algorithm2RIKS(coords[0], coords[1]);

        if (angles == null) {
            return new ArmAngles();
        }

        upperAngleSetpoint = angles[0];
        lowerAngleSetpoint = angles[1];

        xSetpoint = coords[0];
        ySetpoint = coords[1];
        return new ArmAngles(angles[0], angles[1]); // upper theta, lower theta
    }

    public void resetSetpoint() {
        Translation2d current = ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());
        xSetpoint = current.getX();
        ySetpoint = current.getY();
    }

    public void driveManually(double upperSpeed, double lowerSpeed) {
        if (Math.abs(upperSpeed) <= 0.15) {
            upperSpeed = 0;
        }
        if (Math.abs(lowerSpeed) <= 0.15) {
            lowerSpeed = 0;
        }

        upperArmMotor.drivePercentOutput(upperSpeed);
        lowerArmMotor.drivePercentOutput(lowerSpeed);
    }

    public Translation2d getPose() {
        return ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());
    }

    public ArmAngles calculate(double x, double y) {
        double[] angles = ArmKinematics.algorithm2RIKS(x, y);
        return new ArmAngles(angles[0], angles[1]); // upper theta, lower theta
    }

    /**
     * The angle the lower arm is at (in radians)
     * 0 is pointing straight up, increasing when the arm is pointing forward.
     * Will be negative when the arm is pointing backwards.
     * 
     * @return lower arm angle
     */
    public double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.861614) * 360;
        double formatted = x;
        // * Math.PI / 180;
        return (-1.0 * formatted) * Math.PI / 180;
    }

    /**
     * The angle the upper arm is at (in radians)
     * pi/2 rads is pointing directly forward, and pi rads is straight up.
     * 
     * @return upper arm angle
     */
    public double getUpperArm() {
        // double x = (upperArmEncoder.getAbsolutePosition() - 0.53) * 350 + 3;
        double x = (upperArmEncoder.getAbsolutePosition() - 0.266396) * 350;
        double formatted = x;

        return formatted * Math.PI / 180;
    }

    public ArmAngles getArmAngles() {
        return new ArmAngles(getUpperArm(), getLowerArm());
    }

    public double getLowerArmDegrees() {
        return getLowerArm() * 180 / Math.PI;
    }

    public double getUpperArmDegrees() {
        return getUpperArm() * 180 / Math.PI;
    }

    public void setLowerArm(double x) {

        if (getLowerArm() <= softStop && x < 0) {
            lowerArmMotor.motor.set(x);
        } else {
            lowerArmMotor.motor.set(0);

        }
        lowerArmMotor.motor.set(x);
    }

    public void setUpperArm(double x) {
        upperArmMotor.motor.set(x);
    }

    public void driveLowerArm(double x) {
        lowerArmMotor.motor.setVoltage(12 * x);
    }

    public void driveUpperArm(double x) {
        upperArmMotor.motor.set(12 * x);
    }

    public double getUpperArmRel() {
        return upperArmMotor.motor.getEncoder().getPosition();
    }

    public double getLowerArmRel() {
        return lowerArmMotor.getSelectedSensorPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ARM X", () -> ArmKinematics.getArmPosition(getLowerArm(), getUpperArm()).getX(),
                null);
        builder.addDoubleProperty("ARM Y", () -> ArmKinematics.getArmPosition(getLowerArm(), getUpperArm()).getY(),
                null);
        builder.addDoubleProperty("Upper Arm Absolute Angle", () -> upperArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Lower Arm Absolute Angle", () -> lowerArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Upper Arm Absolute Radians", () -> getUpperArm(), null);
        builder.addDoubleProperty("Lower Arm Absolute Rasians", () -> getLowerArm(), null);
        builder.addDoubleProperty("Upper Arm Absolute Degrees", () -> getUpperArmDegrees(), null);
        builder.addDoubleProperty("Lower Arm Absolute Degrees", () -> getLowerArmDegrees(), null);
        builder.addBooleanProperty("Cube Mode", () -> cubeMode, null);
        builder.addDoubleProperty("X Setpoint", () -> xSetpoint, null);
        builder.addDoubleProperty("Y Setpoint", () -> ySetpoint, null);
        builder.addDoubleProperty("Upper Angle Setpoint", () -> upperAngleSetpoint, null);
        builder.addDoubleProperty("Lower Angle Setpoint", () -> lowerAngleSetpoint, null);
        builder.addBooleanProperty("Cube Mode", () -> cubeMode, null);
        builder.addDoubleProperty("Upper Arm Relative", () -> getUpperArmRel(), null);

    }
}