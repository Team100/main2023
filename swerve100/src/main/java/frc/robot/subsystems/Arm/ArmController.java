package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCNEO;

public class ArmController extends SubsystemBase {

    private static final double kArmMaxXCoordinate = 1.4;
    private static final double kArmMaxYCoordinate = 1.4;

    private double xSetpoint = 1;
    private double ySetpoint = 1;

    public boolean cubeMode = true;

    private double upperAngleSetpoint = 0;
    private double lowerAngleSetpoint = 0;

    // Lower arm objects
    public ArmSegment lowerArmSegment;
    private FRCNEO lowerArmMotor;
    private final AnalogEncoder lowerArmEncoder = new AnalogEncoder(4);

    // Upper arm objects
    public ArmSegment upperArmSegment;
    private FRCNEO upperArmMotor;

    private final AnalogEncoder upperArmEncoder = new AnalogEncoder(5);
    
    public ArmController() {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            .3, // velocity rad/s
            3 // accel rad/s^2
        );
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
     * Uses x and y velocity from joystick to control the arm. Should be used for manual control
     * <p>The coordinate grid is cartesian but rotated 90 degrees CCW.
     * Positive X is upwards and positive Y is forward.</p>
     * @param x vertical axis coordinate
     * @param y horizontal axis coordinate
     */
    public InverseKinematicsAngle manualSetpoint(XboxController m_driverController) {

        
        double dx = -m_driverController.getLeftY();
        double dy = m_driverController.getRightX();

        if(Math.abs(dx) <= 0.15){
            dx = 0;
        } else {
            dx = (dx - 0.15) / 0.85;
        }

        if(Math.abs(dy) <= 0.15){
            dy = 0;
        } else {
            dy = (dy - 0.15) / 0.85;
        }

        

        Translation2d current = ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());


        dx *= 0.2;
        dy *= 0.2;

        double x = current.getX() + dx;
        double y = current.getY() + dy;

        double[] coords = {x, y};
        
        
        
        double[] angles = ArmKinematics.algorithm2RIKS(coords[0], coords[1]);

        if(angles == null){
          return new InverseKinematicsAngle();
        }

        upperAngleSetpoint = angles[0];
        lowerAngleSetpoint = angles[1];

        xSetpoint = coords[0];
        ySetpoint = coords[1];
        return new InverseKinematicsAngle(angles[0], angles[1]); //upper theta, lower theta
    }

    public void resetSetpoint() {
        Translation2d current = ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());
        xSetpoint = current.getX();
        ySetpoint = current.getY();
    }

    public void driveManually(double x, double y){
        if(Math.abs(x) <= 0.15){
            x = 0;
        }
        if(Math.abs(y) <= 0.15){
            y = 0;
        }  

        upperArmMotor.drivePercentOutput(x);
        lowerArmMotor.drivePercentOutput(y);
    }


    public Translation2d getPose(){
        return ArmKinematics.getArmPosition(getLowerArm(), getUpperArm());
    }


    public InverseKinematicsAngle calculate(double x, double y) {
        double[] angles = ArmKinematics.algorithm2RIKS(x, y);
        return new InverseKinematicsAngle(angles[0], angles[1]); //upper theta, lower theta        
    }


    /**
     * Clamps the coordinates to the max values.
     * TODO: Make sure the arm doesn't collide with itself or the robot
     * @param x
     * @param y
     * @return clamped coordinates
     */
    private double[] clampCoords(double x, double y) {
        double[] coords = new double[2];
        coords[0] = MathUtil.clamp(x, 0, kArmMaxXCoordinate);
        coords[1] = MathUtil.clamp(y, 0, kArmMaxYCoordinate);
        return coords;
    }

    /**
     * The angle the lower arm is at (in radians)
     * 0 is pointing straight up, increasing when the arm is pointing forward.
     * Will be negative when the arm is pointing backwards.
     * @return lower arm angle
     */
    public double getLowerArm() {
        double x = (lowerArmEncoder.getAbsolutePosition() - 0.617) * 360;
        double formatted = x;
        //* Math.PI / 180;
        return (-1 * formatted) * Math.PI/180;
    }

    /** 
     * The angle the upper arm is at (in radians)
     * pi/2 rads is pointing directly forward, and pi rads is straight up.
     * @return upper arm angle
     */
    public double getUpperArm() {
        // double x = (upperArmEncoder.getAbsolutePosition() - 0.53) * 350 + 3;
        double x = (upperArmEncoder.getAbsolutePosition()- 0.26) * 350 ;
        double formatted = x;

        return formatted * Math.PI/180 ;
    }


    public double getLowerArmDegrees(){
        return getLowerArm() * 180 / Math.PI;
    }

    public double getUpperArmDegrees(){
        return getUpperArm() * 180 / Math.PI;
    }

    public void setLowerArm(double x){
        lowerArmMotor.motor.set(x);
    }

    public void setUpperArm(double x){
        upperArmMotor.motor.set(x);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ARM X", () -> ArmKinematics.getArmPosition(getLowerArm(), getUpperArm()).getX(), null);
        builder.addDoubleProperty("ARM Y", () -> ArmKinematics.getArmPosition(getLowerArm(), getUpperArm()).getY(), null);
        builder.addDoubleProperty("Upper Arm Absolute Angle", () -> upperArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Lower Arm Absolute Angle", () -> lowerArmEncoder.getAbsolutePosition(), null);
        builder.addDoubleProperty("Upper Arm Absolute Radians", () -> getUpperArm(), null);
        builder.addDoubleProperty("Lower Arm Absolute Rasians", () -> getLowerArm(), null);
        builder.addDoubleProperty("Upper Arm Absolute Degrees", () -> getUpperArmDegrees(), null);
        builder.addDoubleProperty("Lower Arm Absolute Degrees", () -> getLowerArmDegrees(), null);
        builder.addBooleanProperty("Cube Mode", () -> cubeMode, null);
        // builder.addDoubleProperty("X Setpoint", () -> xSetpoint, null);
        // builder.addDoubleProperty("Y Setpoint", () -> ySetpoint, null);
        // builder.addDoubleProperty("Upper Angle Setpoint", () -> upperAngleSetpoint, null);
        // builder.addDoubleProperty("Lower Angle Setpoint", () -> lowerAngleSetpoint, null);
    }

}

class ArmKinematics {
    private static final double kUpperArmLength = 0.905;
    private static final double kLowerArmLength = 0.93;

    /**
     * Calculates the position of the arm based on the angles of the segments
     * @param lowerArmAngle
     * @param upperArmAngle
     * @return Translation2d object containing the position of the arm
     */
    public static Translation2d getArmPosition(double lowerArmAngle, double upperArmAngle) {
        // upperArmAngle += lowerArmAngle;
        double upperArmX = kUpperArmLength * Math.cos(upperArmAngle);
        double upperArmY = kUpperArmLength * Math.sin(upperArmAngle);

        double lowerArmX = kLowerArmLength * Math.cos(lowerArmAngle);
        double lowerArmY = kLowerArmLength * Math.sin(lowerArmAngle);

        return new Translation2d(upperArmX + lowerArmX, upperArmY + lowerArmY);
    }

    /**
     * Calculates the angles the arm should be at to reach the setpoint.
     * Accounts for the fact that our segment angles are independent.
     * @param x
     * @param y
     * @return array containing the angles [lowerArmAngle, upperArmAngle]
     */
    static double[] algorithm2RIK(double x, double y) {
        double[] angles = new double[2];

        double c2 = (x * x + y * y - kUpperArmLength * kUpperArmLength - kLowerArmLength * kLowerArmLength)
                / (2 * kUpperArmLength * kLowerArmLength);
        double s2 = Math.sqrt(1 - c2 * c2);
        angles[1] = Math.atan2(s2, c2);

        double k1 = kUpperArmLength + kLowerArmLength * c2;
        double k2 = kLowerArmLength * s2;
        angles[0] = Math.atan2(y, x) - Math.atan2(k2, k1);

        return angles;
    }

    static double[] algorithm2RIKS(double x, double y) {
        //https://www.youtube.com/watch?v=RH3iAmMsolo&t=7s

        
        double[] angles = new double[2];
        double xSquared = Math.pow(x, 2);
        double ySquared = Math.pow(y, 2);

        if(Math.sqrt(ySquared + xSquared) + 0.05 >= kLowerArmLength + kUpperArmLength){
            return null;
        }
        
        double upperLengthSquare = Math.pow(kUpperArmLength, 2);
        double lowerLengthSquare = Math.pow(kLowerArmLength, 2);
        double lengthSquared = upperLengthSquare + lowerLengthSquare;
         double q2 = Math.PI - Math.acos((lengthSquared - xSquared - ySquared)/(2*kUpperArmLength*kLowerArmLength));
        //maybe x/y try if it dosent work
        double q1 = Math.atan(y/x) - Math.atan( (kUpperArmLength*Math.sin(q2)) / (kLowerArmLength + kUpperArmLength*Math.cos(q2)) );

        angles[0] = q1 + q2; //upper theta
        angles[1] = q1; //lower theta

        return angles;  

    }

    
}