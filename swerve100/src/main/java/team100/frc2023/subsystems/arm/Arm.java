package team100.frc2023.subsystems.arm;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team100.lib.motors.FRCNEO;

/** Add your docs here. */
public class Arm extends SubsystemBase {

    private final AnalogEncoder encoder0 = new AnalogEncoder(5);
    private final AnalogEncoder encoder1 = new AnalogEncoder(4);

    public FRCNEO upperArm;
    public FRCNEO lowerArm;

    public UpperArm upperArmSubsytem;
    public LowerArm lowerArmSubsytem;

    DecimalFormat df;

    public Arm() {

        df = new DecimalFormat("000.00");

        upperArm = new FRCNEO.FRCNEOBuilder(42)
                .withInverted(false)
                // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(30)
                // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
                .withPeakOutputForward(0.3)
                .withPeakOutputReverse(-0.3)
                .withNeutralMode(IdleMode.kBrake)
                .withForwardSoftLimitEnabled(false)
                .build();

        lowerArm = new FRCNEO.FRCNEOBuilder(43)
                .withInverted(true)
                // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
                .withSensorPhase(false)
                .withTimeout(10)
                .withCurrentLimitEnabled(true)
                .withCurrentLimit(40)
                // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
                .withPeakOutputForward(40)
                .withPeakOutputReverse(-0.3)
                .withNeutralMode(IdleMode.kBrake)
                .build();

        // encoder1.setDistancePerRotation( 360);
        encoder0.setDistancePerRotation(360);

        // encoder0.setPositionOffset(0.15);

        upperArmSubsytem = new UpperArm(() -> getUpperArm(), upperArm);
        lowerArmSubsytem = new LowerArm(() -> getLowerArm(), lowerArm);

        SmartDashboard.putData("Arm", this);

    }

    /**
     * The angle the upper arm is at (in degrees)
     * 90 degrees is pointing directly forward, and 180 degrees is straight up.
     * @return upper arm angle
     */
    public double getUpperArm() {
        double x = (1 - encoder0.getAbsolutePosition()) * 350 + 13;
        double formatted = Math.round(x);

        return formatted;
    }

    // public double get(){
    // return Double.parseDouble(df.format());

    // return encoder0.get();
    // }

    public void reset() {
        encoder0.reset();
    }

    /**
     * The angle the lower arm is at (in degrees)
     * 0 degrees is pointing straight up, increasing when the arm is pointing forward.
     * Will be negative when the arm is pointing backwards.
     * @return lower arm angle
     */
    public double getLowerArm() {
        double x = (1 - encoder1.getAbsolutePosition()) * 360 - 101;
        // double formatted = Math.round(x);

        return x;
    }

    public void drive(double x, double y) {
        upperArm.drivePercentOutput(x);
        lowerArm.drivePercentOutput(y);
    }

    @Override
    public void periodic() {

    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Upper Arm", () -> getUpperArm(), null);
        builder.addDoubleProperty("Lower Arm", () -> getLowerArm(), null);
        builder.addDoubleProperty("Upper Arm Get", () -> encoder0.getAbsolutePosition(), null);
        builder.addDoubleProperty("Lower Arm Get", () -> encoder1.get(), null);
        builder.addDoubleProperty("OUptut", () -> upperArm.motor.get(), null);

    }

}
