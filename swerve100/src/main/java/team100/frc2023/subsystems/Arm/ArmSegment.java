package team100.frc2023.subsystems.Arm;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team100.lib.motors.FRCNEO;

public class ArmSegment extends SubsystemBase{
   

    private FRCNEO motor;
    private Supplier<Double> positionSupplier;
    

    /** Creates a new UpperArm. */
    public ArmSegment(Supplier<Double> positionSupplier, FRCNEO motor, String name) {
        this.motor = motor;
        this.positionSupplier = positionSupplier;

        SmartDashboard.putData(name, this);


    }

    /**
     * Sets the motor output, accounting for soft limits.
     * <p><b>THIS SHOULD BE THE ONLY PLACE WHERE MOTOR OUTPUT IS SET.</b></p>
     * @param x
     */
    public void setMotor(double x) {
        // Account for soft limits
        // kGravityGain *= Math.sin(positionSupplier.get());
        // if (getAngle() <= lowerSoftLimit && x < 0)
        //     this.motor.motor.set(0);
        // else if (getAngle() >= upperSoftLimit && x > 0)
        //     this.motor.motor.set(0);
        // else
            this.motor.motor.set(x) ;
    }

    @Override
    public void periodic() {
        
    }


    public double getDegrees() {
        return positionSupplier.get() * 180/Math.PI;
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // builder.addDoubleProperty("Error", () -> m_controller.getPositionError(), null);
        // builder.addDoubleProperty("Measurement", () -> getMeasurement(), null);
        // builder.addDoubleProperty("Setpoint", () -> m_controller.getSetpoint().position, null);
        // builder.addDoubleProperty("Goal", () -> m_controller.getGoal().position, null);
        // builder.addDoubleProperty("Output", () -> output, null);
        // builder.addDoubleProperty("Degrees", () -> getDegrees(), null);




    }
}