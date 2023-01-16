package frc.robot.subsystems.motors;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningMotor extends Sendable {
	double get();
    void set(double output);
}