package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;

public interface TurningMotor extends Sendable {
	double get();
    void set(double output);
}