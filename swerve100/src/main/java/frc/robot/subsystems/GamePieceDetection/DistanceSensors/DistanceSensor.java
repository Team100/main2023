package frc.robot.subsystems.GamePieceDetection.DistanceSensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DistanceSensor extends SubsystemBase {
    public abstract double getCentimeters();
}
