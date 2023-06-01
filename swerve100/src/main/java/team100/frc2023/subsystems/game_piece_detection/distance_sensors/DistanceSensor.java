package team100.frc2023.subsystems.game_piece_detection.distance_sensors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DistanceSensor extends SubsystemBase {
    public abstract double getCentimeters();
}
