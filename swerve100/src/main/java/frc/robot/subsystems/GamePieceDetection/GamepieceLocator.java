// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.GamePieceDetection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.GamePieceDetection.DistanceSensors.DistanceSensor;
import frc.robot.subsystems.GamePieceDetection.DistanceSensors.NTDistanceSensor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamepieceLocator extends SubsystemBase {
    private final double kSeparationWidth = 40;
    private final DistanceSensor leftSensor;
    private final DistanceSensor rightSensor;

    private double offset;
    private boolean hasGamepiece;

    /** Creates a new GamepieceLocator. */
    public GamepieceLocator() {
        leftSensor = new NTDistanceSensor("distance_a");
        rightSensor = new NTDistanceSensor("distance_b");

        SmartDashboard.putData("Gamepiece Locator", this);

        addChild("Left Sensor", leftSensor);
        addChild("Right Sensor", rightSensor);
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in centimeters.
     * Positive is towards the right
     * @return offset in centimeters (defaults to 0 if no gamepiece is detected)
     */
    public double getOffsetCentimeters() {
        return offset;
    }

    /**
     * Get the offset of the gamepiece from the center of the manipulator in meters.
     * In robot coords (y-axis), so positive is towards the left
     * @return offset in meters (defaults to 0 if no gamepiece is detected)
     */
    public double getOffsetMeters() {
        return getOffsetCentimeters() / 100;
    }

    /**
     * Whether or not a gamepiece has been detected.
     * @return true if a gamepiece is detected
     */
    public boolean hasGamepiece() {
        return hasGamepiece;
    }

    @Override
    public void periodic() {
        // Read sensors
        double left = leftSensor.getCentimeters();
        double right = rightSensor.getCentimeters();

        // If the sensors detect something outside of their rated range, the Pi will return -1.
        // Also will be -2 if the Pi is not on NetworkTables. Detect those cases here
        if (left < 0 || right < 0) {
            offset = 0;
            hasGamepiece = false;
            // TODO add error state tracking
        }

        // If the sensors give us a distance larger than the manipulator, we can assume they are
        // interfering with each other. This is likely because there is no gamepiece in the manipulator
        else if (left > kSeparationWidth || right > kSeparationWidth || left + right > kSeparationWidth) {
            offset = 0;
            hasGamepiece = false;
        }

        // We have a gamepiece!
        else {
            offset = (right - left) / 2;
            hasGamepiece = true;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("offset_cm", () -> this.getOffsetCentimeters(), null);
        builder.addDoubleProperty("offset_m", () -> this.getOffsetMeters(), null);
        builder.addBooleanProperty("hasGamepiece", () -> this.hasGamepiece(), null);
    }
}
