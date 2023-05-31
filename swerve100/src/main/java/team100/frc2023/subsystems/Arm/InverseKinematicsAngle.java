// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team100.frc2023.subsystems.Arm;

/** Add your docs here. */
public class InverseKinematicsAngle {
    public double lowerTheta;
    public double upperTheta;

    public InverseKinematicsAngle(double upperTheta, double lowerTheta) {
        this.lowerTheta = lowerTheta;
        this.upperTheta = upperTheta;
    }

    
    public InverseKinematicsAngle() {
        this.lowerTheta = 100;
        this.upperTheta = 100;
    }

}
