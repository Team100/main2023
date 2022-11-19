// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FRCLib.Motors.FRCTalonFX;

public class arm extends SubsystemBase {
  private FRCTalonFX upyDowny, leftyRighty, spinnyBoi;
  /** Creates a new arm. */
  public arm() {
    upyDowny = new FRCTalonFX.FRCTalonFXBuilder(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.CAN_ID)
            .withKP(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KP)
            .withKI(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KI)
            .withKD(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KD)
            .withKF(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KF)
            .withInverted(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.INVERTED)
            .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.SENSOR_PHASE)
            .withPeakOutputForward(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.PEAK_OUTPUT_FORWARD)
            .withPeakOutputReverse(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.PEAK_OUTPUT_REVERSE)
            //.withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.NEUTRAL_MODE)
            .build();
    leftyRighty = new FRCTalonFX.FRCTalonFXBuilder(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.CAN_ID)
            .withKP(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.KP)
            .withKI(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.KI)
            .withKD(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.KD)
            .withKF(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.KF)
            .withInverted(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.INVERTED)
            .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.SENSOR_PHASE)
            .withPeakOutputForward(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.PEAK_OUTPUT_FORWARD)
            .withPeakOutputReverse(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.PEAK_OUTPUT_REVERSE)
            //.withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.RightMaster.NEUTRAL_MODE)
            .build();
    spinnyBoi = new FRCTalonFX.FRCTalonFXBuilder(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.CAN_ID)
            .withKP(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.KP)
            .withKI(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.KI)
            .withKD(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.KD)
            .withKF(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.KF)
            .withInverted(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.INVERTED)
            .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.SENSOR_PHASE)
            .withPeakOutputForward(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.PEAK_OUTPUT_FORWARD)
            .withPeakOutputReverse(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.PEAK_OUTPUT_REVERSE)
            // .withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.RightFollower.NEUTRAL_MODE)
            .build();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
