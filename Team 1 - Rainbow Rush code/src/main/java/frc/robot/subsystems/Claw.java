// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//nightmare

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FRCLib.Motors.FRCNEO;

public class Claw extends SubsystemBase {
  public FRCNEO pinchyBoi;

  /** Creates a new claw. */
  public Claw() {
    pinchyBoi = new FRCNEO.FRCNEOBuilder(1)
            .withInverted(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.INVERT)
            .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
            .withSensorPhase(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.SENSOR_PHASE)
            .withTimeout(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.TIMEOUT)
            .withCurrentLimitEnabled(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.ENABLE_CURRENT_LIMIT)
            .withCurrentLimit(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.CURRENT_LIMIT)
            .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
            .withPeakOutputForward(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.PEAK_OUTPUT_FORWARD)
            .withPeakOutputReverse(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.PEAK_OUTPUT_REVERSE)
            // .withNeutralMode(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.NEUTRAL_MODE)
            .build();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double x){
      pinchyBoi.drivePercentOutput(x);
  }


  public void getPinchyBoi(){
    pinchyBoi.getSelectedSensorPosition();
}

  
}

//how often do you consider the moral neccicities of a concious mind?z