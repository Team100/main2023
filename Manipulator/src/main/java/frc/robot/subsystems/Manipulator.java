// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCTalonFX;
import frc.robot.FRCLib.Motors.FRCTalonSRX;
import frc.robot.FRCLib.Motors.FRCTalonSRX.FRCTalonSRXBuilder;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  FRCTalonSRX pinch;
  public Manipulator() {
    pinch = new FRCTalonSRXBuilder(1)
    // .withKP(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KP)
    // .withKI(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KI)
    // .withKD(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KD)
    // .withKF(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KF)
    .withInverted(false)
    // .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.SENSOR_PHASE)
    .withPeakOutputForward(0.5)
    .withPeakOutputReverse(-0.5)
    //.withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.NEUTRAL_MODE)
    //.withCurrentLimitEnabled(true)
    //.withCurrentLimit(5)

    .build();

    pinch.motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pinch.getAppliedOutput());

    // System.out.println(pinch.getSelectedSensorPosition());
    SmartDashboard.putData("Manipulator", this);
  }

  public void pinch(double d){
    pinch.drivePercentOutput(-0.6*d);

  }
 
  public double getEncoderPosition(){
    return pinch.getSelectedSensorPosition();
  }
  
  public boolean getInnerLimitSwitch(){
    return pinch.motor.isFwdLimitSwitchClosed()==1;
  }

  public boolean getOuterLimitSwitch(){
    return pinch.motor.isRevLimitSwitchClosed()==1;
  }

  public void configSoftLimits(double innerSoftLimit, double outerSoftLimit){ 
    pinch.motor.configReverseSoftLimitThreshold(innerSoftLimit); 
    pinch.motor.configForwardSoftLimitThreshold(outerSoftLimit);
    pinch.motor.configReverseSoftLimitEnable(true);
    pinch.motor.configForwardSoftLimitEnable(true);
  }


  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Encoder", () -> pinch.getSelectedSensorPosition(), null);
    builder.addBooleanProperty("Inner Limit Switch", () -> { return pinch.motor.isRevLimitSwitchClosed() == 1; }, null);
    builder.addBooleanProperty("Outer Limit Switch", () -> { return pinch.motor.isFwdLimitSwitchClosed() == 1; }, null);
  }

}
