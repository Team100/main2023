// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCTalonSRX;
import frc.robot.FRCLib.Motors.FRCTalonSRX.FRCTalonSRXBuilder;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  FRCTalonSRX pinch;
  public AnalogEncoder position;
  public PIDController pinchController;

  public Manipulator() {
    pinch = new FRCTalonSRXBuilder(10)
    // .withKP(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KP)
    // .withKI(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KI)
    // .withKD(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KD)
    // .withKF(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.KF)
    .withInverted(false)
    // .withSensorPhase(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.SENSOR_PHASE)
    .withPeakOutputForward(1)
    .withPeakOutputReverse(-1)
    //.withNeutralMode(Constants.DrivetrainConstants.DrivetrainMotors.LeftMaster.NEUTRAL_MODE)
    // .withCurrentLimitEnabled(true)
    //.withCurrentLimit(7)
    .build();

    pinch.motor.configPeakCurrentLimit(25, 1000);
    pinch.motor.configContinuousCurrentLimit(2);

    // pinch.motor.configPeakCurrentDuration(0)
    // pinch.motor.configPeakCurrentDuration(1000);

    pinch.motor.enableCurrentLimit(true);
    // pinch.motor.configCurrent

    

    

    position = new AnalogEncoder(6);

    position.reset();
    pinchController = new PIDController(0.2, 0, 0);

    SmartDashboard.putData("Manipulator", this);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(pinch.getAppliedOutput());

  }

  public void pinch(double d){
    pinch.motor.set(d);

  }

  public void pinchv2(double x, double y){
    if(x > 0){
      pinch.drivePercentOutput(x/4);
    }else if(y > 0){
      pinch.drivePercentOutput(-y/4);
    }else{
      pinch.drivePercentOutput(0);
    }

  }

  public double getPosition(){
    return position.get();
  }

  public boolean getForwardLimitSwitch(){
    if(pinch.motor.isFwdLimitSwitchClosed() == 1){
      return true;
    }else{
      return false;
    }
  }

  public boolean getReverseLimitSwitch(){
    if(pinch.motor.isRevLimitSwitchClosed() == 1){
      return true;
    }else{
      return false;
    }
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Position", () -> getPosition(), null );
    builder.addBooleanProperty("Fwd Limit Switch", () -> getForwardLimitSwitch(), null );
    builder.addBooleanProperty("Rev Limit Switch", () -> getReverseLimitSwitch(), null );
    builder.addDoubleProperty("Output Current", ()->pinch.motor.getStatorCurrent(), null);
    builder.addDoubleProperty("Input Current", ()->pinch.motor.getSupplyCurrent(), null);

  }

}
