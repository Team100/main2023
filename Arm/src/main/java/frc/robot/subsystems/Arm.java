// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCNEO;

public class Arm extends ProfiledPIDSubsystem{
  /** Creates a new Arm. */
  public FRCNEO lowerArm;
  public FRCNEO upperArm;
  // public Spark lowerArmv2;

  

  PIDController lowerArmController = new PIDController(0.1, 0, 0);

  private final AnalogEncoder encoder0 = new AnalogEncoder(5);
  private final AnalogEncoder encoder1 = new AnalogEncoder(4);
  SlewRateLimiter limiter = new SlewRateLimiter(0.5);

  public Arm() {
    

    super(
        new ProfiledPIDController(
            0.1,
            0,
            0,
            new TrapezoidProfile.Constraints(   
                2,
                1)),
        0.8);

    encoder0.setPositionOffset(0);
    encoder1.setPositionOffset(0);

    lowerArm = new FRCNEO.FRCNEOBuilder(43)
            .withInverted(false)
            // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
            .withSensorPhase(false)
            .withTimeout(10)
            .withCurrentLimitEnabled(true)
            .withCurrentLimit(40)
            // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
            .withPeakOutputForward(40)
            .withPeakOutputReverse(-0.3)
            .withNeutralMode(IdleMode.kBrake)
            .build();

  upperArm = new FRCNEO.FRCNEOBuilder(42)
            .withInverted(false)
            // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
            .withSensorPhase(false)
            .withTimeout(10)
            .withCurrentLimitEnabled(true)
            .withCurrentLimit(20)
            // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
            .withPeakOutputForward(0.3)
            .withPeakOutputReverse(-0.3)
            .withNeutralMode(IdleMode.kBrake)
            .withForwardSoftLimitEnabled(false)
            .build();

    setGoal(0.8);


        
    SmartDashboard.putData("Arm", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLowerArm(double x){
    if ((getLowerArm() >= 0.97 && x < 0) || (getLowerArm() <= 0.73 && x>0)) {
      lowerArm.drivePercentOutput(0);
      upperArm.drivePercentOutput(0);
    }else{
      lowerArm.drivePercentOutput(x);
    }
  }

  public void setBoth(double x, double y){
    // System.out.println(x);
    if( x <=0.4 && x >= -0.4){
      setUpperArm(y/2);
      setLowerArm(y/2 * 3);
    } else {
      setUpperArm(x/2);
    }
  }

  public void setLimited(double x, double y){
    if( x <=0.4 && x >= -0.4){

      
      setUpperArm(Math.min(limiter.calculate(y/2), y));
      setLowerArm(Math.min(limiter.calculate(y/2 * 3), y));
    } else {
      setUpperArm(Math.min(limiter.calculate(x/2), y));
    }
  }

  public void setUpperArm(double x){
    if (getUpperArm() > 0.44 && x > 0) {
      upperArm.drivePercentOutput(0);
    }else{
      upperArm.drivePercentOutput(x);
    }

    //(getUpperArm() < 0.2 && x < 0)

    // upperArm.drivePercentOutput(x/2);
  }

  public double getLowerArm(){
    
    return encoder1.getAbsolutePosition(); 
  }

  public double getUpperArm(){
    return encoder0.getAbsolutePosition();
    
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Lower Arm", () -> getLowerArm(), null);
    builder.addDoubleProperty("Upper Arm", () -> getUpperArm(), null);
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    if(setpoint.position >= 0.9 || setpoint.position <= 0.75){
      lowerArm.motor.setVoltage(0);
    }else{
      lowerArm.motor.setVoltage(output);
    }
  }

  @Override
  protected double getMeasurement() {
    return getLowerArm();
  }

  // public void moveToPoint(){
  //   if(lowerArm >= )
  // }
}
