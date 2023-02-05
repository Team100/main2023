// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FRCLib.Motors.FRCNEO;

public class Arm extends SubsystemBase{
  /** Creates a new Arm. */
  public FRCNEO lowerArm;
  public FRCNEO upperArm;
  // public Spark lowerArmv2;

  private final AnalogEncoder encoder0 = new AnalogEncoder(0);
  private final AnalogEncoder encoder1 = new AnalogEncoder(1);
  
  public Arm() {
    lowerArm = new FRCNEO.FRCNEOBuilder(43)
            .withInverted(false)
            // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
            .withSensorPhase(false)
            .withTimeout(10)
            .withCurrentLimitEnabled(true)
            .withCurrentLimit(10)
            // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
            .withPeakOutputForward(0.3)
            .withPeakOutputReverse(-0.3)
            .withNeutralMode(IdleMode.kBrake)
            .build();

  upperArm = new FRCNEO.FRCNEOBuilder(42)
            .withInverted(false)
            // .withFeedbackPort(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.FEEDBACK_PORT)
            .withSensorPhase(false)
            .withTimeout(10)
            .withCurrentLimitEnabled(true)
            .withCurrentLimit(10)
            // .withOpenLoopRampRate(Constants.IndexerConstants.IndexerMotors.IndexerStageOne.OPEN_LOOP_RAMP)
            .withPeakOutputForward(0.3)
            .withPeakOutputReverse(-0.3)
            .withNeutralMode(IdleMode.kBrake)
            .build();
    // lowerArmv2 = new Spark(42);


        
    SmartDashboard.putData("Arm", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLowerArm(double x){
    lowerArm.drivePercentOutput(x/2);

  }

  public void setUpperArm(double x){
    upperArm.drivePercentOutput(x/2);

  }

  public double getLowerArm(){
    return encoder0.getAbsolutePosition(); 
  }

  public double getUpperArm(){
    return encoder1.getAbsolutePosition();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Encoder 0", () -> getLowerArm(), null);
    builder.addDoubleProperty("Encoder 1", () -> getUpperArm(), null);
  }
}
