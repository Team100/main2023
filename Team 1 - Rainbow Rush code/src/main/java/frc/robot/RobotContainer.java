// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.controlArm;
import frc.robot.commands.pinchCommand;
import frc.robot.commands.unPinchCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  arm armSubsystem = new arm();
  XboxController controller = new XboxController(0);
  Claw pinchSubsystem = new Claw();
  pinchCommand pinch = new pinchCommand(pinchSubsystem);
  unPinchCommand unPinch = new unPinchCommand(pinchSubsystem);
  controlArm armControl = new controlArm(armSubsystem, controller);
  
  public final XboxController m_joystick = new XboxController(0);
    final JoystickButton l2 = new JoystickButton(m_joystick, 0);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    l2.whileHeld(armControl);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return armControl;
  }
}

//comments are funnnnn
//3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679821480865132823066470938446095505822317253594081284811164462294895493038196