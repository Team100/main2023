package org.team100.frc2023.autonomous;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.DriveMobility;
import org.team100.frc2023.commands.Arm.ArmTrajectory;
import org.team100.frc2023.commands.Arm.SetCubeMode;
import org.team100.frc2023.commands.Manipulator.Close;
import org.team100.frc2023.subsystems.AHRSClass;
import org.team100.frc2023.subsystems.Manipulator;
import org.team100.frc2023.subsystems.SwerveDriveSubsystem;
import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.autonomous.DriveStop;
import org.team100.lib.commands.ResetRotation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {

    double armExtendDelay = 1.5;
    double manipulatorRunDelay = 0.2;
    double armSafeDelay = 2;
  /** Creates a new Autonomous. */
  public Autonomous(SwerveDriveSubsystem m_robotDrive, ArmController m_arm, Manipulator m_manipulator, AHRSClass m_gyro, int routine, boolean isBlueAlliance) {
      
      if(routine == 2){
        addCommands(
            new SetCubeMode(m_arm, m_robotDrive),
            new ParallelDeadlineGroup(new WaitCommand(armExtendDelay), new ArmTrajectory(ArmPosition.HIGH, m_arm)),
            new ParallelDeadlineGroup(new WaitCommand(manipulatorRunDelay), new Close(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(armSafeDelay), new ArmTrajectory(ArmPosition.SAFE, m_arm)),
        //     // new WaitCommand(0.25),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),


            new DriveMobility(m_robotDrive),
            new ParallelDeadlineGroup(new WaitCommand(1), new DriveStop(m_robotDrive)),
            new DriveToThreshold(m_robotDrive),
            new AutoLevel(true, m_robotDrive, m_gyro)
            
        );
      } else if(routine == 1){
        addCommands(
            new SetCubeMode(m_arm, m_robotDrive),
            new ParallelDeadlineGroup(new WaitCommand(armExtendDelay), new ArmTrajectory(ArmPosition.HIGH, m_arm)),
            new ParallelDeadlineGroup(new WaitCommand(manipulatorRunDelay), new Close(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(armSafeDelay), new ArmTrajectory(ArmPosition.SAFE, m_arm)),
        //     // new WaitCommand(0.25),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),


            // new DriveMobility(m_robotDrive),
            // new ParallelDeadlineGroup(new WaitCommand(1), new DriveStop(m_robotDrive)),
            // new DriveToThreshold(m_robotDrive),
            new AutoLevel(false, m_robotDrive, m_gyro)
        );
      } else if(routine == 0){
        addCommands(
            new SetCubeMode(m_arm, m_robotDrive),
            new ParallelDeadlineGroup(new WaitCommand(armExtendDelay), new ArmTrajectory(ArmPosition.HIGH, m_arm)),
            new ParallelDeadlineGroup(new WaitCommand(manipulatorRunDelay), new Close(m_manipulator)),
            new ParallelDeadlineGroup(new WaitCommand(armSafeDelay), new ArmTrajectory(ArmPosition.SAFE, m_arm)),
        //     // new WaitCommand(0.25),
            new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180))


            // new DriveMobility(m_robotDrive),
            // new ParallelDeadlineGroup(new WaitCommand(1), new DriveStop(m_robotDrive)),
            // new DriveToThreshold(m_robotDrive),
            // new AutoLevel(false, m_robotDrive, m_gyro)
        );
      }
        
    
    
  
  }

}
