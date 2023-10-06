package org.team100.frc2023.autonomous;

import java.util.function.Supplier;

import org.team100.frc2023.commands.AutoLevel;
import org.team100.frc2023.commands.DriveMobility;
import org.team100.frc2023.commands.DriveWithHeading;
import org.team100.frc2023.commands.WheelsForward;
import org.team100.frc2023.commands.arm.ArmTrajectory;
import org.team100.frc2023.commands.arm.SetCubeMode;
import org.team100.frc2023.commands.manipulator.Intake;
import org.team100.frc2023.subsystems.ManipulatorInterface;
import org.team100.frc2023.subsystems.arm.ArmInterface;
import org.team100.frc2023.subsystems.arm.ArmPosition;
import org.team100.lib.autonomous.DriveStop;
import org.team100.lib.commands.ResetPose;
import org.team100.lib.commands.ResetRotation;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.motion.drivetrain.HeadingInterface;
import org.team100.lib.motion.drivetrain.SpeedLimit;
import org.team100.lib.motion.drivetrain.SpeedLimitsFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;
import org.team100.lib.sensors.RedundantGyroInterface;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Autonomous extends SequentialCommandGroup {
    public static class Config {
        public double kArmExtendTimeout = 2;
        public double kManipulatorRunTimeout = 0.2;
        public double kArmSafeTimeout = 2;
        public double kStopTimeout = 1;        
    }

    private final Config m_config = new Config();
    private final SwerveDriveSubsystem m_robotDrive;
    private final FrameTransform m_transform;
    private final ArmInterface m_arm;
    private final ManipulatorInterface m_manipulator;
    private final RedundantGyroInterface m_gyro;
    private final LEDIndicator m_indicator;
    private final HeadingInterface m_heading;

    // TODO: make routine an enum
    public Autonomous(
            SwerveDriveSubsystem robotDrive,
            FrameTransform transform,
            ArmInterface arm,
            ManipulatorInterface manipulator,
            RedundantGyroInterface gyro,
            LEDIndicator indicator,
            HeadingInterface heading,
            int routine) {
        m_robotDrive = robotDrive;
        m_heading = heading;
        m_transform = transform;
        m_arm = arm;
        m_manipulator = manipulator;
        m_gyro = gyro;
        m_indicator = indicator;
        addRequirements(m_robotDrive);

        

        // if (routine == 0) {
        //     placeCube();

        // } else if (routine == 1) {
            // new WaitCommand(1);
            // new ResetPose(robotDrive, routine, routine, routine)
            // placeCube();
            // autoLevel(false);

            // reset180();
            // placeCube();
            // autoLevel(false);
            
        // } else if (routine == 2) {
        //     placeCube();
        //     driveOutAndBack();
        //     autoLevel(true);
        // }
        // addCommands(
        //         new ResetPose(m_robotDrive, 0, 0, Math.PI),
        //         // new DeadDrivetrain(m_robotDrive),
        //         // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
        //         // new ResetPose(m_robotDrive, 0, 0, Math.PI),
        //         // new DeadDrivetrain(m_robotDrive),

        //         new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
        //         // timeout(new ArmTrajectory(ArmPosition.AUTO, m_arm, false), m_config.kArmExtendTimeout),
        //         // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new ArmTrajectory(ArmPosition.AUTO, m_arm, false))
        //         new ParallelDeadlineGroup(new WaitCommand(2), new Intake(m_manipulator)),
        //         new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
        //         new AutoLevel(false, m_robotDrive, m_gyro, m_transform)
        // );

        
        if(routine == 2){
            addCommands(
                new ResetPose(m_robotDrive, 0, 0, Math.PI),
                new SetCubeMode(m_arm, m_indicator),
                new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
                   
                new ParallelDeadlineGroup(new WaitCommand(0.2), new Intake(m_manipulator, m_arm)),
                new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
    
                new DriveMobility(robotDrive),
                // timeout(new DriveStop(m_robotDrive, m_heading), 2),
                new ParallelDeadlineGroup(new WaitCommand(1), new DriveStop(robotDrive, m_heading)),
    
                new DriveToThreshold(m_robotDrive),
    
                new AutoLevel(true, m_robotDrive, m_gyro, m_transform)
    
            );
        } else if(routine == 1){
            addCommands(
                new ResetPose(m_robotDrive, 0, 0, Math.PI),
                new SetCubeMode(m_arm, m_indicator),
                new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
                   
                new ParallelDeadlineGroup(new WaitCommand(0.2), new Intake(m_manipulator, m_arm)),
                new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
                new AutoLevel(false, m_robotDrive, m_gyro, m_transform)
    
                
    
            );
        } else if(routine == 0){

            Supplier<Rotation2d> rotSupplier = () -> new Rotation2d(Math.PI);
            addCommands(
                new ResetPose(m_robotDrive, 0, 0, Math.PI),
                new SetCubeMode(m_arm, m_indicator),
                // new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
                   
                // new ParallelDeadlineGroup(new WaitCommand(0.2), new Intake(m_manipulator, m_arm)),
                // new ParallelDeadlineGroup(new WaitCommand(2), new ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
                // new DriveMobility(robotDrive)
                new ParallelDeadlineGroup(new WaitCommand(2.5' hb    hFM,AFxfFCFF),  new DriveWithHeading( () -> new Twist2d(2, 0.02, 0.0), m_robotDrive, m_heading, () -> SpeedLimitsFactory.getSpeedLimits(SpeedLimit.Medium), new Timer(), rotSupplier ))
               
                
    
            );
        }

        
    }

    private void placeCube() {
        addCommands(
                // new ResetPose(m_robotDrive, 0, 0, Math.PI),
                new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                new ResetPose(m_robotDrive, 0, 0, Math.PI),

                // timeout(new ArmTrajectory(ArmPosition.AUTO, m_arm, false), m_config.kArmExtendTimeout),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new ArmTrajectory(ArmPosition.AUTO, m_arm, false)),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new Intake(m_manipulator)),
                // new ParallelDeadlineGroup(new DeadDrivetrain(m_robotDrive), new ArmTrajectory(ArmPosition.SAFE, m_arm, false)),
                new AutoLevel(false, m_robotDrive, m_gyro, m_transform)
                );
                // timeout(new Intake(m_manipulator), m_config.kManipulatorRunTimeout),
                // timeout(new ArmTrajectory(ArmPosition.SAFE, m_arm, false), m_config.kArmSafeTimeout));
                // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)) );

    }

    

    private void reset180() {
        addCommands(
                // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                new ResetPose(m_robotDrive, 0, 0, Math.PI)
        );
                
    }

    

    private void autoLevel(boolean reversed) {
        addCommands(
                // new ResetPose(m_robotDrive, 0, 0, 0),
                // new ResetRotation(m_robotDrive, Rotation2d.fromDegrees(180)),
                new AutoLevel(reversed, m_robotDrive, m_gyro, m_transform));
    }

    // TODO: why do we need a timeout?
    private Command timeout(Command command, double seconds) {
        return new ParallelDeadlineGroup(new WaitCommand(seconds), command);
    }
}
