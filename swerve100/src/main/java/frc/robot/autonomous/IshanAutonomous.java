// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.autonomous;

// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.SwerveDriveSubsystem;

// public class IshanAutonomous extends CommandBase {
//   /** Creates a new IshanAutonomous. */
//   SwerveDriveSubsystem m_robotDr;
//   public IshanAutonomous(SwerveDriveSubsystem dS) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     driveSubsystem = dS;
//     addRequirements(driveSubsystem);

//     // Create config for trajectory
//     TrajectoryConfig config = new TrajectoryConfig(
//         AutoConstants.kMaxSpeedMetersPerSecond,
//         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//         // Add kinematics to ensure max speed is actually obeyed
//         .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
//     double controlPointAngle = Math.atan2((1.071626-m_robotDrive.getPose().getY()), (14.513558-m_robotDrive.getPose().getX()));
//     // An example trajectory to follow. All units in meters.
//     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d(controlPointAngle)),
//         List.of(new Translation2d((14.513558+m_robotDrive.getPose().getX())/2, (1.071626+m_robotDrive.getPose().getY())/2)),
//         new Pose2d(14.513558, 1.071626, new Rotation2d(controlPointAngle)),
//         config);
//     // System.out.println(exampleTrajectory);

//     m_robotDrive.thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SmartDashboard.putData(m_field);

//     // Push the trajectory to Field2d.
//     m_field.getObject("traj").setTrajectory(exampleTrajectory);

//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//         exampleTrajectory,
//         m_robotDrive::getPose, // Functional interface to feed supplier
//         SwerveDriveSubsystem.kDriveKinematics,

//         // Position controllers
//         m_robotDrive.xController,
//         m_robotDrive.yController,
//         m_robotDrive.thetaController,
//         () -> new Rotation2d(),
//         m_robotDrive::setModuleStates,
//         m_robotDrive);

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
