package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AHRSClass;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VasiliAutonomous extends SequentialCommandGroup {

    ControlVectorList controlVectors = new ControlVectorList();

    
    /** Creates a new autonomous. */
    public VasiliAutonomous(SwerveDriveSubsystem m_robotDrive, AHRSClass m_gyro) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        // Rotation2d desiredRots = new Rotation2d(Math.PI);
        // SwerveModuleState[] desiredStates = new SwerveModuleState[] {
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots),
        // new SwerveModuleState(0, desiredRots)
        // };
        // CommandBase command = new CommandBase() {
        // @Override
        // public void initialize() {
        // m_robotDrive.setModuleStates(desiredStates);
        // };

        // @Override
        // public boolean isFinished() {
        // return false;
        // }
        // };
        // command.addRequirements(m_robotDrive);

        controlVectors.add(new Spline.ControlVector( 
            new double[] { 0.0, 0, 0 },
            new double[] { 0.0, 2, 0.0 }
        ));

        controlVectors.add(new Spline.ControlVector( 
            new double[] { 5, 4, 0 },
            new double[] { 2, 0, 0 }
        ));

        addCommands(
                // TODO add place cone/cube command here
                // TODO Create a new holonmoic drive controller with the rotation fix
                // moveFromStartingPoseToGamePiece
                // .newMoveFromStartingPoseToGamePiece(
                // m_robotDrive,
                // new Pose2d(
                // m_robotDrive.getPose().getX(),
                // m_robotDrive.getPose().getY(),
                // new Rotation2d(Math.PI / 2)),
                // new Pose2d(0, 0.92, new Rotation2d(Math.PI / 2)),
                // () -> new Rotation2d(Math.PI)),

                VasiliWaypointTrajectory
                        .newMoveFromStartingPoseToGamePiece(
                                m_robotDrive,
                                controlVectors,
                                () -> new Rotation2d(Math.PI),
                                m_gyro),
                new Rotate(m_robotDrive, 0)
        );

        // TODO Make a pickup cone/cube from floor command here

    }
}
