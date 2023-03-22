package frc.robot.autonomous;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AHRSClass;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;


public class VasiliWaypointTrajectory extends TrajectoryCommand {
    private double IsRunning = 5;

    /** Creates a new moveFromStartingPoseToGamePiece. */
    public VasiliWaypointTrajectory(SwerveDriveSubsystem m_robotDrive, Trajectory trajectory,
            Supplier<Rotation2d> desiredRotation, AHRSClass gyro) {
        super(m_robotDrive, trajectory, desiredRotation, gyro);
        SmartDashboard.putData("Move From Starting Pose To Game Piece", this);
    }

    public static VasiliWaypointTrajectory newMoveFromStartingPoseToGamePiece(SwerveDriveSubsystem m_robotDrive,
            ControlVectorList controlVectors,
            Supplier<Rotation2d> desiredRotation,
            AHRSClass gyro,
            String path) {

        // ControlVectorList controlVectors = new ControlVectorList();
        // at origin, pointing down x
        // controlVectors.add(new Spline.ControlVector(
        //         splineOne,
        //         splineTwo));
        // // at (0.75,0), pointing down x
        // controlVectors.add(new Spline.ControlVector(
        //         splineThree,
        //         splineFour));
        // at (1,0.25), pointing down y, no curve
        // controlVectors.add(new Spline.ControlVector(
        // new double[] { 1.0, 0.0, 0.0 },
        // new double[] { 0.25, 0.5, 0.0 }));
        // at (1,1), pointing down y, no curve
        // controlVectors.add(new Spline.ControlVector(
        // new double[] { 1.0, 0.0, 0.0 },
        // new double[] { 1.0, 1.0, 0.0 }));

        // String trajecJSON = "GoToCube.wpilib.json"; 
        
        Trajectory trajectory = new Trajectory(); 


        

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
        }


        
        Trajectory t = TrajectoryGenerator.generateTrajectory(
                controlVectors, getConfig());

        for (double time = 0; time < t.getTotalTimeSeconds(); time += 0.1) {
            Trajectory.State s = t.sample(time);
            System.out.printf("%5.3f %5.3f\n", s.poseMeters.getX(), s.poseMeters.getY());
        }
        // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        // 5,
        // 5)
        // // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(SwerveDriveSubsystem.kDriveKinematics);
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // startingPose,
        // List.of(),
        // targetPose,
        // trajectoryConfig);
        // // System.out.println(exampleTrajectory);
        return new VasiliWaypointTrajectory(m_robotDrive, trajectory, () -> desiredRotation.get(), gyro);
    }

    @Override
    public void execute() {
        super.execute();
        // System.out.println("WREGERGERUERzZGRHERIUGRE");
        IsRunning = 5;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        IsRunning = 0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("IsRunning", () -> IsRunning, null);
    }

    private static TrajectoryConfig getConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
                5, // max velocity 1 m/s
                5); // max acceleration 1 m/s/s
        config.setStartVelocity(0); // zero starting velocity
        config.setEndVelocity(0); // zero ending velocity
        config.addConstraint(new CentripetalAccelerationConstraint(1)); // max accel 1 m/s/s
        return config;
    }
}