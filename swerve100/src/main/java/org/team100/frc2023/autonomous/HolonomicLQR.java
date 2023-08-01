// package org.team100.frc2023.autonomous;

// import org.team100.frc2023.LQRManager;
// import org.team100.lib.motion.drivetrain.SpeedLimits;
// import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
// import org.team100.lib.profile.MotionProfile;
// import org.team100.lib.profile.MotionProfileGenerator;
// import org.team100.lib.profile.MotionState;

// import edu.wpi.first.math.MathUtil;

// // these are replaced by our own versions
// // import com.acmerobotics.roadrunner.profile.MotionProfile;
// // import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
// // import com.acmerobotics.roadrunner.profile.MotionState;

// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.DoublePublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Timer;

// /**
//  * This holonomic drive controller can be used to follow trajectories using a
//  * holonomic drivetrain
//  * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler
//  * problem to solve
//  * compared to skid-steer style drivetrains because it is possible to
//  * individually control forward,
//  * sideways, and angular velocity.
//  *
//  * <p>
//  * The holonomic drive controller takes in one PID controller for each
//  * direction, forward and
//  * sideways, and one profiled PID controller for the angular direction. Because
//  * the heading dynamics
//  * are decoupled from translations, users can specify a custom heading that the
//  * drivetrain should
//  * point toward. This heading reference is profiled for smoothness.
//  */
// public class HolonomicLQR {
//     private Pose2d m_poseError = new Pose2d();
//     private Rotation2d m_rotationError = new Rotation2d();
//     private Pose2d m_poseTolerance = new Pose2d();
//     private boolean m_enabled = true;

//     private final LQRManager m_xManager;
//     private final LQRManager m_yManager;
//     private final LQRManager m_thetaManager;

//     private MotionProfile profile_x;
//     private MotionProfile profile_y;
//     private MotionProfile profile_theta;
//     double kV_t = 1.15;
//     double kA_t = 0.7;
//     double kV_r = 1.15;
//     double kA_r = .07;
//     Timer m_timer = new Timer();

//     SwerveDriveSubsystem m_robotDrive;

//     private MotionState m_xRef = new MotionState(0, 0);
//     private MotionState m_yRef = new MotionState(0, 0);
//     private MotionState m_thetaRef = new MotionState(0, 0);

//     public HolonomicLQR(
//             SwerveDriveSubsystem robotDrive,
//             LQRManager xManager,
//             LQRManager yManager,
//             LQRManager thetaManager) {
//         m_xManager = xManager;
//         m_yManager = yManager;
//         m_thetaManager = thetaManager;
//         m_robotDrive = robotDrive;
//     }

//     /**
//      * Returns true if the pose error is within tolerance of the reference.
//      *
//      * @return True if the pose error is within tolerance of the reference.
//      */

//     public boolean atReference() {
//         final var translation_error = m_poseError.getTranslation();
//         final var rotation_error = m_rotationError;
//         final var translation_tolerance = m_poseTolerance.getTranslation();
//         final var rotation_tolerance = m_poseTolerance.getRotation();
//         return Math.abs(translation_error.getX()) < translation_tolerance.getX()
//                 && Math.abs(translation_error.getY()) < translation_tolerance.getY()
//                 && Math.abs(rotation_error.getRadians()) < rotation_tolerance.getRadians();
//     }

//     /**
//      * Sets the pose error which is considered tolerance for use with atReference().
//      *
//      * @param tolerance The pose error which is tolerable.
//      */
//     public void setTolerance(Pose2d tolerance) {
//         m_poseTolerance = tolerance;
//     }

//     /**
//      * Returns the next output of the holonomic drive controller.
//      *
//      * @param currentPose                          The current pose, as measured by
//      *                                             odometry or pose estimator.
//      * @param trajectoryPose                       The desired trajectory pose, as
//      *                                             sampled for the current timestep.
//      * @param desiredLinearVelocityMetersPerSecond The desired linear velocity.
//      * @param desiredHeading                       The desired heading.
//      * @return field-relative twist, meters and radians per second
//      */

//     public Twist2d calculate(
//             Pose2d currentPose,
//             Trajectory.State desiredState,
//             Rotation2d desiredHeading) {
//         Pose2d trajectoryPose = desiredState.poseMeters;

//         // TODO: turn feedforward back on

//         m_poseError = trajectoryPose.relativeTo(currentPose);
//         m_rotationError = new Rotation2d(
//                 MathUtil.angleModulus(desiredHeading.minus(currentPose.getRotation()).getRadians()));

//         if (!m_enabled) {
//             return null;
//         }

//         m_xRef = profile_x.get(m_timer.get());
//         m_yRef = profile_y.get(m_timer.get());
//         m_thetaRef = profile_theta.get(m_timer.get());

//         xDesired.set(m_xRef.getX());
//         yDesired.set(m_yRef.getX());
//         thetaDesired.set(m_thetaRef.getX());

//         m_xManager.m_Loop.setNextR(m_xRef.getX(), m_xRef.getV());
//         m_yManager.m_Loop.setNextR(m_yRef.getX(), m_yRef.getV());
//         m_thetaManager.m_Loop.setNextR(m_thetaRef.getX(), m_thetaRef.getV());

//         m_xManager.m_Loop.correct(VecBuilder.fill(m_robotDrive.getPose().getX()));
//         m_yManager.m_Loop.correct(VecBuilder.fill(m_robotDrive.getPose().getY()));
//         m_thetaManager.m_Loop.correct(VecBuilder.fill(m_robotDrive.getPose().getRotation().getRadians()));

//         m_xManager.m_Loop.predict(0.020);
//         m_yManager.m_Loop.predict(0.020);
//         m_thetaManager.m_Loop.predict(0.020);

//         Twist2d fieldRelativeTarget = new Twist2d(
//                 ((kV_t * m_xRef.getV()) + (kA_t * m_xRef.getA())),
//                 ((kV_t * m_yRef.getV()) + (kA_t * m_yRef.getA())),
//                 ((kV_r * m_thetaRef.getV()) + (kA_r * m_thetaRef.getA())));

//         return fieldRelativeTarget;

//     }

//     public void reset(Pose2d currentPose) {
//         m_xManager.m_Loop.reset(VecBuilder.fill(m_robotDrive.getPose().getX(), 0));
//         m_yManager.m_Loop.reset(VecBuilder.fill(m_robotDrive.getPose().getY(), 0));
//         m_thetaManager.m_Loop.reset(VecBuilder.fill(m_robotDrive.getPose().getRotation().getRadians(), 0));

//         m_xRef = new MotionState(m_robotDrive.getPose().getX(), 0);
//         m_yRef = new MotionState(m_robotDrive.getPose().getY(), 0);
//         m_thetaRef = new MotionState(m_robotDrive.getPose().getRotation().getRadians(), 0);

//     }

//     public void start() {
//         m_timer.restart();
//     }

//     public void updateProfile(Pose2d X, SpeedLimits speedLimits) {
//         profile_x = MotionProfileGenerator.generateSimpleMotionProfile(
//                 new MotionState(m_robotDrive.getPose().getX(), 0),
//                 new MotionState(X.getX(), 0),
//                 speedLimits.speedM_S,
//                 speedLimits.accelM_S2,
//                 speedLimits.jerkM_S3);

//         profile_x = MotionProfileGenerator.generateSimpleMotionProfile(
//                 new MotionState(m_robotDrive.getPose().getY(), 0),
//                 new MotionState(X.getY(), 0),
//                 speedLimits.speedM_S,
//                 speedLimits.accelM_S2,
//                 speedLimits.jerkM_S3);

//         profile_theta = MotionProfileGenerator.generateSimpleMotionProfile(
//                 new MotionState(m_robotDrive.getPose().getRotation().getRadians(), 0),
//                 new MotionState(X.getRotation().getRadians(), 0),
//                 speedLimits.angleSpeedRad_S,
//                 speedLimits.angleAccelRad_S2,
//                 speedLimits.angleJerkRad_S3);
//     }

//     /**
//      * Enables and disables the controller for troubleshooting problems. When
//      * calculate() is called on
//      * a disabled controller, only feedforward values are returned.
//      *
//      * @param enabled If the controller is enabled or not.
//      */
//     public void setEnabled(boolean enabled) {
//         m_enabled = enabled;
//     }

//     NetworkTableInstance inst = NetworkTableInstance.getDefault();
//     NetworkTable table = inst.getTable("Holonomic LQR");
//     DoublePublisher xOutPublisher = table.getDoubleTopic("X Output").publish();
//     DoublePublisher yOutPublisher = table.getDoubleTopic("Y Output").publish();
//     DoublePublisher xDesired = table.getDoubleTopic("X Desired").publish();
//     DoublePublisher yDesired = table.getDoubleTopic("Y Desired").publish();
//     DoublePublisher thetaDesired = table.getDoubleTopic("Theta Desired").publish();
// }
