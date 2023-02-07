package frc.robot.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionDataProvider {

    Supplier<Boolean> getMoving;
    private final DoublePublisher timestamp_publisher;
    private final HashTag hashTag = new HashTag();
    private final ObjectMapper object_mapper;
    SwerveDrivePoseEstimator poseEstimator;

    private static double kCameraXOffset = .076;
    private static double kCameraYOffset = -.13;
    private static double kCameraZOffset = .0;

    public VisionDataProvider(SwerveDrivePoseEstimator pE, Supplier<Boolean> getM) {
        getMoving = getM;
        poseEstimator = pE;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer("example server");
        NetworkTable example_table = inst.getTable("example_table");
        timestamp_publisher = example_table.getDoubleTopic("timestamp").publish();
        object_mapper = new ObjectMapper(new MessagePackFactory());
        NetworkTable vision_table = inst.getTable("Vision");
        inst.addListener(
                vision_table.getEntry("tags"),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                (event) -> accept(event));

    }

    /***
     * Accept a NetworkTableEvent and convert it to a Blips object
     * 
     * @param event the event to accept
     */
    private void accept(NetworkTableEvent event) {
        try {
            Blips blips = object_mapper.readValue(event.valueData.value.getRaw(), Blips.class);
            // System.out.printf("PAYLOAD %s\n", blips);
            // System.out.printf("DELAY (s): %f\n", blips.et);
            // System.out.printf("BLIP COUNT: %d\n", blips.tags.size());
            estimateRobotPose(blips);
            // System.out.println("YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /***
     * Convert a Blip to a Pose3d
     * 
     * @param b the blip to convert
     * @return a Pose3d representing the blip
     */
    private Pose3d blipToPose(Blip b) {
        Translation3d t = new Translation3d(b.pose_t[0][0], b.pose_t[1][0], b.pose_t[2][0]);
        // Matrix<N3, N3> rot = new Matrix<N3, N3>(Nat.N3(), Nat.N3());

        // for (int i = 0; i < 3; ++i) {
        //     for (int j = 0; j < 3; ++j) {
        //         rot.set(i, j, b.pose_R[i][j]);
        //     }
        // }

        Rotation2d rotation2d = new Rotation2d(b.pose_R[0][0], b.pose_R[2][0]);
        Rotation3d r = new Rotation3d(0, 0, rotation2d.getRadians());

        // Rotation3d r = new Rotation3d(rot);

        Pose3d p = new Pose3d(t, r);
        return p;
    }

    /***
     * Update the timestamp on the NetworkTable
     */
    public void updateTimestamp() {
        timestamp_publisher.set(Timer.getFPGATimestamp());
    }

    private void estimateRobotPose(Blips blips) {
        for (Blip b : blips.tags) {
            Pose3d p = blipToPose(b);
            // System.out.printf("TAG ID: %d\n", b.id);
            // System.out.printf("POSE: %s\n", p);

            if (p != null) {
                if (hashTag.getTag((int)b.id) != null) {
                    if(!getMoving.get()){
                        Pose3d tagPose = cameraToRobot(p);
                        // Rotation3d tagRotation3d = new Rotation2d().
                        // double headingtoRad = getHeading.;
                        // Rotation3d tagRotation3d = new Rotation3d(0, 0, poseEstimator.getEstimatedPosition().getRotation().getRadians() );

                        // System.out.println("rotttttttt: " + tagPose.getRotation().getAngle() * 180.0 / Math.PI);
                        Pose2d robotPose = toFieldCoordinates(tagPose.getTranslation(), tagPose.getRotation(), hashTag.getTag((int)b.id)).toPose2d();
                        // System.out.println("ROBOT ROTATION: " + robotPose.getRotation().getDegrees());
                        poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp() - 0.3);

                    }
                    
                }
            }
        }

    }

    private static Pose3d toFieldCoordinates(Translation3d translation, Rotation3d rotation, TestAprilTag apriltag) {
        Transform3d robotRelative = new Transform3d(translation, rotation);
        Transform3d tagRelative = robotRelative.inverse();
        Pose3d fieldRelative = apriltag.getPose().plus(tagRelative);
        return fieldRelative;
    }

    private static Pose3d cameraToRobot(Pose3d cameraPose) {
        double robotX, robotY, robotZ; // X is forward, Y is left, Z is up

        robotX = cameraPose.getZ() - kCameraXOffset;
        robotY = -cameraPose.getX() - kCameraYOffset;
        robotZ = cameraPose.getY() - kCameraZOffset;

        Translation3d translation = new Translation3d(robotX, robotY, robotZ);
        Rotation3d rotation = cameraPose.getRotation();
        
        return new Pose3d(translation, rotation);
    }

}
