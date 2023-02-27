package frc.robot.localization;

import java.io.IOException;
import java.nio.file.Path;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
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
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class VisionDataProvider extends SubsystemBase implements TableEventListener {

    Supplier<Boolean> getMoving;
    Supplier<Pose2d> getPose;
    private final DoublePublisher timestamp_publisher;
    // private final HashTag hashTag = new HashTag();
    private final ObjectMapper object_mapper;
    SwerveDrivePoseEstimator poseEstimator;

    // public AprilTagFieldLayout layout;
    public AprilFieldLayout2 layout;

    SwerveDriveSubsystem m_robotDrive;

    Pose2d currentRobotinFieldCoords;
    Pose2d currentTagInRobotCoords;
    Pose2d currentTagInFieldCoords;
    Pose2d pastRobotPose;
    boolean first = true;

    public VisionDataProvider(SwerveDrivePoseEstimator pE, Supplier<Boolean> getM, Supplier<Pose2d> getP) {

        getPose = getP;

        getMoving = getM;
        poseEstimator = pE;

        pastRobotPose = new Pose2d();
        currentRobotinFieldCoords = new Pose2d();
        currentTagInRobotCoords = new Pose2d();
        currentTagInFieldCoords = new Pose2d();

        // TAG MAP
        System.out.println(Filesystem.getDeployDirectory());
        try {
            Path path = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
            // layout = new AprilTagFieldLayout(path);
            layout = new AprilFieldLayout2(path);

            // layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
            System.out.println("JSON map loaded");
            for (AprilTag t : layout.getTags()) {
                System.out.printf("tag %s\n", t.toString());
            }
        } catch (IOException e) {
            System.out.println("Could not find JSON map");
            e.printStackTrace();
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer("example server");
        NetworkTable example_table = inst.getTable("example_table");
        timestamp_publisher = example_table.getDoubleTopic("timestamp").publish();
        object_mapper = new ObjectMapper(new MessagePackFactory());
        NetworkTable vision_table = inst.getTable("Vision");
        // inst.addListener(
        // vision_table.getEntry("tags"),
        // EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        // (event) -> accept(event));

        // Listen to ALL the updates in the vision table. :-)
        vision_table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), this);
        SmartDashboard.putData("Vision Data Provider", this);
    }

    /***
     * Accept a NetworkTableEvent and convert it to a Blips object
     * 
     * @param event the event to acceptx
     */
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        try {
            System.out.printf("TABLE PATH %s\n", table.getPath());
            System.out.printf("KEY %s\n", key);
            Blips blips = object_mapper.readValue(event.valueData.value.getRaw(), Blips.class);
            System.out.printf("PAYLOAD %s\n", blips);
            System.out.printf("DELAY (s): %f\n", blips.et);
            System.out.printf("BLIP COUNT: %d\n", blips.tags.size());
            estimateRobotPose(key, blips);
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
        // for (int j = 0; j < 3; ++j) {
        // rot.set(i, j, b.pose_R[i][j]);
        // }
        // }

        Rotation2d rotation2d = new Rotation2d(b.pose_R[0][0], b.pose_R[2][0]);
        // Rotation3d poseEstimatorRotation = new Rotation3d(0, 0, 0);
        Rotation3d tagRotation = new Rotation3d(0, 0, rotation2d.getRadians());

        // Rotation3d r = new Rotation3d(rot);

        Pose3d TagInCameraCords = new Pose3d(t, tagRotation);

        return TagInCameraCords;
    }

    /***
     * Update the timestamp on the NetworkTable
     */
    public void updateTimestamp() {
        timestamp_publisher.set(Timer.getFPGATimestamp());
    }

    /**
     * @param key   the camera identity, obtained from proc/cpuinfo
     * @param blips all the targets the camera sees right now
     */
    private void estimateRobotPose(String key, Blips blips) {
        for (Blip b : blips.tags) {
            Pose3d TagInCameraCords = blipToPose(b);
            // System.out.printf("TAG ID: %d\n", b.id);
            // System.out.printf("POSE: %s\n", TagInCameraCords);

            if (TagInCameraCords != null) {

                Optional<Pose3d> tagInFieldCords = layout.getTagPose(b.id);

                if (tagInFieldCords.isPresent()) {

                    Pose3d tagInRobotCords = cameraToRobot(key, TagInCameraCords);
                    Pose2d robotInFieldCords = toFieldCoordinates(tagInRobotCords.getTranslation(),
                            tagInRobotCords.getRotation(),
                            tagInFieldCords.get()).toPose2d();

                    currentTagInRobotCoords = tagInRobotCords.toPose2d();
                    currentRobotinFieldCoords = new Pose2d(robotInFieldCords.getTranslation(),
                            getPose.get().getRotation());
                    currentTagInFieldCoords = tagInFieldCords.get().toPose2d();
                    // System.out.println("ROBOOOOOOOT POSE: " + robotInFieldCords);

                    // if(first){
                    poseEstimator.addVisionMeasurement(currentRobotinFieldCoords, Timer.getFPGATimestamp() - .075);
                    // }else if( Math.abs(currentRobotinFieldCoords.getX() - pastRobotPose.getX())
                    // <= 0.5 && !first ){
                    // poseEstimator.addVisionMeasurement(currentRobotinFieldCoords,
                    // Timer.getFPGATimestamp() - .075);
                }

                // pastRobotPose = currentRobotinFieldCoords;
                first = false;
                // }
            }
        }

    }

    /**
     * @param tagTranslationInRobotCords tag translation in robot coords
     * @param tagRotationInRobotCords    tag rotation in robot coords -- this is NOT
     *                                   USED, instead we substitute the robot's
     *                                   rotation
     * @return robot pose in field coordinates.
     */
    private Pose3d toFieldCoordinates(Translation3d tagTranslationInRobotCords, Rotation3d tagRotationInRobotCords,
            Pose3d tagInFieldCords) {
        // TODO: i think there's a bug in here about the substitution of robot rotation
        // TODO: is the Math.PI here correct in all cases?  what does it mean?
        Transform3d tagInRobotCords = new Transform3d(tagTranslationInRobotCords,
                new Rotation3d(0, 0, Math.PI - getPose.get().getRotation().getRadians()));
        Transform3d robotInTagCords = tagInRobotCords.inverse();
        Pose3d robotInFieldCords = tagInFieldCords.plus(robotInTagCords);
        return robotInFieldCords;
    }

    /**
     * Transforms tag poses from camera coordinates (Z-forward, X-right, Y-down) to
     * robot coordinates (X-forward, Y-left, Z-up).
     * 
     * @param key              the camera identity, obtained from proc/cpuinfo
     * @param tagInCameraCords one tag pose relative to the camera
     * @return tag in robot coordinates
     */
    private static Pose3d cameraToRobot(String key, Pose3d tagInCameraCords) {
        // TODO: fill out these offsets
        Translation3d cameraOffsetTranslation;
        Rotation3d cameraRotationOffset;
        switch (key) {
            case "100000004e0a1fb9": // CAMERA ONE
                cameraRotationOffset = new Rotation3d();
                cameraOffsetTranslation = new Translation3d();
                break;
            case "1000000013c9c96c": // CAMERA TWO
                cameraRotationOffset = new Rotation3d();
                cameraOffsetTranslation = new Translation3d();
                break;
            case "10000000a7a892c0": // CAMERA THREE
                cameraRotationOffset = new Rotation3d();
                cameraOffsetTranslation = new Translation3d();
                break;
            default:
                cameraRotationOffset = new Rotation3d();
                cameraOffsetTranslation = new Translation3d();
        }

        Translation3d translationInRobotCoords = new Translation3d(
                tagInCameraCords.getZ(),
                -tagInCameraCords.getX(),
                -tagInCameraCords.getY())
                .minus(cameraOffsetTranslation);

        Rotation3d rotationInRobotCoords = tagInCameraCords.getRotation().minus(cameraRotationOffset);

        return new Pose3d(translationInRobotCoords, rotationInRobotCoords);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Vision X", () -> currentRobotinFieldCoords.getX(), null);
        builder.addDoubleProperty("Vision Y", () -> currentRobotinFieldCoords.getY(), null);
        builder.addDoubleProperty("Vision Rotation", () -> currentRobotinFieldCoords.getRotation().getRadians(), null);

        builder.addDoubleProperty("tag X", () -> currentTagInRobotCoords.getX(), null);
        builder.addDoubleProperty("tag Y", () -> currentTagInRobotCoords.getY(), null);
        builder.addDoubleProperty("tag Rotation", () -> currentTagInRobotCoords.getRotation().getRadians(), null);

        builder.addDoubleProperty("Field Tag X", () -> currentTagInFieldCoords.getX(), null);
        builder.addDoubleProperty("Field Tag Y", () -> currentTagInFieldCoords.getY(), null);
        builder.addDoubleProperty("Field Tag Rotation", () -> currentTagInFieldCoords.getRotation().getRadians(), null);

        builder.addDoubleProperty("Tag 1 Pose X", () -> layout.getTagPose(1).get().toPose2d().getX(), null);
        builder.addDoubleProperty("Tag 1 Pose Y", () -> layout.getTagPose(1).get().toPose2d().getY(), null);
        builder.addDoubleProperty("Tag 1 Rotation",
                () -> layout.getTagPose(1).get().toPose2d().getRotation().getDegrees(), null);

    }

}
