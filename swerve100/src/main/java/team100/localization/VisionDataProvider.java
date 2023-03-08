package team100.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.config.Camera;
import team100.indicator.GoNoGoIndicator;

/**
 * Extracts robot pose estimates from camera input.
 */
public class VisionDataProvider extends SubsystemBase implements TableEventListener {
    private final Supplier<Pose2d> getPose;
    private final DoublePublisher timestamp_publisher;
    private final ObjectMapper object_mapper;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final double kVisionChangeToleranceMeters = .1;

    // TODO Make this private
    public AprilTagFieldLayoutWithCorrectOrientation layout;

    SwerveDriveSubsystem m_robotDrive;

    Pose2d currentRobotinFieldCoords;

    private final GoNoGoIndicator indicator;
    private Pose2d lastRobotInFieldCoords;

    public VisionDataProvider(
            DriverStation.Alliance alliance,
            SwerveDrivePoseEstimator poseEstimator,
            Supplier<Pose2d> getPose)
            throws IOException {

        this.getPose = getPose;
        this.poseEstimator = poseEstimator;
        indicator = new GoNoGoIndicator(8); // 8 hz = flash fast

        currentRobotinFieldCoords = new Pose2d();

        if (alliance == DriverStation.Alliance.Blue) {
            layout = AprilTagFieldLayoutWithCorrectOrientation.blueLayout();
        } else { // red
            layout = AprilTagFieldLayoutWithCorrectOrientation.redLayout();
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startServer("example server");
        NetworkTable example_table = inst.getTable("example_table");
        timestamp_publisher = example_table.getDoubleTopic("timestamp").publish();
        object_mapper = new ObjectMapper(new MessagePackFactory());
        NetworkTable vision_table = inst.getTable("Vision");

        // Listen to ALL the updates in the vision table. :-)
        vision_table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), this);
        SmartDashboard.putData("Vision Data Provider", this);
    }

    public void close() {
        indicator.close();
    }

    /***
     * Accept a NetworkTableEvent and convert it to a Blips object
     * 
     * @param event the event to acceptx
     */
    public void accept(NetworkTable table, String key, NetworkTableEvent event) {
        try {
            // System.out.printf("TABLE PATH %s\n", table.getPath());
            // System.out.printf("KEY %s\n", key);
            Blips blips = object_mapper.readValue(event.valueData.value.getRaw(), Blips.class);
            // System.out.printf("PAYLOAD %s\n", blips);
            // System.out.printf("DELAY (s): %f\n", blips.et);
            // System.out.printf("BLIP COUNT: %d\n", blips.tags.size());
            estimateRobotPose(this::cameraOffset, poseEstimator::addVisionMeasurement, key, blips);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /***
     * Update the timestamp on the NetworkTable
     */
    public void updateTimestamp() {
        timestamp_publisher.set(Timer.getFPGATimestamp());
    }

    /**
     * Camera views relative to the robot center at the floor.
     */
    private Transform3d cameraOffset(String serialNumber) {

        Camera cam = Camera.get(serialNumber);
        switch (cam) {
            case REAR:
                return new Transform3d(new Translation3d(0.4, 0, 0.3), new Rotation3d(0, -0.35, 3.14));
            case FRONT:
                return new Transform3d(new Translation3d(.406, .1025, 0.3), new Rotation3d(0, -0.192, 0));
            case RIGHT:
                return new Transform3d(new Translation3d(0, 0.215, 0.228), new Rotation3d(0, -0.35, -0.35));
            case LEFT:
                return new Transform3d(new Translation3d(0, 0.205, 0.24), new Rotation3d(0, -0.35, 0.35));
            case UNKNOWN:
                return new Transform3d(new Translation3d(0.254, 0.127, 0.3), new Rotation3d(0, 0, 0));
            default:
                return new Transform3d();
        }
    }

    /**
     * @param estimateConsumer is the pose estimator but exposing it here makes it
     *                         easier to test.
     * @param key              the camera identity, obtained from proc/cpuinfo
     * @param blips            all the targets the camera sees right now
     */
    void estimateRobotPose(
            Function<String, Transform3d> cameraOffsets,
            BiConsumer<Pose2d, Double> estimateConsumer,
            String key,
            Blips blips) {
        for (Blip b : blips.tags) {
            Optional<Pose3d> tagInFieldCordsOptional = layout.getTagPose(b.id);
            if (!tagInFieldCordsOptional.isPresent())
                continue;

            Rotation2d gyroRotation = getPose.get().getRotation();

            Transform3d cameraInRobotCoordinates = cameraOffsets.apply(key);

            // Gyro only produces yaw so use zero roll and zero pitch
            Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            Pose3d robotPoseInFieldCoords = PoseEstimationHelper.getRobotPoseInFieldCoords(
                    cameraInRobotCoordinates, tagInFieldCordsOptional.get(), b, robotRotationInFieldCoordsFromGyro);
            // Pose2d robotInFieldCords = robotPoseInFieldCoords.toPose2d();
            Translation2d robotTranslationInFieldCoords = robotPoseInFieldCoords.getTranslation().toTranslation2d();

            currentRobotinFieldCoords = new Pose2d(robotTranslationInFieldCoords, gyroRotation);

            if (lastRobotInFieldCoords != null) {
                Transform2d translationSinceLast = currentRobotinFieldCoords.minus(lastRobotInFieldCoords);
                double xComponent = translationSinceLast.getX();
                double yComponent = translationSinceLast.getY();
                if (xComponent * xComponent +
                        yComponent * yComponent <= kVisionChangeToleranceMeters
                                * kVisionChangeToleranceMeters) {
                    // tell the vision indicator we have a fix.
                    indicator.go();
                    estimateConsumer.accept(currentRobotinFieldCoords, Timer.getFPGATimestamp() - .075);
                }
            }
            lastRobotInFieldCoords = currentRobotinFieldCoords;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Vision X", () -> currentRobotinFieldCoords.getX(), null);
        builder.addDoubleProperty("Vision Y", () -> currentRobotinFieldCoords.getY(), null);
        builder.addDoubleProperty("Vision Rotation", () -> currentRobotinFieldCoords.getRotation().getRadians(), null);
    }

}
