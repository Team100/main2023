package team100.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.msgpack.jackson.dataformat.MessagePackFactory;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.cscore.CameraServerCvJNI;
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
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveSubsystem;
import team100.config.Camera;
import team100.control.DualXboxControl;
import team100.indicator.GoNoGoIndicator;

/**
 * Extracts robot pose estimates from camera input.
 */
public class VisionDataProvider extends SubsystemBase implements TableEventListener {
    /**
     * If the tag is closer than this threshold, then the camera's estimate of tag
     * rotation might be more accurate than the gyro, so we use the camera's
     * estimate of tag rotation to update the robot pose. If the tag is further away
     * than this, then the camera-derived rotation is probably less accurate than
     * the gyro, so we use the gyro instead.
     * 
     * Set this to zero to disable tag-derived rotation and always use the gyro.
     * 
     * Set this to some large number (e.g. 100) to disable gyro-derived rotation and
     * always use the camera.
     */
    private static final double kTagRotationBeliefThresholdMeters = 1;

    private final Supplier<Pose2d> getPose;
    private final DoublePublisher timestamp_publisher;
    private final ObjectMapper object_mapper;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final double kVisionChangeToleranceMeters = .1;

    // TODO Make this private
    public AprilTagFieldLayoutWithCorrectOrientation layout;

    SwerveDriveSubsystem m_robotDrive;


    Rotation3d tagRotation;

    Pose2d currentRobotinFieldCoords;

    public final GoNoGoIndicator indicator;
    private Pose2d lastRobotInFieldCoords;

    DualXboxControl m_control;

    boolean tagFound = false;

    Timer m_timer;


    public VisionDataProvider(
            DriverStation.Alliance alliance,
            SwerveDrivePoseEstimator poseEstimator,
            Supplier<Pose2d> getPose,
            DualXboxControl control)
            throws IOException {

        // load the JNI (used by PoseEstimationHelper)
        CameraServerCvJNI.forceLoad();

        this.getPose = getPose;
        this.poseEstimator = poseEstimator;
        // indicator = new GoNoGoIndicator(1); // 8 hz = flash fast
        indicator = new GoNoGoIndicator(1);

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

        tagRotation = new Rotation3d();
        // Listen to ALL the updates in the vision table. :-)
        vision_table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), this);

        m_control = control;

        m_timer = new Timer();

        m_timer.start();
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
            // case REAR:
            //     return new Transform3d(new Translation3d(-0.326, 0.003, 0.332), new Rotation3d(0, -0.419, 3.142));
            case FRONT:
                return new Transform3d(new Translation3d(0.398, 0.075, 0.201), new Rotation3d(0, -0.35, 0));
            case RIGHT:
                return new Transform3d(new Translation3d(0.012, -0.264, 0.229), new Rotation3d(0, -0.401, -0.35)); //-0.399363
            case LEFT:
                return new Transform3d(new Translation3d(0.012, 0.159, 0.240), new Rotation3d(0, -0.332, 0.35)); //0.566077
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

        if(blips.tags.size() == 0){
            tagFound = false;
        }
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
                    cameraInRobotCoordinates,
                    tagInFieldCordsOptional.get(),
                    b,
                    robotRotationInFieldCoordsFromGyro,
                    kTagRotationBeliefThresholdMeters);

            // Pose2d robotInFieldCords = robotPoseInFieldCoords.toPose2d();
            Translation2d robotTranslationInFieldCoords = robotPoseInFieldCoords.getTranslation().toTranslation2d();

            currentRobotinFieldCoords = new Pose2d(robotTranslationInFieldCoords, gyroRotation);

            tagRotation = PoseEstimationHelper.blipToRotation(b);
            if (lastRobotInFieldCoords != null) {
                Transform2d translationSinceLast = currentRobotinFieldCoords.minus(lastRobotInFieldCoords);
                double xComponent = translationSinceLast.getX();
                double yComponent = translationSinceLast.getY();
                if (xComponent * xComponent +
                        yComponent * yComponent <= kVisionChangeToleranceMeters
                                * kVisionChangeToleranceMeters) {
                    // tell the vision indicator we have a fix.
                    // indicator.go();
                    // tagFound = true;
                    rumbleOn();

                    estimateConsumer.accept(currentRobotinFieldCoords, Timer.getFPGATimestamp() - .075);
                }
            }

            lastRobotInFieldCoords = currentRobotinFieldCoords;
        }

        rumbleCheck();

    }

    public void rumbleOn(){
        m_timer.restart();

        // m_control.rumbleOn();
    }

    public void rumbleCheck(){

        if(RobotContainer.isEnabled()){
            // System.out.println("NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO");
            if(m_timer.get() > 0.1){
                m_control.rumbleOff();
            } else {
                // m_timer.restart();
                m_control.rumbleOn();
            }
        }

        

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Vision X", () -> currentRobotinFieldCoords.getX(), null);
        builder.addDoubleProperty("Vision Y", () -> currentRobotinFieldCoords.getY(), null);
        builder.addDoubleProperty("Vision Rotation", () -> currentRobotinFieldCoords.getRotation().getRadians(), null);
        builder.addDoubleProperty("Tag Rotation", () -> tagRotation.getAngle(), null);
    }

}
