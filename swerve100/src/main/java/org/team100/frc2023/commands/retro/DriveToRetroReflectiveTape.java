package org.team100.frc2023.commands.retro;

import java.io.IOException;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.lib.localization.Tapes;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinematics.FrameTransform;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveToRetroReflectiveTape extends CommandBase {
    public static class Config {
        public double filterTimeConstantS = 0.06;
        public double filterPeriodS = 0.02;
    }

    private final Config m_config = new Config();

    private final SwerveDriveSubsystem m_robotDrive;
    private final FrameTransform m_chassisSpeedFactory;
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;
    private final ObjectMapper object_mapper;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController xController;

    boolean firstRun = false;

    public DriveToRetroReflectiveTape(
            SwerveDriveSubsystem robotDrive,
            FrameTransform chassisSpeedFactory) {
        m_robotDrive = robotDrive;
        m_chassisSpeedFactory = chassisSpeedFactory;
        xFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        yFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        object_mapper = new ObjectMapper(new MessagePackFactory());

        yController = new ProfiledPIDController(1, 0, 0, new Constraints(0.25, 0.5));
        xController = new ProfiledPIDController(1.2, 0, 0, new Constraints(0.25, 0.5));
        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
    }

    @Override
    public void initialize() {
        firstRun = true;
    }

    @Override
    public void execute() {
        Rotation2d rot = m_robotDrive.getPose().getRotation();

        try {
            byte[] data = tapeSubscriber.get();
            Tapes tapes = object_mapper.readValue(data, Tapes.class);

            if (tapes.tapes.size() > 0) {
                double yMeasurment = yFilter.calculate(-tapes.tapes.get(0).pose_t[1]);
                double xMeasurment = xFilter.calculate(-tapes.tapes.get(0).pose_t[0]);

                if (firstRun == true) {
                    yController.reset(yMeasurment);
                    xController.reset(xMeasurment);
                    firstRun = false;
                }
                double yOutput = yController.calculate(yMeasurment, 0);
                double xOutput = xController.calculate(xMeasurment, -0.6);

                measurmentX.set(xMeasurment);
                measurmentY.set(yMeasurment);

                tagView.set(0);

                yOutputPub.set(yOutput);
                xOutputPub.set(xOutput);

                errorX.set(xController.getPositionError());
                errorY.set(yController.getPositionError());

                Twist2d twistM_S = new Twist2d(xOutput, yOutput, 0);
                Twist2d fieldRelative = m_chassisSpeedFactory.toFieldRelativeSpeeds(
                        twistM_S.dx, twistM_S.dy, twistM_S.dtheta, rot);
                m_robotDrive.driveInFieldCoords(fieldRelative);
            } else {
                m_robotDrive.stop();
                tagView.set(2);
            }

            setpointX.set(xController.getSetpoint().position);
            setpointY.set(yController.getSetpoint().position);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    //////////////////////////////////////////////////////////////

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Retro Tape");
    private final DoublePublisher setpointX = table.getDoubleTopic("Setpoint X").publish();
    private final DoublePublisher setpointY = table.getDoubleTopic("Setpoint Y").publish();
    private final DoublePublisher measurmentX = table.getDoubleTopic("Measurment X").publish();
    private final DoublePublisher measurmentY = table.getDoubleTopic("Measurment Y").publish();
    private final DoublePublisher errorX = table.getDoubleTopic("Error X").publish();
    private final DoublePublisher errorY = table.getDoubleTopic("Error Y").publish();
    private final DoublePublisher tagView = table.getDoubleTopic("Tag View").publish();
    private final DoublePublisher xOutputPub = table.getDoubleTopic("X Ouput View").publish();
    private final DoublePublisher yOutputPub = table.getDoubleTopic("Y Output View").publish();
    private final NetworkTable vision = inst.getTable("Retro Vision");
    private final RawSubscriber tapeSubscriber = vision.getRawTopic("tapes").subscribe("raw", new byte[0]);
}
