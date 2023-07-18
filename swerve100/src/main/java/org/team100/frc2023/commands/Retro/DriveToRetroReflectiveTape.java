package org.team100.frc2023.commands.Retro;

import java.io.IOException;

import org.msgpack.jackson.dataformat.MessagePackFactory;
import org.team100.lib.localization.Tapes;
import org.team100.lib.subsystems.SwerveDriveSubsystem;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
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
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;
    private final ObjectMapper object_mapper;

    private final NetworkTableInstance inst;
    private final DoublePublisher setpointX;
    private final DoublePublisher setpointY;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher errorX;
    private final DoublePublisher errorY;
    private final DoublePublisher tagView;
    private final DoublePublisher xOutputPub;
    private final DoublePublisher yOutputPub;
    private final RawSubscriber tapeSubscriber;

    private final ProfiledPIDController yController;
    private final ProfiledPIDController xController;

    boolean firstRun = false;

    public DriveToRetroReflectiveTape(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        xFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        yFilter = LinearFilter.singlePoleIIR(m_config.filterTimeConstantS, m_config.filterPeriodS);
        object_mapper = new ObjectMapper(new MessagePackFactory());
        inst = NetworkTableInstance.getDefault();
        setpointX = inst.getTable("Retro Tape").getDoubleTopic("Setpoint X").publish();
        setpointY = inst.getTable("Retro Tape").getDoubleTopic("Setpoint Y").publish();
        measurmentX = inst.getTable("Retro Tape").getDoubleTopic("Measurment X").publish();
        measurmentY = inst.getTable("Retro Tape").getDoubleTopic("Measurment Y").publish();
        errorX = inst.getTable("Retro Tape").getDoubleTopic("Error X").publish();
        errorY = inst.getTable("Retro Tape").getDoubleTopic("Error Y").publish();
        tagView = inst.getTable("Retro Tape").getDoubleTopic("Tag View").publish();
        xOutputPub = inst.getTable("Retro Tape").getDoubleTopic("X Ouput View").publish();
        yOutputPub = inst.getTable("Retro Tape").getDoubleTopic("Y Output View").publish();
        tapeSubscriber = inst.getTable("RetroVision").getRawTopic("tapes").subscribe("raw", new byte[0]);
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
                m_robotDrive.driveMetersPerSec(new Twist2d(xOutput, yOutput, 0), false);
            } else {
                m_robotDrive.driveMetersPerSec(new Twist2d(0, 0, 0), false);
                tagView.set(2);
            }

            setpointX.set(xController.getSetpoint().position);
            setpointY.set(yController.getSetpoint().position);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
