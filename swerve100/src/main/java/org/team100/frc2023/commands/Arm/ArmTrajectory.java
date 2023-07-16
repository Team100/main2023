package org.team100.frc2023.commands.Arm;

import org.team100.frc2023.subsystems.arm.ArmController;
import org.team100.frc2023.subsystems.arm.ArmPosition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmTrajectory extends CommandBase {
    public static class Config {
        public double oscillatorFrequencyHz = 2;
        /** amplitude (each way) of oscillation in encoder units */
        public double oscillatorScale = 0.025;
        /** start oscillating when this close to the target. */
        public double oscillatorZone = 0.1;
    }

    private final Config m_config = new Config();
    private final ArmController m_arm;
    private final ArmPosition m_position;
    private final boolean m_oscillate;
    private final ArmTrajectories m_trajectories;
    private final Timer m_timer;

    private final PIDController m_upperController;
    private final PIDController m_lowerController;

    private final NetworkTableInstance inst;
    private final DoublePublisher measurmentX;
    private final DoublePublisher measurmentY;
    private final DoublePublisher setpointUpper;
    private final DoublePublisher setpointLower;

    private Trajectory m_trajectory;

    /**
     * Go to the specified position and optionally oscillate when you get there.
     */
    public ArmTrajectory(ArmPosition position, ArmController arm, boolean oscillate) {
        m_arm = arm;
        m_position = position;
        m_oscillate = oscillate;

        if (m_position != ArmPosition.SAFE) {
            m_trajectories = new ArmTrajectories(new TrajectoryConfig(12, 2));
        } else {
            m_trajectories = new ArmTrajectories(new TrajectoryConfig(9, 1.5));
        }

        m_timer = new Timer();

        if (m_position == ArmPosition.SAFE) {
            m_upperController = new PIDController(2.5, 0, 0);
            m_lowerController = new PIDController(2.5, 0, 0);
        } else {
            m_upperController = new PIDController(4, 0.2, 0.05);
            m_lowerController = new PIDController(3, 0, 0.1);
            m_upperController.setTolerance(0.001);
            m_lowerController.setTolerance(0.001);
        }

        inst = NetworkTableInstance.getDefault();
        measurmentX = inst.getTable("Arm Trajec").getDoubleTopic("measurmentX").publish();
        measurmentY = inst.getTable("Arm Trajec").getDoubleTopic("measurmentY").publish();
        setpointUpper = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Upper").publish();
        setpointLower = inst.getTable("Arm Trajec").getDoubleTopic("Setpoint Lower").publish();

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_trajectory = m_trajectories.makeTrajectory(m_arm.getArmAngles(), m_position, m_arm.cubeMode);
    }

    public void execute() {
        if (m_trajectory == null) {
            return;
        }
        double currentUpper = m_arm.getUpperArm();
        double currentLower = m_arm.getLowerArm();

        double curTime = m_timer.get();
        State desiredState = m_trajectory.sample(curTime);

        double desiredUpper = desiredState.poseMeters.getX();
        double desiredLower = desiredState.poseMeters.getY();

        double upperError = desiredUpper - currentUpper;

        if (m_oscillate && upperError < m_config.oscillatorZone) {
            desiredUpper += oscillator(curTime);
        }

        measurmentX.set(currentUpper);
        measurmentY.set(currentLower);

        setpointUpper.set(desiredUpper);
        setpointLower.set(desiredLower);

        m_arm.setUpperArm(m_upperController.calculate(currentUpper, desiredUpper));
        m_arm.setLowerArm(m_lowerController.calculate(currentLower, desiredLower));

    }

    private double oscillator(double timeSec) {
        return m_config.oscillatorScale * Math.sin(2 * Math.PI * m_config.oscillatorFrequencyHz * timeSec);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setUpperArm(0);
        m_arm.setLowerArm(0);
    }

    @Override
    public boolean isFinished() {
        if (m_position == ArmPosition.SAFEWAYPOINT) {
            return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        }
        return false;
    }
}
