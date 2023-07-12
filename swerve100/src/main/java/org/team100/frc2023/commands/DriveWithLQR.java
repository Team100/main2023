// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveWithLQR extends CommandBase {
    private final SwerveDriveSubsystem m_robotDrive;

    TrapezoidProfile.State goal;

    boolean done = false;

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(
            5,
            5);
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    private final LinearSystem<N2, N1, N1> m_drivePlant = LinearSystemId.identifyPositionSystem(.5, 0.025);

    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            m_drivePlant,
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            m_drivePlant,
            VecBuilder.fill(0.5, 10.0), // qelms.
            VecBuilder.fill(12), // relms. Control effort (voltage) tolerance. Decrease this to more
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

    private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(m_drivePlant, m_controller, m_observer,
            12.0, 0.020);

    public DriveWithLQR(SwerveDriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;
        done = false;
        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        m_loop.reset(VecBuilder.fill(m_robotDrive.getPose().getX(), 0));

        goal = new TrapezoidProfile.State(m_robotDrive.getPose().getX() + 1, 0.0);

        m_lastProfiledReference = new TrapezoidProfile.State(m_robotDrive.getPose().getX(), 0);

    }

    @Override
    public void execute() {
        m_lastProfiledReference = (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference)).calculate(0.020);

        if (m_lastProfiledReference.position == goal.position) {
            done = true;
        }
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

        m_loop.correct(VecBuilder.fill(m_robotDrive.getPose().getX()));

        m_loop.predict(0.020);

        double nextVoltage = m_loop.getU(0);

        m_robotDrive.drive(nextVoltage, 0, 0, true);

    }

    @Override
    public void end(boolean interrupted) {
        done = false;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
