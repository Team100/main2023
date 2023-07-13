package org.team100.frc2023;

import java.io.IOException;

import org.team100.lib.config.Identity;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final DigitalInput auto1 = new DigitalInput(0);
    private final DigitalInput auto2 = new DigitalInput(1);
    private final DigitalInput auto4 = new DigitalInput(2);
    private final DigitalInput auto8 = new DigitalInput(3);

    // TODO: why two switches to represent one bit?
    private final DigitalInput alliance1 = new DigitalInput(4);
    private final DigitalInput alliance2 = new DigitalInput(5);

    private RobotContainer m_robotContainer;

    @Override
    public void close() {
        super.close();
        auto1.close();
        auto2.close();
        auto4.close();
        auto8.close();
        alliance1.close();
        alliance2.close();
    }

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.get().name());

        DriverStation.Alliance m_alliance;
        DataLogManager.start();

        if (getAlliance() == true) {
            m_alliance = DriverStation.Alliance.Blue;
        } else {
            m_alliance = DriverStation.Alliance.Red;

        }

        try {
            m_robotContainer = new RobotContainer(m_alliance);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

    }

    @Override
    public void disabledInit() {
        m_robotContainer.ledStart();
        RobotContainer.enabled = false;
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putNumber("SWITCH VALUE", getAutoSwitchValue());
        SmartDashboard.putBoolean("BLUE ALLIANCE", getAlliance());

        m_robotContainer.m_routine = getAutoSwitchValue();

        double keyList = NetworkTableInstance.getDefault().getTable("Vision").getKeys().size();

        SmartDashboard.putNumber("KEY LIST", keyList);

        if (keyList == 0) {
            m_robotContainer.red();
        } else {
            m_robotContainer.green();
        }
    }

    private int getAutoSwitchValue() {
        int val = 0;
        if (auto8.get())
            val += 8;
        if (auto4.get())
            val += 4;
        if (auto2.get())
            val += 2;
        if (auto1.get())
            val += 1;
        return 15 - val;
    }

    // TODO: use RED and BLUE here
    // TODO: important, make this complain if neither bit is set
    private boolean getAlliance() {
        int val = 0;
        if (alliance2.get())
            val += 2;
        if (alliance1.get())
            val += 1;
        // 0 is redAlliance
        if (val == 3) {
            return false;
        }
        return true;
    }

    @Override
    public void autonomousInit() {
        int routine = getAutoSwitchValue();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand2(routine);

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        RobotContainer.enabled = true;
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        m_robotContainer.runTest2();
    }
}