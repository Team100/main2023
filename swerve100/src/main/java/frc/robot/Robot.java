package frc.robot;

import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team100.config.Identity;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    UsbCamera manipulatorCamera;

    private final DigitalInput auto1 = new DigitalInput(0);
    private DigitalInput auto2 = new DigitalInput(1);
    private DigitalInput auto4 = new DigitalInput(2);
    private DigitalInput auto8 = new DigitalInput(3);

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version); // 2023.2.1
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.get().name());

        manipulatorCamera = CameraServer.startAutomaticCapture(0);
        manipulatorCamera.setResolution(240, 160);
        manipulatorCamera.setFPS(15);
        try {
            m_robotContainer = new RobotContainer();
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
        // m_robotContainer.ledStop();
        m_robotContainer.ledStart();
        m_robotContainer.enabled = false;
    }

    @Override
    public void disabledPeriodic() {
    }

    private int getAutoSwitchValue() {
        int val = 0;
        for (DigitalInput s : new DigitalInput[]{auto8, auto4, auto2, auto1}) {
            val <<= 1;
            val += s.get() ? 1 : 0;
        }

        return val;
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
        System.out.println("teleopInit");
        // m_robotContainer.ledStart();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.enabled = true;

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        System.out.println("testInit");
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        m_robotContainer.runTest2();
    }
}
