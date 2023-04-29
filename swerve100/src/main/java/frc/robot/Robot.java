package frc.robot;

import java.io.IOException;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team100.config.Identity;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    UsbCamera manipulatorCamera;

    private final DigitalInput auto1 = new DigitalInput(0);
    private final DigitalInput auto2 = new DigitalInput(1);
    private final DigitalInput auto4 = new DigitalInput(2);
    private final DigitalInput auto8 = new DigitalInput(3);

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

        if(getAlliance() == true){ //Aidan told me to do this blame him I wanted to use a ternary Sanjan
           m_alliance = DriverStation.Alliance.Blue;
        } else {
            m_alliance = DriverStation.Alliance.Red;

        }

        
        

        manipulatorCamera = CameraServer.startAutomaticCapture(0);
        manipulatorCamera.setResolution(240, 160);
        manipulatorCamera.setFPS(15);
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
        // m_robotContainer.ledStop();
        m_robotContainer.ledStart();
        m_robotContainer.enabled = false;
    }

    @Override
    public void disabledPeriodic() {
        // System.out.println("SWITCH VALUE " +getAutoSwitchValue());
        // System.out.println("ALLIANCE" + getAlliance());
        SmartDashboard.putNumber("SWITCH VALUE", getAutoSwitchValue());
        SmartDashboard.putBoolean("BLUE ALLIANCE", getAlliance());


        m_robotContainer.m_routine = getAutoSwitchValue();
    }

    private int getAutoSwitchValue() {
        int val = 0;
        if (auto8.get()) val += 8;
        if (auto4.get()) val += 4;
        if (auto2.get()) val += 2;
        if (auto1.get()) val += 1;
        // for (DigitalInput s : new DigitalInput[]{auto8, auto4, auto2, auto1}) {
        //     val <<= 1;
        //     val += s.get() ? 1 : 0;
        // }

        return 15 - val;
    }


    private boolean getAlliance() {
        int val = 0;
        if (alliance2.get()) val += 2;
        if (alliance1.get()) val += 1;
        // val = 2 * alliance2;
        // val += 
        // for (DigitalInput s : new DigitalInput[]{alliance2, alliance1}) {
        //     val <<= 1;
        //     val += s.get() ? 1 : 0;
        // }

        //0 is redAlliance
        if(val == 3){
            return false;
        }



        return true;
    }


    @Override
    public void autonomousInit() {
        int routine = getAutoSwitchValue();
        boolean isBlueAlliance = getAlliance();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand2(routine, isBlueAlliance);

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