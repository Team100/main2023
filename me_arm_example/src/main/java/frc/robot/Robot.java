package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {
    private final Servo m_swing = new Servo(0);
    private final Servo m_boom = new Servo(1);
    private final Servo m_stick = new Servo(2);
    private final Servo m_grip = new Servo(3);
    XboxController m_controller = new XboxController(0);

    @Override
    public void teleopInit() {
        m_swing.setAngle(90);
        m_boom.setAngle(90);
        m_stick.setAngle(90);
        m_grip.setAngle(90);
    }

    @Override
    public void teleopPeriodic() {
        // this is the SAE pattern, see https://en.wikipedia.org/wiki/Excavator_controls
        m_swing.setAngle(m_swing.getAngle() + m_controller.getLeftX());
        m_boom.setAngle(m_boom.getAngle() + m_controller.getLeftY());
        m_stick.setAngle(m_stick.getAngle() + m_controller.getRightY());
        m_grip.setAngle(m_grip.getAngle() + m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis());
    }
}
