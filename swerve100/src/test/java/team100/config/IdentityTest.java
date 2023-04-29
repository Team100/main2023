package team100.config;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Robot;

public class IdentityTest {

    @Test
    public void testDefaultIdentity() {
        // Robot instance is required for the JNI stuff to work.
        Robot robot = new Robot();
        RoboRioSim.resetData();
        assertEquals(Identity.BLANK, Identity.get());
        robot.close();
    }

    @Test
    public void testForceIdentity() {
        // Robot instance is required for the JNI stuff to work.
        Robot robot = new Robot();
        RoboRioSim.resetData();
        Identity.set(Identity.CAMERA_DOLLY);
        assertEquals(Identity.CAMERA_DOLLY, Identity.get());
        robot.close();
    }

    @Test
    public void testUnknownIdentity() {
        // Robot instance is required for the JNI stuff to work.
        Robot robot = new Robot();
        RoboRioSim.resetData();
        RoboRioSim.setSerialNumber("Hello");
        assertEquals(Identity.UNKNOWN, Identity.get());
        robot.close();
    }

    @Test
    public void testSerialNumber() {
        // Robot instance is required for the JNI stuff to work.
        Robot robot = new Robot();
        RoboRioSim.resetData();
        assertEquals("", RobotController.getSerialNumber(), "serial number is initally blank");
        RoboRioSim.setSerialNumber("Hello");
        assertEquals("Hello", RoboRioSim.getSerialNumber(), "sim retains the new value");
        assertEquals("Hello", RobotController.getSerialNumber(), "and it's also the real value");
        robot.close();
    }

}
