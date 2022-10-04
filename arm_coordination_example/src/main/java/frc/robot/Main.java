package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Starts the robot.
 */
public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
