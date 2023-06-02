package org.team100.frc2023.kinematics;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Replacement for the static methods in wpilib ChassisSpeeds. Supports
 * lag correction.
 */
public class ChassisSpeedFactory {
    private final DoubleSupplier gyroRateRadS;
    private final double delaySec;

    /**
     * @param gyroRateRadS supplies gyro rate, NWU, counterclockwise-positive
     *                     (rad/s). Note this is the *measurement* not the
     *                     *setpoint*.
     * @param delayS       total delay to correct for in rotational sensing and
     *                     actuation (s)
     */
    public ChassisSpeedFactory(DoubleSupplier gyroRateRadS, double delayS) {
        this.gyroRateRadS = gyroRateRadS;
        this.delaySec = delayS;
    }

    /**
     * Non-corrected version, identical to WPI.
     */
    public ChassisSpeedFactory() {
        this(() -> 0.0, 0.0);
    }

    Rotation2d correctAngle(Rotation2d robotAngle) {
        return robotAngle.plus(new Rotation2d(gyroRateRadS.getAsDouble() * delaySec));
    }

    /**
     * Converts a user provided field-relative set of speeds into a robot-relative
     * ChassisSpeeds object.
     *
     * @param vxMetersPerSecond     The component of speed in the x direction
     *                              relative to the field.
     *                              Positive x is away from your alliance wall.
     * @param vyMetersPerSecond     The component of speed in the y direction
     *                              relative to the field.
     *                              Positive y is to your left when standing behind
     *                              the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot. Note this is the
     *                              *setpoint*, not the *measurement*.
     * @param robotAngle            The angle of the robot as measured by a
     *                              gyroscope. The robot's angle is
     *                              considered to be zero when it is facing directly
     *                              away from your alliance station wall.
     *                              Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public ChassisSpeeds fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {
        double gyroRate = gyroRateRadS.getAsDouble() * delaySec;
        robotAngle = robotAngle.plus(new Rotation2d(gyroRate));
        return new ChassisSpeeds(
                vxMetersPerSecond * robotAngle.getCos() + vyMetersPerSecond * robotAngle.getSin(),
                -vxMetersPerSecond * robotAngle.getSin() + vyMetersPerSecond * robotAngle.getCos(),
                omegaRadiansPerSecond);
    }

    /**
     * Converts a user provided field-relative ChassisSpeeds object into a
     * robot-relative ChassisSpeeds object.
     *
     * @param fieldRelativeSpeeds The ChassisSpeeds object representing the speeds
     *                            in the field frame
     *                            of reference. Positive x is away from your
     *                            alliance wall. Positive y is to your left when
     *                            standing behind the alliance wall.
     * @param robotAngle          The angle of the robot as measured by a gyroscope.
     *                            The robot's angle is
     *                            considered to be zero when it is facing directly
     *                            away from your alliance station wall.
     *                            Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of
     *         reference.
     */
    public ChassisSpeeds fromFieldRelativeSpeeds(
            ChassisSpeeds fieldRelativeSpeeds, Rotation2d robotAngle) {
        return fromFieldRelativeSpeeds(
                fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond,
                fieldRelativeSpeeds.omegaRadiansPerSecond,
                robotAngle);
    }
}
