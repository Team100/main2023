package org.team100.lib.sensors;

import org.team100.lib.config.Identity;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO: figure out why we disabled this mechanism during the 2023 season
public class RedundantGyro implements RedundantGyroInterface, Sendable {

    /** For robots without a gyro. */
    private static class Noop implements RedundantGyroInterface {

        @Override
        public float getRedundantGyroRateNED() {
            System.out.println("HIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII");
            return 0;
        }

        @Override
        public float getRedundantGyroZ() {
            return 0;
        }

        @Override
        public float getRedundantPitch() {
            return 0;
        }

        @Override
        public float getRedundantRoll() {
            return 0;
        }

        @Override
        public float getRedundantYawNED() {
            return 0;
        }
    }

    public static class Factory {
        private final Identity m_identity;

        public Factory(Identity identity) {
            m_identity = identity;
        }

        public RedundantGyroInterface get() {
            switch (m_identity) {
                case COMP_BOT:
                case SWERVE_ONE:
                case SWERVE_TWO:
                    return new RedundantGyro();
                default:
                    return new Noop();
            }
        }
    }

    private final AHRS m_gyro1;
    private final AHRS m_gyro2;
    private final Timer m_timer;

    private boolean gyrosWorking = true;
    private boolean gyro1Connected = true;
    private boolean gyro2Connected = true;
    private float gyroZOffset_I2C;
    private float gyroZOffset_USB;
    private boolean timeGap = false;
    float globalRedundRate = 0;
    float globalTMPUnits = 0;

    public RedundantGyro() {
        m_timer = new Timer();
        m_timer.start();

        m_gyro1 = new AHRS(SerialPort.Port.kUSB);
        m_gyro2 = new AHRS(I2C.Port.kMXP);
        m_gyro1.enableBoardlevelYawReset(true);
        m_gyro2.enableBoardlevelYawReset(true);
        m_gyro1.calibrate();
        m_gyro2.calibrate();

        while (m_timer.get() < 2) {
            // wait a bit
        }

        while ((m_gyro1.isConnected() && m_gyro1.isCalibrating() || m_gyro2.isConnected() && m_gyro2.isCalibrating())
                || timeGap) {
        }

        m_gyro1.zeroYaw();
        m_gyro2.zeroYaw();

        gyroZOffset_I2C = -m_gyro2.getRawGyroZ();
        gyroZOffset_USB = -m_gyro1.getRawGyroZ();
        SmartDashboard.putData("AHRSClass", this);
    }

    /** NOTE NOTE NOTE this is NED = clockwise positive = backwards */
    public float getRedundantYawNED() {
        if (!m_gyro1.isConnected()) {
            gyro1Connected = false;
        }
        if (!m_gyro2.isConnected()) {
            gyro2Connected = false;
        }
        float redundYaw = 0;
        int tmpInputs = 0;
        if (gyro1Connected) {
            redundYaw += m_gyro1.getYaw();
            tmpInputs += 1;
        }
        if (gyro2Connected) {
            redundYaw += m_gyro2.getYaw();
            tmpInputs += 1;
        }
        if (!gyro2Connected && !gyro1Connected) {
            gyrosWorking = false;
        }
        return m_gyro1.getYaw();
    }

    /**
     * TODO: is this really degrees?
     * 
     * @returns pitch in degrees [-180,180]
     */
    public float getRedundantPitch() {
        if (!m_gyro1.isConnected()) {
            gyro1Connected = false;
        }
        if (!m_gyro2.isConnected()) {
            gyro2Connected = false;
        }
        float redundPitch = 0;
        int tmpInputs = 0;
        if (gyro2Connected) {
            redundPitch += m_gyro2.getPitch();
            tmpInputs += 1;
        }
        if (gyro1Connected) {
            redundPitch += m_gyro1.getPitch();
            tmpInputs += 1;
        }
        if (!gyro2Connected && !gyro1Connected) {
            gyrosWorking = false;
        }
        return (redundPitch) / tmpInputs;
    }

    /**
     * TODO: is this really degrees?
     * 
     * @returns roll in degrees [-180,180]
     */
    public float getRedundantRoll() {
        if (!m_gyro1.isConnected()) {
            gyro1Connected = false;
        }
        if (!m_gyro2.isConnected()) {
            gyro2Connected = false;
        }
        float redundRoll = 0;
        int tmpInputs = 0;
        if (gyro2Connected) {
            redundRoll += m_gyro2.getRoll();
            tmpInputs += 1;
        }
        if (gyro1Connected) {
            redundRoll += m_gyro1.getRoll();
            tmpInputs += 1;
        }
        if (!gyro2Connected && !gyro1Connected) {
            gyrosWorking = false;
        }

        globalRedundRate = redundRoll;
        globalTMPUnits = tmpInputs;
        
        return (redundRoll) / tmpInputs;
    }

    /**
     * NOTE this is NED = clockwise positive = backwards
     * TODO: check the units
     */
    public float getRedundantGyroRateNED() {
        if (!m_gyro1.isConnected()) {
            gyro1Connected = false;
        }
        if (!m_gyro2.isConnected()) {
            gyro2Connected = false;
        }
        float redundRate = 0;
        int tmpInputs = 0;
        if (gyro2Connected) {
            redundRate += m_gyro2.getRate();
            tmpInputs += 1;
        }
        if (gyro1Connected) {
            redundRate += m_gyro1.getRate();
            tmpInputs += 1;
        }
        if (!gyro2Connected && !gyro1Connected) {
            gyrosWorking = false;
        }

        
        return (redundRate) / tmpInputs;
    }

    // do we need this ?
    public float getRedundantGyroZ() {
        if (!m_gyro1.isConnected()) {
            gyro1Connected = false;
        }
        if (!m_gyro2.isConnected()) {
            gyro2Connected = false;
        }
        float redundGyroZ = 0;
        int tmpInputs = 0;
        if (gyro2Connected) {
            redundGyroZ += m_gyro2.getRawGyroZ() + gyroZOffset_I2C;
            tmpInputs += 1;
        }
        if (gyro1Connected) {
            redundGyroZ += m_gyro1.getRawGyroZ() + gyroZOffset_USB;
            tmpInputs += 1;
        }
        if (!gyro2Connected && !gyro1Connected) {
            gyrosWorking = false;
        }
        return (redundGyroZ) / tmpInputs;
    }

    // if the gyro isn't working we're in deep trouble, i don't think this is worth
    // dealing with.
    // public boolean getGyroWorking() {
    // return gyrosWorking;
    // }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Gyro Redundant Roll (deg)", () -> getRedundantRoll(), null);
        builder.addDoubleProperty("Gyro Redundant Pitch (deg)", () -> getRedundantPitch(), null);
        builder.addDoubleProperty("Gyro 1 Angle (deg)", () -> m_gyro1.getAngle(), null);
        builder.addDoubleProperty("Gyro 2 Angle (deg)", () -> m_gyro2.getAngle(), null);
        builder.addDoubleProperty("Gyro 1 Fused (deg)", () -> m_gyro1.getFusedHeading(), null);
        builder.addDoubleProperty("Gyro 2 Fused (deg)", () -> m_gyro2.getFusedHeading(), null);
        builder.addDoubleProperty("Gyro Redundant Rate NED (rad/s)", () -> getRedundantGyroRateNED(), null);
        builder.addDoubleProperty("Gyro Redundant Yaw NED (rad)", () -> getRedundantYawNED(), null);
        builder.addDoubleProperty("Gyro 1 Yaw", () -> m_gyro1.getYaw(), null);
        builder.addDoubleProperty("Gyro 2 Yaw", () -> m_gyro2.getYaw(), null);
        builder.addDoubleProperty("Gyro 1 Angle Mod 360 (deg)", () -> m_gyro1.getAngle() % 360, null);
        builder.addDoubleProperty("Gyro 2 Angle Mod 360 (deg)", () -> m_gyro2.getAngle() % 360, null);
        builder.addDoubleProperty("Gyro 1 Compass Heading (deg)", () -> m_gyro1.getCompassHeading(), null);
        builder.addDoubleProperty("Gyro 2 Compass Heading (deg)", () -> m_gyro2.getCompassHeading(), null);
        builder.addBooleanProperty("Gyro 1 Connected", () -> gyro1Connected, null);
        builder.addBooleanProperty("Gyro 2 Connected", () -> gyro2Connected, null);
        builder.addBooleanProperty("Gyro 1 Connected Raw", () -> m_gyro1.isConnected(), null);
        builder.addBooleanProperty("Gyro 2 Connected Raw", () -> m_gyro2.isConnected(), null);
        builder.addBooleanProperty("Any Gyros Working", () -> gyrosWorking, null);
        builder.addFloatProperty("Redund Roll", () ->globalRedundRate, null);
        builder.addFloatProperty("TMP ", () ->globalTMPUnits, null);


    }
}
