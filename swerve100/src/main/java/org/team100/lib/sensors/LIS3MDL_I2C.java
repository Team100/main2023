package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;

/**
 * Adafruit 4517 contains this magnetometer, interfaced with I2C on MXP.
 * 
 * It uses NED coordinates, clockwise-positive, like a compass.
 * 
 * Constants come from github.com/stm32duino/LIS3MDL/LIS3MDL_MAG_Driver.h
 * 
 * NOTE: it is probably not possible to use this on a robot, because of the
 * effect of motor currents.
 */
public class LIS3MDL_I2C implements Supplier<Rotation2d>, Sendable {
    // 8-bit addr is 0x38 so 7-bit is shifted, 0x1C
    private static final byte LIS3MDL_MAG_I2C_ADDRESS_LOW = (byte) 0x38;
    private static final byte LIS3MDL_MAG_CTRL_REG1 = (byte) 0x20;
    private static final byte LIS3MDL_MAG_CTRL_REG2 = (byte) 0x21;
    private static final byte LIS3MDL_MAG_CTRL_REG3 = (byte) 0x22;
    // for now i only care about X and Y
    private static final byte LIS3MDL_MAG_OUTX_L = (byte) 0x28;
    private static final byte LIS3MDL_MAG_OUTY_L = (byte) 0x2A;

    /**
     * Output Data Rate. These are bits 2-4 in CTRL_REG1.
     */
    public enum LIS3MDL_MAG_DO_T {
        LIS3MDL_MAG_DO_0_625Hz(0x00),
        LIS3MDL_MAG_DO_1_25Hz(0x04),
        LIS3MDL_MAG_DO_2_5Hz(0x08),
        LIS3MDL_MAG_DO_5Hz(0x0C),
        LIS3MDL_MAG_DO_10Hz(0x10),
        LIS3MDL_MAG_DO_20Hz(0x14),
        LIS3MDL_MAG_DO_40Hz(0x18),
        LIS3MDL_MAG_DO_80Hz(0x1C);

        public final byte value;
        public static final byte LIS3MDL_MAG_DO_MASK = (byte) 0x1C;

        private LIS3MDL_MAG_DO_T(int value) {
            this.value = (byte) value;
        }
    }

    /**
     * Sensitivity. These are bits 5-6 in CTRL_REG2.
     */
    public enum LIS3MDL_MAG_FS_T {
        LIS3MDL_MAG_FS_4Ga(0x00),
        LIS3MDL_MAG_FS_8Ga(0x20),
        LIS3MDL_MAG_FS_12Ga(0x40),
        LIS3MDL_MAG_FS_16Ga(0x60);

        public final byte value;
        public static final byte LIS3MDL_MAG_FS_MASK = (byte) 0x60;

        private LIS3MDL_MAG_FS_T(int value) {
            this.value = (byte) value;
        }
    }

    /**
     * Operating Mode for X and Y. These are bits 5-6 in CTRL_REG1.
     */
    public enum LIS3MDL_MAG_OM_T {
        LIS3MDL_MAG_OM_LOW_POWER(0x00),
        LIS3MDL_MAG_OM_MEDIUM(0x20),
        LIS3MDL_MAG_OM_HIGH(0x40),
        LIS3MDL_MAG_OM_ULTRA_HIGH(0x60);

        public final byte value;
        public static final byte LIS3MDL_MAG_OM_MASK = (byte) 0x60;

        private LIS3MDL_MAG_OM_T(int value) {
            this.value = (byte) value;
        }
    }

    /**
     * Operating Mode. These are bits 1-2 in CTRL_REG3.
     */
    public enum LIS3MDL_MAG_MD_T {
        LIS3MDL_MAG_MD_CONTINUOUS(0x00),
        LIS3MDL_MAG_MD_SINGLE(0x01),
        LIS3MDL_MAG_MD_POWER_DOWN(0x02),
        LIS3MDL_MAG_MD_POWER_DOWN_AUTO(0x03);

        public final byte value;
        public static final byte LIS3MDL_MAG_MD_MASK = (byte) 0x03;

        private LIS3MDL_MAG_MD_T(int value) {
            this.value = (byte) value;
        }
    }

    private final I2C m_i2c;

    public LIS3MDL_I2C() {
        this(LIS3MDL_MAG_I2C_ADDRESS_LOW,
                LIS3MDL_MAG_DO_T.LIS3MDL_MAG_DO_80Hz, // fastest
                LIS3MDL_MAG_FS_T.LIS3MDL_MAG_FS_4Ga, // most sensitive
                LIS3MDL_MAG_OM_T.LIS3MDL_MAG_OM_HIGH,
                LIS3MDL_MAG_MD_T.LIS3MDL_MAG_MD_CONTINUOUS);
    }

    public LIS3MDL_I2C(byte i2cAddress, LIS3MDL_MAG_DO_T odr,
            LIS3MDL_MAG_FS_T fs, LIS3MDL_MAG_OM_T om, LIS3MDL_MAG_MD_T md) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
        setODR(odr);
        setFS(fs);
        setOperatingModeXY(om);
        enable(md);
    }

    private void setODR(LIS3MDL_MAG_DO_T newValue) {
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LIS3MDL_MAG_CTRL_REG1, 1, buf);
        byte value = buf.get();
        value &= ~LIS3MDL_MAG_DO_T.LIS3MDL_MAG_DO_MASK;
        value |= newValue.value;
        m_i2c.write(LIS3MDL_MAG_CTRL_REG1, value);
    }

    private void setFS(LIS3MDL_MAG_FS_T newValue) {
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LIS3MDL_MAG_CTRL_REG2, 1, buf);
        byte value = buf.get();
        value &= ~LIS3MDL_MAG_FS_T.LIS3MDL_MAG_FS_MASK;
        value |= newValue.value;
        m_i2c.write(LIS3MDL_MAG_CTRL_REG2, value);
    }

    private void setOperatingModeXY(LIS3MDL_MAG_OM_T newValue) {
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LIS3MDL_MAG_CTRL_REG1, 1, buf);
        byte value = buf.get();
        value &= ~LIS3MDL_MAG_OM_T.LIS3MDL_MAG_OM_MASK;
        value |= newValue.value;
        m_i2c.write(LIS3MDL_MAG_CTRL_REG1, value);
    }

    private void enable(LIS3MDL_MAG_MD_T newValue) {
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LIS3MDL_MAG_CTRL_REG3, 1, buf);
        byte value = buf.get();
        value &= ~LIS3MDL_MAG_MD_T.LIS3MDL_MAG_MD_MASK;
        value |= newValue.value;
        m_i2c.write(LIS3MDL_MAG_CTRL_REG3, value);
    }

    /**
     * NED angle, like a compass.
     */
    @Override
    public Rotation2d get() {
        m_raw = getRaw();
        return m_raw;
    }

    public Rotation2d m_raw;

    /**
     * NED angle, like a compass.
     */
    private Rotation2d getRaw() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LIS3MDL_MAG_OUTX_L, 2, buf);
        short xValue = buf.getShort();
        buf.clear();
        m_i2c.read(LIS3MDL_MAG_OUTY_L, 2, buf);
        short yValue = buf.getShort();
        return new Rotation2d(xValue, yValue);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro"); // gyro kinda like a compass
        builder.addDoubleProperty("ned radians", () -> m_raw.getRadians(), null);
    }
}
