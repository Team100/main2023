package org.team100.lib.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;

/**
 * Adafruit 4517 contains this gyro, interfaced with I2C.
 * 
 * It uses NWU coordinates, clockwise-negative, like most of WPILIB.
 * 
 * Constants come from github.com/stm32duino/LSM6DSOX/lsm6dsox_reg.h
 */
public class LSM6DSOX_I2C implements Sendable {
    // 8-bit addr is 0xd5, so 7-bit is shifted, 0x6A
    private static final byte LSM6DSOX_I2C_ADD_L = (byte) 0xD5;
    private static final byte LSM6DSOX_CTRL2_G = (byte) 0x11;
    // private static final byte LSM6DSOX_OUTX_L_G = (byte) 0x22;
    // private static final byte LSM6DSOX_OUTY_L_G = (byte) 0x24;
    // for now i only care about yaw
    private static final byte LSM6DSOX_OUTZ_L_G = (byte) 0x26;

    // This is based on quick look at bias, could be better.
    private static final int kRawOffset = -50;

    /**
     * Output Data Rate. These are the high 4 bits in CTRL2_G.
     */
    public enum LSM6DSOX_ODR_G_T {
        LSM6DSOX_GY_ODR_OFF(0b0000_0000),
        LSM6DSOX_GY_ODR_12Hz5(0b0001_0000),
        LSM6DSOX_GY_ODR_26Hz(0b0010_0000),
        LSM6DSOX_GY_ODR_52Hz(0b0011_0000),
        LSM6DSOX_GY_ODR_104Hz(0b0100_0000),
        LSM6DSOX_GY_ODR_208Hz(0b0101_0000),
        LSM6DSOX_GY_ODR_417Hz(0b0110_0000),
        LSM6DSOX_GY_ODR_833Hz(0b0111_0000),
        LSM6DSOX_GY_ODR_1667Hz(0b1000_0000),
        LSM6DSOX_GY_ODR_3333Hz(0b1001_0000),
        LSM6DSOX_GY_ODR_6667Hz(0b1010_0000);

        // the correctly bit-offset value
        public final byte value;
        // Mask to erase the relevant bits.
        // (Note the ST code doesn't use a mask, it uses bit fields in a struct.)
        public static final byte mask = (byte) 0b1111_0000;

        private LSM6DSOX_ODR_G_T(int value) {
            this.value = (byte) value;
        }
    }

    /**
     * Sensitivity. These are bits 1-3 in CTRL2_G.
     */
    public enum LSM6DSOX_FS_G_T {
        LSM6DSOX_125dps(0b0000_0010, 4.375),
        LSM6DSOX_250dps(0b0000_0000, 8.75),
        LSM6DSOX_500dps(0b0000_0100, 17.5),
        LSM6DSOX_1000dps(0b0000_1000, 35),
        LSM6DSOX_2000dps(0b0000_1100, 70);

        public final byte value;
        public final double mdps;
        // Mask to erase the relevant bits.
        // (Note the ST code doesn't use a mask, it uses bit fields in a struct.)
        public static final byte mask = (byte) 0b0000_1110;

        private LSM6DSOX_FS_G_T(int value, double mdps) {
            this.value = (byte) value;
            this.mdps = mdps;
        }
    }

    private final I2C m_i2c;
    private LSM6DSOX_FS_G_T m_scale;

    /**
     * Use default of 500 degrees per sec. 250 dps seems too low.
     * Update at 100hz, comfortably above robot period of 50hz.
     */
    public LSM6DSOX_I2C() {
        this(LSM6DSOX_I2C_ADD_L,
                LSM6DSOX_ODR_G_T.LSM6DSOX_GY_ODR_104Hz,
                LSM6DSOX_FS_G_T.LSM6DSOX_500dps);
    }

    public LSM6DSOX_I2C(byte i2cAddress,
            LSM6DSOX_ODR_G_T odr,
            LSM6DSOX_FS_G_T fs) {
        m_i2c = new I2C(I2C.Port.kMXP, i2cAddress >>> 1);
        setGyroDataRate(odr);
        setGyroScale(fs);
    }

    private void setGyroDataRate(LSM6DSOX_ODR_G_T data_rate) {
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LSM6DSOX_CTRL2_G, 1, buf);
        byte ctrl2 = buf.get();
        ctrl2 &= ~LSM6DSOX_ODR_G_T.mask;
        ctrl2 |= data_rate.value;
        m_i2c.write(LSM6DSOX_CTRL2_G, ctrl2);
    }

    private void setGyroScale(LSM6DSOX_FS_G_T scale) {
        m_scale = scale;
        ByteBuffer buf = ByteBuffer.allocate(1);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LSM6DSOX_CTRL2_G, 1, buf);
        byte ctrl2 = buf.get();
        ctrl2 &= ~LSM6DSOX_FS_G_T.mask;
        ctrl2 |= scale.value;
        m_i2c.write(LSM6DSOX_CTRL2_G, ctrl2);
    }


    /**
     * NWU yaw rate in radians/sec.
     */
    public double getRate() {
        m_yawRateRaw = (double) getYawRateRaw();
        return m_yawRateRaw * m_scale.mdps * Math.PI / 180000;
    }

    public double m_yawRateRaw;

    /**
     * NWU yaw rate, 16 bits, signed.  Unit depends on full-scale setting.
     */
    private int getYawRateRaw() {
        ByteBuffer buf = ByteBuffer.allocate(2);
        buf.order(ByteOrder.LITTLE_ENDIAN);
        m_i2c.read(LSM6DSOX_OUTZ_L_G, 2, buf);
        return buf.getShort() - kRawOffset;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("yaw rate raw", () -> m_yawRateRaw, null);
    }

}
