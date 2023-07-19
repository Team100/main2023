package org.team100.lib.motion.drivetrain;

import org.team100.frc2023.subsystems.SwerveModuleFactory;
import org.team100.lib.config.Identity;

/** Creates collections according to Identity. */
public class SwerveModuleCollectionFactory {
    public static SwerveModuleCollection get(Identity identity, double currentLimit) {
        switch (identity) {
            case COMP_BOT:
                return new SwerveModuleCollection(
                        SwerveModuleFactory.WCPModule(
                                "Front Left",
                                11, // drive CAN
                                30, // turn CAN
                                0, // turn encoder
                                0.642472, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Front Right",
                                12, // drive CAN
                                32, // turn CAN
                                1, // turn encoder
                                0.740540, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Left",
                                21, // drive CAN
                                31, // turn CAN
                                2, // turn encoder
                                0.149713, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Right",
                                22, // drive CAN
                                33, // turn CAN
                                3, // turn encoder
                                0.296298, // turn offset
                                currentLimit));
            case SWERVE_TWO:
                return new SwerveModuleCollection(
                        SwerveModuleFactory.AMModule(
                                "Front Left",
                                11, // drive CAN
                                3, // turn PWM
                                1, // turn encoder
                                0.911606, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Front Right",
                                12, // drive CAN
                                1, // turn PWM
                                3, // turn encoder
                                0.083566, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Rear Left",
                                21, // drive CAN
                                2, // turn PWM
                                0, // turn encoder
                                0.871471, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Rear Right",
                                22, // drive CAN
                                0, // turn PWM
                                2, // turn encoder
                                0.605593, // turn offset
                                currentLimit));
            case SWERVE_ONE:
                return new SwerveModuleCollection(
                        SwerveModuleFactory.AMModule(
                                "Front Left",
                                11, // drive CAN
                                0, // turn PWM0
                                3, // turn encoder
                                0.69, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Front Right",
                                12, // drive CAN
                                2, // turn PWM
                                0, // turn encoder
                                0.72, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Rear Left",
                                21, // drive CAN
                                1, // turn PWM
                                2, // turn encoder
                                0.37, // turn offset
                                currentLimit),
                        SwerveModuleFactory.AMModule(
                                "Rear Right",
                                22, // drive CAN
                                3, // turn PWM
                                1, // turn encoder
                                0.976726, // turn offset
                                currentLimit));
            case BLANK:
                return new SwerveModuleCollection(
                        SwerveModuleFactory.WCPModule(
                                "Front Left",
                                11, // drive CAN
                                30, // turn CAN
                                2, // turn encoder
                                0.812, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Front Right",
                                12, // drive CAN
                                32, // turn CAN
                                0, // turn encoder
                                0.382, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Left",
                                21, // drive CAN
                                31, // turn CAN
                                3, // turn encoder
                                0.172, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Right",
                                22, // drive CAN
                                33, // turn CAN
                                1, // turn encoder
                                0.789, // turn offset
                                currentLimit));
            case CAMERA_DOLLY:
                return new SwerveModuleCollection(
                        SwerveModuleFactory.WCPModule(
                                "Front Left",
                                11, // drive CAN
                                30, // turn CAN
                                0, // turn encoder
                                0.267276, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Front Right",
                                12, // drive CAN
                                32, // turn CAN
                                1, // turn encoder
                                0.872709, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Left",
                                21, // drive CAN
                                31, // turn CAN
                                2, // turn encoder
                                0.754813, // turn offset
                                currentLimit),
                        SwerveModuleFactory.WCPModule(
                                "Rear Right",
                                22, // drive CAN
                                33, // turn CAN
                                3, // turn encoder
                                0.477917, // turn offset
                                currentLimit));
            default:
                throw new IllegalStateException("Identity is not swerve: " + Identity.get().name());
        }
    }

}
