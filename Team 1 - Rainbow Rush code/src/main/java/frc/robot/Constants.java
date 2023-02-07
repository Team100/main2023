// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DrivetrainConstants {
    
        public static final class DrivetrainMotors {
            public static final class LeftMaster {
                public static final int CAN_ID = 18; //18
                public static final double KP = 0;
                public static final double KI = 0;
                public static final double KD = 0;
                public static final double KF = 0;
                public static final InvertType INVERTED = InvertType.None;
                public static final boolean SENSOR_PHASE = false;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;
                // public static final double NOMINAL_OUTPUT_FORWARD = 0.05;
                // public static final double NOMINAL_OUTPUT_REVERSE = -0.05;
                public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
            }

            public static final class LeftFollower {
                public static final int CAN_ID = 19; //19
                public static final double KP = 0;
                public static final double KI = 0;
                public static final double KD = 0;
                public static final double KF = 0;
                public static final InvertType INVERTED = InvertType.FollowMaster;
                public static final boolean SENSOR_PHASE = false;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;
                // public static final double NOMINAL_OUTPUT_FORWARD = 0.05;
                // public static final double NOMINAL_OUTPUT_REVERSE = -0.05;
                public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
            }

            public static final class RightMaster {
                public static final int CAN_ID = 10; //10
                public static final double KP = 0;
                public static final double KI = 0;
                public static final double KD = 0;
                public static final double KF = 0;
                public static final InvertType INVERTED = InvertType.InvertMotorOutput;
                public static final boolean SENSOR_PHASE = false;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;
                // public static final double NOMINAL_OUTPUT_FORWARD = 0.05;
                // public static final double NOMINAL_OUTPUT_REVERSE = -0.05;
                public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
            }

            public static final class RightFollower {
                public static final int CAN_ID = 11; //11
                public static final double KP = 0;
                public static final double KI = 0;
                public static final double KD = 0;
                public static final double KF = 0;
                public static final InvertType INVERTED = InvertType.None;
                public static final boolean SENSOR_PHASE = false;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;
                // public static final double NOMINAL_OUTPUT_FORWARD = 0.05;
                // public static final double NOMINAL_OUTPUT_REVERSE = -0.05;
                public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
            }
        }
        public static final class DrivetrainControls {
            public static final double RAMP_LIMIT = 0.2;
            public static final double ERROR_ADJUSTMENT_DRIVE = 0.0000025;
        }
    }



    public static final class IndexerConstants {
        public static final class IndexerSensors {
            public static final class FrontSensor {
                public static final int ID = 7;
            }

            public static final class RearSensor {
                public static final int ID = 8;
            }
        }

        public static final class IndexerMotionParameters {
            public static final double STAGE_ONE_PERCENT_OUTPUT_FORWARD = 0.8;
            public static final double STAGE_TWO_PERCENT_OUTPUT_FORWARD = 0.8;

            public static final double STAGE_ONE_PERCENT_OUTPUT_BACKWARD = -0.8;
            public static final double STAGE_TWO_PERCENT_OUTPUT_BACKWARD = -0.8;
        }

        public static final class IndexerMotors {
            public static final class IndexerStageOne {
                public static final int CAN_ID = 5;

                public static final boolean INVERT = false;
                public static final int FEEDBACK_PORT = 0;
                public static final boolean SENSOR_PHASE = false;

                public static final int TIMEOUT = 10;

                public static final boolean ENABLE_CURRENT_LIMIT = false;
                public static final int CURRENT_LIMIT = 80;
                public static final double OPEN_LOOP_RAMP = 0;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;

                public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;
            }

            public static final class IndexerStageTwo {
                public static final int CAN_ID = 4;
              
                public static final boolean INVERT = false;
                public static final int FEEDBACK_PORT = 0;
                public static final boolean SENSOR_PHASE = false;

                public static final int TIMEOUT = 10;

                public static final boolean ENABLE_CURRENT_LIMIT = false;
                public static final int CURRENT_LIMIT = 80;
                public static final double OPEN_LOOP_RAMP = 0;
                public static final double PEAK_OUTPUT_FORWARD = 1;
                public static final double PEAK_OUTPUT_REVERSE = -1;

                public static final IdleMode NEUTRAL_MODE = IdleMode.kBrake;
            }
        }
    }
}
