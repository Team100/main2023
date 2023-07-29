package org.team100.lib.experiments;

/** An experiment is something that can be selectively enabled. */
public enum Experiment {
    /** Smoothing chassis speeds, using 254 code. */
    UseSetpointGenerator,
    /** Offload the drive PID to the motor controller. */
    UseClosedLoopDrive,
    /** Offload the steering PID to the motor controller. */
    UseClosedLoopSteering
}
