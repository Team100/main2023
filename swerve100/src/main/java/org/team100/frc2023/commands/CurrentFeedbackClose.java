package org.team100.frc2023.commands;

import org.team100.frc2023.subsystems.Manipulator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CurrentFeedbackClose extends CommandBase {
    private final double closedCurrent;
    private final double force;
    private boolean finishedFlag;
    double stepForce;

    private final Manipulator manip;

    public CurrentFeedbackClose(Manipulator subsystem, double closedCurrentParm, double forceParm) {
        manip = subsystem;
        closedCurrent = closedCurrentParm;
        force = forceParm;

    }

    @Override
    public void initialize() {
        finishedFlag = false;
        stepForce = 0.0;
    }

    @Override
    public void execute() {
        if (manip.getStatorCurrent() <= closedCurrent) {
            stepForce = Math.min(stepForce + 0.07 * force, force);
            manip.set(stepForce, 30);
        } else {
            manip.set(0, 30);
            finishedFlag = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finishedFlag;
    }

}
