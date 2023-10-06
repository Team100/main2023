package org.team100.lib.controller;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

public class LQRGains {
    public final double maxV;
    public final double maxA;
    public final LinearSystem<N2, N1, N1> plant;
    public final double eX;
    public final double eV;
    public final double mX;
    public final double Q1;
    public final double Q2;
    public final double R;
    public final double maxVolts;
    public final double seconds;
    public LQRGains(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R) {
        this(maxV, maxA, plant, eX, eV, mX, Q1, Q2, R, 12, .02);
    }

    public LQRGains(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R, double maxVolts) {
        this(maxV, maxA, plant, eX, eV, mX, Q1, Q2, R, maxVolts, .02);
    }
    public LQRGains(double maxV, double maxA, LinearSystem<N2, N1, N1> plant, double eX, double eV, double mX, double Q1, double Q2, double R, double maxVolts, double seconds) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.plant = plant;
        this.eX = eX;
        this.eV = eV;
        this.mX = mX;
        this.Q1 = Q1;
        this.Q2 = Q2;
        this.R = R;
        this.maxVolts = maxVolts;
        this.seconds = seconds;
    }
}
