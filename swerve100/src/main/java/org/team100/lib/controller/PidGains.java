package org.team100.lib.controller;

public class PidGains {
    public final Double p;
    public final Double i;
    public final Double d;
    public final Double integratorRange;
    public final Double tolerance;
    public final Boolean continuous;

    public PidGains(double p, double i, double d, double range, double tolerance, boolean continuous) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.integratorRange = range;
        this.tolerance = tolerance;
        this.continuous = continuous;
    }
    public PidGains(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.integratorRange = null;
        this.tolerance = null;
        this.continuous = null;
        
    }
}
