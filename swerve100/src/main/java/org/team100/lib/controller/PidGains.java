package org.team100.lib.controller;

public class PidGains {
    public final double p;
    public final double i;
    public final double d;

    public PidGains(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }
}
