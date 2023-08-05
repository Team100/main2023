package org.team100.lib.system.examples;

import org.team100.lib.math.AngularRandomVector;
import org.team100.lib.math.MeasurementUncertainty;
import org.team100.lib.math.RandomVector;
import org.team100.lib.math.Variance;
import org.team100.lib.math.WhiteNoiseVector;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * One-dimensional double-integrator with friction force proportional to
 * velocity.
 * 
 * In this case, we're modeling rotation, e.g. a wheel.
 */
public class ParamFrictionRotary1D extends Rotary1D {
    private double kV;
    private double kA;

    public ParamFrictionRotary1D(WhiteNoiseVector<N2> w, MeasurementUncertainty<N2> v, double kV, double kA) {
        super(w, v);
        this.kV = kV;
        this.kA = kA;
    }

    public ParamFrictionRotary1D(WhiteNoiseVector<N2> w, MeasurementUncertainty<N2> v) {
        this(w, v, 1, 1);
    }

    /**
     * xdot = f(x,u)
     * pdot = v
     * vdot = u - v
     * 
     * the x jacobian should be constant [0 1 0 -(kV/kA)]
     * the u jacobian should be constant [0 1]
     */
    public RandomVector<N2> f(RandomVector<N2> xmat, Matrix<N1, N1> umat) {
        double v = xmat.x.get(1, 0);
        double u = umat.get(0, 0);
        double pdot = v;
        double vdot = u - (kV/kA) * v;
        Matrix<N2,N1> xdotx = VecBuilder.fill(pdot, vdot);
        Matrix<N2,N2> xdotP = xmat.Kxx.copy().getValue();
        xdotP.fill(0);
        // propagate variance of x through f (u has zero variance)
        double vP = xmat.Kxx.get(1,1);
        xdotP.set(0,0, vP);
        xdotP.set(0,1, -(kV/kA) * vP);
        xdotP.set(1,0, -(kV/kA) * vP);
        xdotP.set(1,1, (kV/kA)*(kV/kA) * vP);
        // note that xdot needs no wrapping, don't return an AngularRandomVector here.
        return new RandomVector<>(xdotx, new Variance<>(xdotP));
    }

    @Override
    public Matrix<N1, N1> finvWrtU(RandomVector<N2> x, RandomVector<N2> xdot) {
        double a = xdot.x.get(1, 0);
        double v = x.x.get(1, 0);
        return VecBuilder.fill(a + (kV/kA) * v);
    }

    @Override
    public RandomVector<N2> finvWrtX(RandomVector<N2> xdot, Matrix<N1, N1> u) {
        double pdot = xdot.x.get(0, 0);
        double vdot = xdot.x.get(1, 0);
        double uu = u.get(0, 0);
        // "pseudoinverse"
        double v = (uu - vdot + pdot) / 2;
        Matrix<N2, N1> xx = new Matrix<>(Nat.N2(), Nat.N1());
        xx.set(1, 0, v);
        Matrix<N2, N2> xP = new Matrix<>(Nat.N2(), Nat.N2());
        xP.set(0, 0, 1e9); // position: "don't know" variance
        xP.set(1, 1, xdot.Kxx.get(0, 0)); // better P?
        // Full state, return Angular.
        return new AngularRandomVector<>(xx, new Variance<>(xP));
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }
}
