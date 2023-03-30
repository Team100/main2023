package team100.localization;

import java.util.ArrayList;
import java.util.List;


public class Tapes {
    /**
     * Elapsed time of the analysis in python.
     * TODO: calibrate this number so we can use it for Kalman filter updates.
     */
    // public final double et;

    /**
     * The set of targets seen by the camera.
     */
    public final List<Tape> tapes;

    /**
     * For the deserializer.
     */
    protected Tapes() {
        tapes = new ArrayList<Tape>();
    }

    @Override
    public String toString() {
        return "";
    }
}