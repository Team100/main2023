package team100.control;

/**
 * Inspect the connected controls and return a Control class corresponding to
 * what's connected.
 */
public class ControlSelect {

    public static Control getControl() {
        return  new DualXboxControl();
       // return  new FlightControl();
    }

}
