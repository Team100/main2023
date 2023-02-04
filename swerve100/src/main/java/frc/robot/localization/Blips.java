package frc.robot.localization;

import java.util.ArrayList;
import java.util.List;

/** The entire payload from the camera. */
public class Blips {
    public final double et;
    // TODO change this name
    public final List<Blip> tags; 

    protected Blips() {
        et = 0;
        tags = new ArrayList<Blip>();
    }

    @Override
    public String toString() {
        return "Blips [et=" + et + ", tags=" + tags + "]";
    }
}