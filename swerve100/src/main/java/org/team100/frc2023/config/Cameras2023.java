package org.team100.frc2023.config;

import org.team100.lib.config.Camera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** How the cameras were configured on the 2023 robot. */
public class Cameras2023 {

    /**
     * Camera views relative to the robot center at the floor.
     */
    public static Transform3d cameraOffset(String serialNumber) {

        Camera cam = Camera.get(serialNumber);

        switch (cam) {
            // case B: // REAR
            // return new Transform3d(
            // new Translation3d(-0.326, 0.003, 0.332),
            // new Rotation3d(0, -0.419, 3.142));
            case A: // FRONT
                return new Transform3d(
                        new Translation3d(0.398, 0.075, 0.201),
                        new Rotation3d(0, -0.35, 0));
            case D: // RIGHT
                return new Transform3d(
                        new Translation3d(0.012, -0.264, 0.229),
                        new Rotation3d(0, -0.401, -0.35));
            case C: // LEFT
                return new Transform3d(
                        new Translation3d(0.012, 0.159, 0.240),
                        new Rotation3d(0, -0.332, 0.35));
            case UNKNOWN:
                return new Transform3d(
                        new Translation3d(0.254, 0.127, 0.3),
                        new Rotation3d(0, 0, 0));
            default:
                return new Transform3d();
        }
    }

}
