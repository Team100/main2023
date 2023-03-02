package team100.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.localization.Blip;
import frc.robot.localization.VisionDataProvider;

public class VisionDataProviderTest {
    private static final double kDelta = 0.01;

    @Test
    public void testBlipToTranslation() {
        // one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip blip = new Blip(5,
                new double[][] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { 
                        { -2 },
                        { -1 },
                        { 3 } });

        Translation3d blipTranslation = VisionDataProvider.blipToTranslation(blip);

        // does not change the frame, just copies the values
        assertEquals(-2, blipTranslation.getX(), kDelta);
        assertEquals(-1, blipTranslation.getY(), kDelta);
        assertEquals(3, blipTranslation.getZ(), kDelta);

    }

    @Test
    public void testCameraToNWU() {
        // one meter up, two meters left, three meters ahead
        // rotation doesn't matter
        Blip blip = new Blip(5,
                new double[][] {
                        { 1, 0, 0 },
                        { 0, 1, 0 },
                        { 0, 0, 1 } },
                new double[][] { 
                        { -2 },
                        { -1 },
                        { 3 } });

        Translation3d blipTranslation = VisionDataProvider.blipToTranslation(blip);
        Translation3d nwuTranslation = VisionDataProvider.cameraToNWU(blipTranslation);

        // NWU values now
        assertEquals(3, nwuTranslation.getX(), kDelta);
        assertEquals(2, nwuTranslation.getY(), kDelta);
        assertEquals(1, nwuTranslation.getZ(), kDelta);

    }
}
