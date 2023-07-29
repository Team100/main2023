package org.team100.lib.indicator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDIndicator {
    private static final int kStripLength = 60;
    private final AddressableLED led;

    /**
     * This enum exists to prepopulate the buffers, so they can be set atomically,
     * which is faster.
     */
    private static enum State {
        // TODO: fix these colors, i think they're all wrong now
        BLACK(new Color(0, 0, 0)),
        RED(new Color(255, 0, 0)),
        GREEN(new Color(0, 0, 255)),
        PURPLE(new Color(254, 100, 0)),
        YELLOW(new Color(255, 0, 80)),
        ORANGE(new Color(255, 0, 30));

        private final AddressableLEDBuffer buffer;

        private State(Color color) {
            buffer = new AddressableLEDBuffer(kStripLength);
            for (int i = 0; i < kStripLength; i++) {
                buffer.setLED(i, color);
            }
        }

        public void set(AddressableLED led) {
            led.setData(buffer);
        }
    }

    public LEDIndicator(int port) {
        led = new AddressableLED(port);
        led.setLength(kStripLength);
        led.start();
        State.ORANGE.set(led);
    }

    public void yellow() {
        State.YELLOW.set(led);
    }

    public void red() {
        State.RED.set(led);
    }

    public void green() {
        State.GREEN.set(led);
    }

    public void purple() {
        State.PURPLE.set(led);
    }

    public void orange() {
        State.ORANGE.set(led);
    }

    public void close() {
        led.close();
    }
}
