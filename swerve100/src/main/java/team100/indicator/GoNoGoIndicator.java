package team100.indicator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Demonstrates the use of an onboard LED strip as a boolean indicator, flashing
 * the whole strip periodically.
 * 
 * I'm using adafruit.com/products/3811, which are 60 LEDs in 0.5m. The current
 * draw is about 400mA per color for the full strip. The RoboRIO can produce
 * 2.2A on the 6V bus, so that's a budget for five colors, i.e. five strips each
 * showing one color (e.g. red, blue, or green), or two strips showing two
 * colors each (e.g. cyan, yellow, magenta) with some left over, or one strip
 * showing white, with some left over.
 * 
 * Since the purpose of this indicator is "go/no-go" indicating, let's use just
 * one color per strip, so we can have up to five strips with a simple hookup.
 * 
 * For more output and/or more colors, use the servo power module
 * (revrobotics/rev-11/1144), which can drive up to 15A at 6V (the absolute max
 * for these strips).
 */
public class GoNoGoIndicator {
    private static final int kStripLength = 60;

    private static enum State {
        OFF(Color.kBlack),
        NOGO(Color.kRed),
        GO(Color.kGreen),
        CUBE(new Color(254, 100, 0)),
        CONE(new Color(255, 0, 80)),
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

    private static final int kPort = 4;
    private final AddressableLED led;
    // private final Notifier notifier;
    private State active;
    private boolean flashOn;

    /**
     * @param freq Desired flashing frequency (Hz)
     */
    public GoNoGoIndicator(double freq) {
        active = State.ORANGE;

        led = new AddressableLED(kPort);
        led.setLength(kStripLength);
        led.start();
        State.ORANGE.set(led);
        // notifier = new Notifier(this::flip);
        // notifier.startPeriodic(0.5 / freq);
    }

    public void start(){
        led.start();
    }
    
    public void close() {
        led.close();
    }

    public void go() {
        // active = State.GO;
    }

    public void nogo() {
        // active = State.NOGO;
    }

    public void cone() {
        active = State.CONE;
        active.set(led);
    }

    public void cube() {
        active = State.CUBE;
        active.set(led);
    }

    public void orange() {
        active = State.ORANGE;
        State.ORANGE.set(led);
    }

    // private void flip() {
    //     if (flashOn) {
    //         State.OFF.set(led);
    //     } else {
    //         active.set(led);
    //     }
    //     flashOn ^= true; // flip with xor
    // }
}
