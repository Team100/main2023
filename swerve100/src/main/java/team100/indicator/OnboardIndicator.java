package team100.indicator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.MyAddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
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
public class OnboardIndicator {
    private static final int kStripLength = 60;
    private static final int kOffset = 30;

    private static enum Face {
        FRONT, BACK;
    }

    private static enum State {
        OFF(Color.kBlack),
        ORANGE(Color.kOrange),
        NO_FIX(Color.kRed),
        GOOD_FIX(Color.kGreen),
        CONE(Color.kYellow),
        CUBE(Color.kBlue);

        public final MyAddressableLEDBuffer buffer;

        private State(Color color) {
            buffer = new MyAddressableLEDBuffer(kStripLength);
            for (int i = 0; i < kStripLength; i++) {
                buffer.setLED(i, color);
            }
        }
    }

    private static final int kPort = 9;
    private final AddressableLED led;
    private final Notifier notifier;
    private State active;
    private boolean flashOn;
    private Rotation2d heading;
    private MyAddressableLEDBuffer activeBuffer;
    private int gamePiece;
    private int goodFixTimeOutCount;

    /**
     * @param freq Desired flashing frequency (Hz)
     */
    public OnboardIndicator(double freq) {
        active = State.NO_FIX;
        activeBuffer = new MyAddressableLEDBuffer(kStripLength);
        led = new AddressableLED(kPort);
        led.setLength(kStripLength);
        led.start();
        notifier = new Notifier(this::flip);
        notifier.startPeriodic(2.0 / freq);
    }

    /**
     * The strip of LEDs has different regions facing different directions; the
     * heading here tells the indicator which way to face each signal.
     * 
     * Indicators on the side don't do any good: the robot can never see sideways
     * and the cube/cone mode is not important when facing sideways.
     * 
     * So the only "views" are front and back.
     * 
     * When the robot is within 90 degrees of fore/aft either way, light up the
     * front/back indicators.
     * 
     * what should they do in the other 180 degree range? maybe just steady white.
     * 
     * Facing 180 degrees are the driver signals:
     * * red flashing = no fix
     * * green flashing = good fix
     * Also facing 180 are operator signals:
     * * blue: cube mode
     * * yellow: cone mode
     * Facing zero: operator signals
     * 
     * so out of the kLength strip, say half faces 0 and half faces 180. how to show
     * both "red" and "blue"? maybe just use half the strip for each signal, so the
     * strip
     * is divided into "operator" and "driver" halves.
     * 
     * the cone-mode signal should be steady
     * 
     * the good-fix signal should be steady
     * 
     * the no-fix signal should flash.
     *
     * so there are 13 combinations that could be precalculated,
     * 
     * front green + blue, back blue
     * front red + blue, back blue
     * front black + blue, back blue
     * front green + yellow, back yellow
     * front red + yellow, back yellow
     * front black + yellow, back yellow
     * front orange, back orange
     * front blue, back green + blue
     * front blue, back red + blue
     * front blue, back black + blue
     * front yellow, back green + yellow
     * front yellow, back red + yellow
     * front yellow, back black + Yellow
     * 
     * @param heading [-Math.PI, Math.PI]
     */
    public void setHeading(Rotation2d heading) {
        this.heading = heading;
    }

    public void go() {
        active = State.GOOD_FIX;
        // Set count down clock which decreases every time flip routine is called.  
        // When it goes to zero, 'active' will be set to State.NO_FIX.  Adjusting this 
        // value in conjuntion with the Notify frequency allows user to specify how 
        // lone the most recent position fix should be considered valid

        goodFixTimeOutCount = 3;  // This value is a guess which should provide about 3/4 sec T.O.?
        // Adjust as appropriate
    }

    public void nogo() {    // This is not expected to be called & should not be called
        active = State.NO_FIX;
    }

    public void setGamePieceType(int gamePiece) {
        this.gamePiece = gamePiece;  // Cone = 1, Cube =2
    }

    /**
     * this actually controls all indicator LEDs, not just flipping light to dark
     */
    private void flip() {
        double absHeadingRadiansNWU = Math.abs(heading.getRadians());
        if (absHeadingRadiansNWU < Math.PI / 4) {
            // front facing driver, back facing loader
            if (flashOn && active == State.NO_FIX) {
                System.arraycopy(State.OFF.buffer.getBuffer(), 0, activeBuffer.getBuffer(), 0, kOffset*4);
               //led.setData(State.OFF.buffer);
            } else {
                System.arraycopy(active.buffer.getBuffer(), 0, activeBuffer.getBuffer(), 0, kOffset*4);
                //led.setData(active.buffer);
            }

            if (gamePiece == 1) {   // Cone = 1
                System.arraycopy(State.CONE.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
            } else {
                System.arraycopy(State.CUBE.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
            }

        } else if (absHeadingRadiansNWU < 3.0 * Math.PI / 4) {
            // some sideways orientation
            // steady orange both sides
            // led.setData(State.ORANGE.buffer); *rff
            System.arraycopy(State.ORANGE.buffer.getBuffer(), 0, activeBuffer.getBuffer(), 0, kOffset*8);
        } else {
            // front facing loader, back facing driver

            if (flashOn && active == State.NO_FIX) {
               System.arraycopy(State.OFF.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
               //led.setData(State.OFF.buffer);
            } else {
                System.arraycopy(active.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
                //led.setData(active.buffer);
            }

            if (gamePiece == 1) {   // Cone = 1
                System.arraycopy(State.CONE.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
            } else {
                System.arraycopy(State.CUBE.buffer.getBuffer(), 0, activeBuffer.getBuffer(), kOffset*4, kOffset*4);
            }
              
        }

        led.setData(activeBuffer);

        flashOn ^= true; // flip with xor

        if (goodFixTimeOutCount-- <= 0) {  // If a good AprilTag Fix is not found, 
           //  set the active state to NO_FIX
            active = State.NO_FIX;
        }
    }
}
