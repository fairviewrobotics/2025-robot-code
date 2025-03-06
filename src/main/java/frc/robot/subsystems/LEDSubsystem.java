/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

/** Subsystem for LED colors and animations Thx Matero for half the code */
public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(LEDConstants.DEVICE_ID, "rio");
    private int r = 0;
    private int g = 0;
    private int b = 0;

    /** Possible animation/colors */
    public enum AnimationTypes {
        Red,
        Green,
        Yellow,
        Blue,
        Off
    }

    /** Create new LED subsystem */
    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = CANdle.LEDStripType.RGB;
        config.brightnessScalar = 0.1;
    }

    /**
     * Set LEDs to static color
     *
     * @param r LED red value 0-255
     * @param b LED blue value 0-255
     * @param g LED green value 0-255
     */
    private void setRGB(int r, int g, int b) {
        if (this.r != r || this.b != b || this.g != g) {
            candle.setLEDs(r, g, b);
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    /**
     * Set animation/color of LEDs
     *
     * @param toChange Animation to change the animation to
     */
    public void setAnimation(AnimationTypes toChange) {
        switch (toChange) {
            case Red:
                setRGB(255, 0, 0);
            case Yellow:
                setRGB(255, 255, 0);
            case Blue:
                setRGB(0, 0, 255);
            case Green:
                setRGB(0, 255, 0);
            case Off:
                setRGB(0, 0, 0);
        }
    }
}
