/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import frc.robot.utils.ConfigManager;
import org.apache.logging.log4j.core.pattern.AbstractStyleNameConverter.Green;
import org.apache.logging.log4j.core.pattern.AbstractStyleNameConverter.Yellow;

// Thx Matero for half the code
public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle =
            new CANdle(
                    (int) ConfigManager.getInstance().get("led_device_id", LEDConstants.DEVICE_ID),
                    "rio");

    private final int ledCount =
            (int) ConfigManager.getInstance().get("led_count", LEDConstants.LED_COUNT);

    private int r = 0;
    private int g = 0;
    private int b = 0;

    public enum AnimationTypes {
        Red,
        Green,
        Yellow,
        Blue,
        Off
    }

    private AnimationTypes currentAnimation = null;
    private Animation animation = null;

    private boolean pureRGB = false;

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = CANdle.LEDStripType.RGB;
        config.brightnessScalar = 0.1;
    }

    private void setRGB(int r, int g, int b) {
        this.pureRGB = true;
        this.r = r;
        this.g = g;
        this.b = b;
        //        this.candle.setLEDs(r, g, b);
    }

    public void setAnimation(AnimationTypes toChange) {
        currentAnimation = toChange;
        this.pureRGB = false;

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

    @Override
    public void periodic() {
        candle.animate(animation);
        if (this.pureRGB) candle.setLEDs(this.r, this.g, this.b);
    }
}
