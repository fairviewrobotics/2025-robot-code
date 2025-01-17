package frc.robot.subsystems;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle candle = new CANdle(1, "rio");

    private final int ledCount = 0;

    private int r = 0;
    private int g = 0;
    private int b = 0;

    private Animation animation = null;


















}
