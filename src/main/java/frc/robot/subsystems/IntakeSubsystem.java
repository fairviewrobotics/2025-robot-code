/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

/** Subsystem to manage the intake (NOT HAND) */
public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor = new WPI_TalonSRX(ArmConstants.MOTOR_ID);

    /** Create a new intake subsystem */
    public IntakeSubsystem() {
        motor.setInverted(false);
    }

    /**
     * Set the speed of the intake
     *
     * @param speed The target speed in percent (0-1)
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Set the voltage of the intake
     *
     * @param voltage The target voltage (0-12)
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
