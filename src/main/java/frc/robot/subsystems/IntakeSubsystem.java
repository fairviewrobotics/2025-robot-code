package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonSRX motor = new WPI_TalonSRX(ArmConstants.LEFT_MOTOR_ID);

    public IntakeSubsystem() {
        motor.setInverted(false);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

}
