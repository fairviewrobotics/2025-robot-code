/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    public SparkFlex climberMotor = new SparkFlex(15, SparkFlex.MotorType.kBrushless);
    public TalonSRX lockMotor = new TalonSRX(16);

    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    public void setLockSpeed(double speed) {
        lockMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }
}
