/* Black Knights Robotics (C) 2025 */
package org.blackknights.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem to controller the climber */
public class ClimberSubsystem extends SubsystemBase {
    public SparkFlex climberMotor = new SparkFlex(15, SparkFlex.MotorType.kBrushless);
    public TalonSRX lockMotor = new TalonSRX(16);

    public SparkFlexConfig climberMotorConfig = new SparkFlexConfig();

    public ClimberSubsystem() {
        climberMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        climberMotorConfig.smartCurrentLimit(40, 40);

        climberMotor.configure(
                climberMotorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Set the speed of the climber motor
     *
     * @param speed The target speed in percent (-1-1)
     */
    public void setClimberSpeed(double speed) {
        climberMotor.set(speed);
    }

    /**
     * Set the speed of the lock motor
     *
     * @param speed The speed in percent (-1-1)
     */
    public void setLockSpeed(double speed) {
        lockMotor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }
}
