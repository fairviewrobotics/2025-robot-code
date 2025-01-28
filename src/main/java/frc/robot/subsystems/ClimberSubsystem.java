/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final WPI_TalonSRX rotaryMotor = new WPI_TalonSRX(ClimberConstants.ROTARY_MOTOR_ID);
    // Bag motor
    private final SparkFlex jointMotor =
            new SparkFlex(ClimberConstants.JOINT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    // Neovortex arm motor
    private final ProfiledPIDController pidController =
            new ProfiledPIDController(
                    ClimberConstants.JOINT_P,
                    ClimberConstants.JOINT_I,
                    ClimberConstants.JOINT_D,
                    ClimberConstants.ARM_CONSTRAINTS);

    // private final RelativeEncoder rotaryEncoder = rotaryMotor.getEncoder
    // I dont think talonsrx has built in encoder.
    private final RelativeEncoder jointEncoder = jointMotor.getEncoder();

    private boolean isRotaryRotating = false;
    private int currentDirection = 1;

    private final ArmFeedforward climberFF =
            new ArmFeedforward(
                    ClimberConstants.JOINT_S,
                    ClimberConstants.JOINT_G,
                    ClimberConstants.JOINT_V,
                    ClimberConstants.JOINT_A);

    public ClimberSubsystem() {

        SparkFlexConfig elbowMotorConfig = new SparkFlexConfig();

        jointMotor.configure(
                elbowMotorConfig,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
                // When we turn motor on, we do not reset parameters.
                // Persist means that it doesn't clear the parameters when it turns off
                );
    }

    /**
     * Sets pidController target position.
     *
     * @param position Position is target PID position.
     */
    public void setJointPosition(double position) {
        pidController.setGoal(position);
    }

    /**
     * Sets rotary motor(Bag motor) voltage.
     *
     * @param voltage Voltage is target voltage for rotary motor.
     */
    public void setRotaryVoltage(double voltage) {
        rotaryMotor.setVoltage(voltage);
    }

    /**
     * gets joint motor velocity(Neo vortex)
     *
     * @return joint encoder velocity.
     */
    public double getJointVelocity() {
        return jointEncoder.getVelocity();
    }

    /**
     * Sets the voltage of the joint motor(Neo vortex)
     *
     * @param voltage Voltage is target voltage
     */
    public void setJointVoltage(double voltage) {
        jointMotor.setVoltage(voltage);
    }

    /**
     * Stops rotary motor when it hits cage and sees current spike.
     *
     * @param currentMax Current maximum when rotary motor hits hardstop.
     */
    public void stopRotaryMotor(double currentMax) {
        if (rotaryMotor.getSupplyCurrent() > currentMax) {
            rotaryMotor.setVoltage(0);
        }
    }

    /**
     * Gets joint encoder position
     *
     * @return joint encoder position
     */
    public double getJointPosition() {
        return jointEncoder.getPosition();
    }

    /**
     * Rotates rotary motor. Sets boolean isRotaryRotating = true when it is called. We set
     * currentDirection *= -1 as it will reverse the direction of the rotary motor after it has
     * rotated.
     *
     * @param voltage Voltage is target voltage.
     */
    public void rotateRotary(double voltage) {
        rotaryMotor.setVoltage(voltage * currentDirection);
        isRotaryRotating = true;
        currentDirection *= -1;
    }

    @Override
    public void periodic() {
        if (isRotaryRotating) {
            stopRotaryMotor(ClimberConstants.CURRENT_MAX);
        }
        double pidCalc = pidController.calculate(getJointPosition());
        double ffCalc = climberFF.calculate(getJointPosition(), getJointVelocity());
        setJointVoltage(pidCalc + ffCalc);
    }

    // Arm motor needs pid for current
    // Rotate until current function
    // Rotate at a specific amount of volts until spike in current and then stop rotating.

}
