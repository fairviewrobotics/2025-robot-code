/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.ConfigManager;

public class ArmSubsystem extends SubsystemBase {
    private final SparkFlex pivotMotor =
            new SparkFlex(ArmConstants.PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private final AbsoluteEncoder pivotAbsEncoder = pivotMotor.getAbsoluteEncoder();

    private final SparkFlexConfig pivotConfig = new SparkFlexConfig();

    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.LEFT_MOTOR_ID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.RIGHT_MOTOR_ID);
    private final DigitalInput gamePieceLineBreak =
            new DigitalInput(ArmConstants.HAND_LINEBREAK_ID);

    private final DoubleEntry armEncoderPos =
            NetworkTableInstance.getDefault()
                    .getTable("Arm")
                    .getDoubleTopic("EncoderPos")
                    .getEntry(getPivotAngle());

    private final DoubleEntry armEncoderVelocity =
            NetworkTableInstance.getDefault()
                    .getTable("Arm")
                    .getDoubleTopic("EncoderVelocity")
                    .getEntry(getPivotSpeed());

    private final ProfiledPIDController pivotPID =
            new ProfiledPIDController(
                    ArmConstants.PIVOT_P,
                    ArmConstants.PIVOT_I,
                    ArmConstants.PIVOT_D,
                    ArmConstants.PIVOT_CONSTRAINTS);

    private final ArmFeedforward pivotFF =
            new ArmFeedforward(
                    ConfigManager.getInstance().get("arm_ks", ArmConstants.PIVOT_KS),
                    ConfigManager.getInstance().get("arm_kg", ArmConstants.PIVOT_KG),
                    ConfigManager.getInstance().get("arm_kv", ArmConstants.PIVOT_KV),
                    ConfigManager.getInstance().get("arm_ka", ArmConstants.PIVOT_KA));

    /**
     * Sets hand motor speed
     *
     * @param speed Target motor speed
     */
    public void setIntakeSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void setIntakeVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    public void setPivotVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    /**
     * Checks if the arm is at set angle
     *
     * @return if the arm is at set angle
     */
    public boolean atTargetAngle() {
        return this.pivotPID.atSetpoint();
    }

    public ArmSubsystem() {

        pivotPID.setTolerance(ArmConstants.PIVOT_TOLERANCE);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);

        pivotConfig.inverted(true);
        pivotConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotConfig
                .absoluteEncoder
                .inverted(true)
                .positionConversionFactor(2 * Math.PI) // radians
                .velocityConversionFactor(2 * Math.PI / 60.0);

        pivotMotor.configure(
                pivotConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Sets pivot motor speed
     *
     * @param speed Target pivot speed
     */
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * Checks if the hand has a game piece
     *
     * @return returns if the hand has a game piece
     */
    public boolean hasGamePiece() {
        return !gamePieceLineBreak.get();
    }

    /**
     * moves arm to a set angle
     *
     * @param angle Target pivot angle
     */
    public void setPivotAngle(double angle) {
        angle = MathUtil.clamp(angle, ArmConstants.PIVOT_MIN_ANGLE, ArmConstants.PIVOT_MAX_ANGLE);

        double pidValue = pivotPID.calculate(getPivotAngle(), angle);
        double ffValue = pivotFF.calculate(angle, 0);

        setPivotSpeed(pidValue + ffValue);
    }

    /**
     * Get the angle of the arm motor
     *
     * @return arm angle
     */
    public double getPivotAngle() {
        return (pivotAbsEncoder.getPosition() >= Math.PI)
                ? pivotAbsEncoder.getPosition() - 2 * Math.PI - ArmConstants.PIVOTENCODEROFFSET
                : pivotAbsEncoder.getPosition() - ArmConstants.PIVOTENCODEROFFSET;
    }

    public double getPivotSpeed() {
        return pivotAbsEncoder.getVelocity();
    }

    public void periodic() {
        armEncoderPos.set(getPivotAngle());
        armEncoderVelocity.set(getPivotSpeed());
        pivotPID.setP(ConfigManager.getInstance().get("arm_p", ArmConstants.PIVOT_P));
        pivotPID.setI(ConfigManager.getInstance().get("arm_i", ArmConstants.PIVOT_I));
        pivotPID.setD(ConfigManager.getInstance().get("arm_d", ArmConstants.PIVOT_D));
    }

    public void resetPID() {
        pivotPID.reset(getPivotAngle(), getPivotSpeed());
    }

    public void resetEncoder() {
        pivotMotor.getEncoder().setPosition(0.0);
    }
}
