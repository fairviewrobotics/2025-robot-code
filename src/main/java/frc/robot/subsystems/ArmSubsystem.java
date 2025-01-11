/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.ArmUtils;
import frc.robot.utils.ConfigManager;

public class ArmSubsystem extends SubsystemBase {
    private final SparkFlex pivotMotor =
            new SparkFlex(ArmConstants.PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.LEFT_MOTOR_ID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.RIGHT_MOTOR_ID);
    private final DigitalInput gamePieceLineBreak =
            new DigitalInput(ArmConstants.HAND_LINEBREAK_ID);

    private final ProfiledPIDController pivotPID =
            new ProfiledPIDController(
                    ArmConstants.PIVOT_P,
                    ArmConstants.PIVOT_I,
                    ArmConstants.PIVOT_D,
                    ArmConstants.PIVOT_CONSTRAINTS);

    private final ArmFeedforward pivotFF =
            new ArmFeedforward(
                    ArmConstants.PIVOT_KS,
                    ArmConstants.PIVOT_KG,
                    ArmConstants.PIVOT_KV,
                    ArmConstants.PIVOT_KA);

    /**
     * Sets hand motor speed
     *
     * @param speed Target motor speed
     */
    public void setHandSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
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

        pivotPID.setTolerance(
                ConfigManager.getInstance().get("arm_tolerance", ArmConstants.PIVOT_TOLERANCE));

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
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
        angle =
                MathUtil.clamp(
                        angle,
                        ConfigManager.getInstance()
                                .get("arm_pivot_min_angle", ArmConstants.PIVOT_MIN_ANGLE),
                        ConfigManager.getInstance()
                                .get("arm_pivot_max_angle", ArmConstants.PIVOT_MAX_ANGLE));
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
        return ArmUtils.encoderToRad(pivotMotor.getEncoder().getPosition());
    }
}
