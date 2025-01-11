package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;
import frc.robot.utils.ArmUtils;

public class ArmSubsystem extends SubsystemBase {
    private final SparkFlex pivotMotor = new SparkFlex(ArmConstants.PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final WPI_TalonSRX leftMotor = new WPI_TalonSRX(ArmConstants.LEFT_MOTOR_ID);
    private final WPI_TalonSRX rightMotor = new WPI_TalonSRX(ArmConstants.RIGHT_MOTOR_ID);

    private final ProfiledPIDController pivotPID = new ProfiledPIDController(
            ArmConstants.PIVOT_P,
            ArmConstants.PIVOT_I,
            ArmConstants.PIVOT_D,
            ArmConstants.PIVOT_CONSTRAINTS
    );

    private final ArmFeedforward pivotFF = new ArmFeedforward(
            ArmConstants.PIVOT_KS,
            ArmConstants.PIVOT_KG,
            ArmConstants.PIVOT_KV,
            ArmConstants.PIVOT_KA
    );

    /**
     * sets hand motor speed
     * @param speed
     */
    private void setHandSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    /**
     * Checks if the arm is at set angle
     * @return if the arm is at set angle
     */
    public boolean atTargetAngle() {
        return this.pivotPID.atSetpoint();
    }

    public ArmSubsystem() {

        pivotPID.setTolerance(ArmConstants.PIVOT_TOLERANCE);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
    }

    /**
     * sets pivot motor speed
     * @param speed
     */
    private void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    /**
     * moves arm to a set angle
     * @param angle
     */
    public void setPivotAngle(double angle) {
        angle = MathUtil.clamp(angle, ArmConstants.PIVOT_MIN_ANGLE, ArmConstants.PIVOT_MAX_ANGLE);
        double pidValue = pivotPID.calculate(getPivotAngle(), angle);
        double ffValue = pivotFF.calculate(angle, 0);

        setPivotSpeed(pidValue + ffValue);
    }

    /**
     * Get the angle of the arm motor
     * @return arm angle
     */
    public double getPivotAngle() {
        return ArmUtils.encoderToRad(pivotMotor.getEncoder().getPosition());
    }
}
