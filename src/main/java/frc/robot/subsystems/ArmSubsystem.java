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

    private final ProfiledPIDController handPID = new ProfiledPIDController(
            ArmConstants.HAND_P,
            ArmConstants.HAND_I,
            ArmConstants.HAND_D,
            ArmConstants.HAND_CONSTRAINTS
    );

    private void setSpeed(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public boolean atTargetAngle() {
        return this.pivotPID.atSetpoint();
    }

    public ArmSubsystem() {

        pivotPID.setTolerance(ArmConstants.PIVOT_TOLERANCE);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
    }

    private void runPivotVolts(double volts) {
        pivotMotor.set(volts);
    }

    public void setPivotAngle(double angle) {
        angle = MathUtil.clamp(angle, ArmConstants.PIVOT_MIN_ANGLE, ArmConstants.PIVOT_MAX_ANGLE);
        double pidValue = pivotPID.calculate(getPivotAngle(), angle);
        double ffValue = pivotFF.calculate(angle, 0);

        pivotMotor.setVoltage(pidValue + ffValue);
    }

    public double getPivotAngle() {
        return ArmUtils.encoderToRad(pivotMotor.getEncoder().getPosition());
    }
}
