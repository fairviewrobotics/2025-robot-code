/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;
import frc.robot.utils.ConfigManager;

public class ElevatorSubsystem extends SubsystemBase {

    // TODO Elevator move to position function

    // Two neovortexes
    private final SparkFlex leftElevatorMotor =
            new SparkFlex(ElevatorConstants.LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex rightElevatorMotor =
            new SparkFlex(ElevatorConstants.RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // Relative Encoders
    private final RelativeEncoder leftEncoder = leftElevatorMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightElevatorMotor.getEncoder();

    // Linebreaks
    private final DigitalInput topLineBreak = new DigitalInput(ElevatorConstants.TOP_LINEBREAK_ID);
    private final DigitalInput bottomLineBreak =
            new DigitalInput(ElevatorConstants.BOTTOM_LINEBREAK_ID);

    private final ProfiledPIDController elevatorPID =
            new ProfiledPIDController(
                    ElevatorConstants.ELEVATOR_P,
                    ElevatorConstants.ELEVATOR_I,
                    ElevatorConstants.ELEVATOR_D,
                    ElevatorConstants.CONSTRAINTS);

    private final ElevatorFeedforward elevatorFF =
            new ElevatorFeedforward(
                    ElevatorConstants.ELEVATOR_KS,
                    ElevatorConstants.ELEVATOR_KG,
                    ElevatorConstants.ELEVATOR_KV,
                    ElevatorConstants.ELEVATOR_KA);

    private final DoubleEntry elevatorEncoderPos =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getDoubleTopic("EncoderPos")
                    .getEntry(getElevatorPosition());

    private final DoubleEntry elevatorLEncoderPos =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getDoubleTopic("EncoderLPos")
                    .getEntry(0.0);

    private final DoubleEntry elevatorREncoderPos =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getDoubleTopic("EncoderRPos")
                    .getEntry(0.0);

    private final DoubleEntry elevatorCurrent =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getDoubleTopic("Output Current")
                    .getEntry(rightElevatorMotor.getOutputCurrent());

    private final DoubleEntry elevatorVoltage =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getDoubleTopic("Applied Voltage")
                    .getEntry(rightElevatorMotor.getAppliedOutput());

    public double zeroVoltage =
            ConfigManager.getInstance()
                    .get("elevator_zero_voltage", ElevatorConstants.ELEVATOR_ZEROING_VOLTAGE);

    /** Subsystem for the elevator */
    public ElevatorSubsystem() {
        SparkFlexConfig rightElevatorMotorConfig = new SparkFlexConfig();
        SparkFlexConfig leftElevatorMotorConfig = new SparkFlexConfig();

        elevatorPID.setTolerance(
                ConfigManager.getInstance()
                        .get("elevator_pid_tolerance", ElevatorConstants.ELEVATOR_TOLERANCE));

        rightElevatorMotorConfig.inverted(true);
        rightElevatorMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        leftElevatorMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        rightElevatorMotorConfig.smartCurrentLimit(40, 40);
        leftElevatorMotorConfig.smartCurrentLimit(40, 40);

        //        rightElevatorMotorConfig.secondaryCurrentLimit(40);

        rightElevatorMotor.configure(
                rightElevatorMotorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        leftElevatorMotor.configure(
                leftElevatorMotorConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        elevatorPID.setGoal(0);
    }

    /**
     * Set the elevator speed in percent
     *
     * @param speed Target percent
     */
    public void setElevatorSpeed(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }

    /**
     * Set the voltage of the motors for the elevator
     *
     * @param voltage The target voltage
     */
    public void setVoltage(double voltage) {
        leftElevatorMotor.setVoltage(voltage);
        rightElevatorMotor.setVoltage(voltage);
        // Always set voltage for PID and FF
    }

    /**
     * Set the target position for the elevator
     *
     * @param position The target position in meters
     */
    public void setTargetPosition(double position) {
        position =
                MathUtil.clamp(
                        position, ElevatorConstants.ELEVATOR_MIN, ElevatorConstants.ELEVATOR_MAX);
        elevatorPID.setGoal(position);
        double pidCalc = elevatorPID.calculate(getElevatorPosition());
        double ffCalc = elevatorFF.calculate(0.0);

        if (Math.abs(this.getLeftEncoderPosition() - this.getRightEncoderPosition())
                > ConfigManager.getInstance().get("max_roation_diff", 1)) {
            setVoltage(0.0);
            return;
        }
        setVoltage(pidCalc + ffCalc);
    }

    public void holdPosition() {
        double ffCalc = elevatorFF.calculate(0.0);
        setVoltage(ffCalc);
    }

    public void zeroElevator() {
        if (getBottomLinebreak()) {
            setVoltage(0.0);
        } else {
            setVoltage(zeroVoltage);
        }
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    /** Reset elevator PID */
    public void resetPID() {
        elevatorPID.reset(getElevatorPosition());
    }

    /**
     * Get the elevator position
     *
     * @return The elevator position in meters
     */
    public double getElevatorPosition() {
        double encoderAveragePos = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
        // Calculates average pos
        //        double encoderAveragePos = leftEncoder.getPosition();
        return encoderAveragePos * ElevatorConstants.ROTATIONS_TO_METERS;
    }

    /**
     * Get the velocity of the elevator
     *
     * @return The velocity of the elevator in m/s
     */
    public double getElevatorVelocity() {
        double encoderAverageVel = (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
        // Calculates average pos
        return encoderAverageVel * ElevatorConstants.ROTATIONS_TO_METERS;
    }

    public boolean getBottomLinebreak() {
        return !bottomLineBreak.get();
    }

    /**
     * Check if elevator is at target position
     *
     * @return If the elevator is at the correct position
     */
    public boolean isAtPosition() {
        return elevatorPID.atSetpoint();
    }

    @Override
    public void periodic() {

        elevatorPID.setP(
                ConfigManager.getInstance().get("elevator_p", ElevatorConstants.ELEVATOR_P));
        elevatorPID.setI(
                ConfigManager.getInstance().get("elevator_i", ElevatorConstants.ELEVATOR_I));
        elevatorPID.setD(
                ConfigManager.getInstance().get("elevator_d", ElevatorConstants.ELEVATOR_D));

        elevatorLEncoderPos.set(leftEncoder.getPosition());
        elevatorREncoderPos.set(rightEncoder.getPosition());

        elevatorEncoderPos.set(getElevatorPosition());
        zeroVoltage =
                ConfigManager.getInstance()
                        .get("elevator_zero_voltage", ElevatorConstants.ELEVATOR_ZEROING_VOLTAGE);

        elevatorCurrent.set(rightElevatorMotor.getOutputCurrent());
        elevatorVoltage.set(rightElevatorMotor.getAppliedOutput());

        //
        // Elevator zeroing
    }
}
// TODO Add elevator timeout, add boolean position,
