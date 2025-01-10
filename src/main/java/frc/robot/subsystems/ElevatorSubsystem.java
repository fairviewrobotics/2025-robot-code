package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase {



    //TODO Elevator move to position function

    //Two neovortexes
    private final SparkFlex leftElevatorMotor = new SparkFlex(ElevatorConstants.LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex rightElevatorMotor = new SparkFlex(ElevatorConstants.RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    //Relative Encoders
    private final RelativeEncoder leftEncoder = leftElevatorMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightElevatorMotor.getEncoder();

    //Linebreaks
    private final DigitalInput topLineBreak = new DigitalInput(ElevatorConstants.TOP_LINEBREAK_ID);
    private final DigitalInput bottomLineBreak = new DigitalInput(ElevatorConstants.BOTTOM_LINEBREAK_ID);

    private boolean elevatorZeroed = false;
    //Becomes true when we boot up robot and enable.

    private final ProfiledPIDController elevatorPID = new ProfiledPIDController(
            ElevatorConstants.ELEVATOR_P,
            ElevatorConstants.ELEVATOR_I,
            ElevatorConstants.ELEVATOR_D,
            ElevatorConstants.CONSTRAINTS
    );

    private final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(
            ElevatorConstants.ELEVATOR_KS,
            ElevatorConstants.ELEVATOR_KG,
            ElevatorConstants.ELEVATOR_KV,
            ElevatorConstants.ELEVATOR_KA
    );

    public ElevatorSubsystem() {
        SparkFlexConfig rightElevatorMotorConfig = new SparkFlexConfig();
        SparkFlexConfig leftElevatorMotorConfig = new SparkFlexConfig();

        rightElevatorMotorConfig.inverted(true);

        rightElevatorMotor.configure(rightElevatorMotorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        leftElevatorMotor.configure(leftElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }
    public void moveElevator(double speed) {
        leftElevatorMotor.set(speed);
        rightElevatorMotor.set(speed);
    }
    //TODO We need to LEFT_ELEVATOR_ID do something for L1, L2, L3
    //NO absolute encoders
    public void setVoltage(double voltage) {
        leftElevatorMotor.setVoltage(voltage);
        rightElevatorMotor.setVoltage(voltage);
        //Always set voltage for PID and FF
    }

    public void setPosition(double position) {

        double pidCalc = elevatorPID.calculate(getElevatorPosition(), position);
        double ffCalc = elevatorFF.calculate(getElevatorPosition());
        setVoltage(pidCalc+ffCalc);
        //Combines calculations for voltage calculation.
    }
    public void resetPID() {
        elevatorPID.reset(getElevatorPosition());
    }

    public double getElevatorPosition() {
        double encoderAveragePos = (leftEncoder.getPosition() + rightEncoder.getPosition())/2;
        //Calculates average pos
        return encoderAveragePos * ElevatorConstants.ROTATIONS_TO_METERS;
    }
    public double getElevatorVelocity() {
        double encoderAverageVel = (leftEncoder.getVelocity() + rightEncoder.getVelocity())/2;
        //Calculates average pos
        return encoderAverageVel * ElevatorConstants.ROTATIONS_TO_METERS;
    }


    @Override
    public void periodic(){

        if(!elevatorZeroed) {
            setVoltage(ElevatorConstants.ELEVATOR_ZEROING_VOLTAGE);
        }
        if(bottomLineBreak.get()) {

            if(!elevatorZeroed) {

                leftEncoder.setPosition(0);
                rightEncoder.setPosition(0);
                elevatorZeroed = true;

                setVoltage(0);

            }
        }
        //Elevator zeroing
    }


}
