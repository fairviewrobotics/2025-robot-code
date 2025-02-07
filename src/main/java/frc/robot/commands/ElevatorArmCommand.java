/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.Controller;

public class ElevatorArmCommand extends Command {
    public ElevatorSubsystem elevatorSubsystem;
    public ArmSubsystem armSubsystem;
    public Controller controller;

    public String armPosKey;
    public String elevatorPosKey;

    private final BooleanEntry isAtHold =
            NetworkTableInstance.getDefault()
                    .getTable("Elevator")
                    .getBooleanTopic("AtHoldPos")
                    .getEntry(true);

    public ElevatorArmCommand(
            ElevatorSubsystem elevatorSubsystem,
            ArmSubsystem armSubsystem,
            Controller controller,
            String armPosKey,
            String elevatorPosKey) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.controller = controller;
        this.armPosKey = armPosKey;
        this.elevatorPosKey = elevatorPosKey;

        addRequirements(elevatorSubsystem, armSubsystem);
    }

    public void initialize() {
        elevatorSubsystem.resetPID();
        armSubsystem.resetPID();
    }

    @Override
    public void execute() {
        double elevatorPos = ConfigManager.getInstance().get(elevatorPosKey, 0.0);
        double armPos = ConfigManager.getInstance().get(armPosKey, 0.0);

        if ((armSubsystem.getPivotAngle() <= -Math.PI / 4 || armSubsystem.getPivotAngle() >= 0)
                && elevatorSubsystem.getElevatorPosition() <= 0.24
                && elevatorPos != 0) {
            isAtHold.set(true);
            armSubsystem.setPivotAngle(0.0);
            elevatorSubsystem.holdPosition();
        } else {
            isAtHold.set(false);
            armSubsystem.setPivotAngle(armPos);
            elevatorSubsystem.setTargetPosition(elevatorPos);
        }
        if (controller.getRightBumperButton()) {
            this.armSubsystem.setIntakeSpeed(ConfigManager.getInstance().get("intake_speed", 0.2));
        } else if (controller.getLeftBumperButton()) {
            this.armSubsystem.setIntakeSpeed(
                    ConfigManager.getInstance().get("outtake_speed", -0.2));
        } else {
            this.armSubsystem.setIntakeSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        //        armSubsystem.setPivotVoltage(0);
        armSubsystem.setIntakeVoltage(0);
        //        elevatorSubsystem.setVoltage(0);
    }
}
