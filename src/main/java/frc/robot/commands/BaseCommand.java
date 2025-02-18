/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class BaseCommand extends Command {
    public ElevatorSubsystem elevatorSubsystem;
    public ArmSubsystem armSubsystem;

    public BaseCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(elevatorSubsystem, armSubsystem);
    }

    public void initialize() {
        elevatorSubsystem.resetPID();
        armSubsystem.resetPID();
    }

    @Override
    public void execute() {
        armSubsystem.setPivotAngle(-0.1);
        if (armSubsystem.getPivotAngle() <= -Math.PI / 4 || armSubsystem.getPivotAngle() >= 0.2) {
            elevatorSubsystem.holdPosition();
        } else {
            elevatorSubsystem.zeroElevator();
        }
    }
}
