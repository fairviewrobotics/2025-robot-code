/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private double position;

    public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.position = position;
    }

    public void initialize() {
        elevatorSubsystem.resetPID();
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetPosition(position);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.setVoltage(0);
    }
}
