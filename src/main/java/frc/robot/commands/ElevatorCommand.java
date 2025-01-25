package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private double voltage;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double voltage) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.voltage = voltage;
    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.setVoltage(0);
    }
}
