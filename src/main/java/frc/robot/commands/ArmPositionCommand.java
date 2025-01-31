/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPositionCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private double position;

    public ArmPositionCommand(ArmSubsystem armSubsystem, double position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
    }

    public void initialize() {
        armSubsystem.resetPID();
    }

    @Override
    public void execute() {
        this.armSubsystem.setPivotAngle(position);
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setPivotVoltage(0);
    }
}
