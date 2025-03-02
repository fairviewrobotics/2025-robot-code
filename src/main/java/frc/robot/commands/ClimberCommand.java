/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utils.Controller;

public class ClimberCommand extends Command {
    public ClimberSubsystem climberSubsystem;
    public Controller controller;

    public ClimberCommand(ClimberSubsystem climberSubsystem, Controller controller) {
        this.climberSubsystem = climberSubsystem;
        this.controller = controller;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (controller.dpadDown.getAsBoolean()) {
            climberSubsystem.setClimberSpeed(1);
        } else if (controller.dpadUp.getAsBoolean()) {
            climberSubsystem.setClimberSpeed(-1);
        } else if (controller.dpadLeft.getAsBoolean()) {
            climberSubsystem.setLockSpeed(0.5);
        } else if (controller.dpadRight.getAsBoolean()) {
            climberSubsystem.setLockSpeed(-0.5);
        } else {
            climberSubsystem.setClimberSpeed(0);
            climberSubsystem.setLockSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setClimberSpeed(0);
        climberSubsystem.setLockSpeed(0);
    }
}
