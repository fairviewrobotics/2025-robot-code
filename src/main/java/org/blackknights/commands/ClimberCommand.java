/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.blackknights.subsystems.ClimberSubsystem;

/** Climber command to control the climber */
public class ClimberCommand extends Command {
    public ClimberSubsystem climberSubsystem;
    public CommandXboxController controller;

    /**
     * Command to controller the climber, right now over pure voltage
     *
     * @param climberSubsystem The instance of {@link ClimberSubsystem}
     * @param controller A {@link CommandXboxController} to control the climber
     */
    public ClimberCommand(ClimberSubsystem climberSubsystem, CommandXboxController controller) {
        this.climberSubsystem = climberSubsystem;
        this.controller = controller;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (!MathUtil.isNear(controller.getLeftY(), 0.0, 0.1)) {
            climberSubsystem.setClimberSpeed(controller.getLeftY());
        } else {
            climberSubsystem.setClimberSpeed(0);
        }

        if (controller.povLeft().getAsBoolean()) {
            climberSubsystem.setLockSpeed(0.5);
        } else if (controller.povRight().getAsBoolean()) {
            climberSubsystem.setLockSpeed(-0.5);
        } else {
            climberSubsystem.setLockSpeed(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.setClimberSpeed(0);
        climberSubsystem.setLockSpeed(0);
    }
}
