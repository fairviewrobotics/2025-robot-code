/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import frc.robot.utils.ConfigManager;

public class IntakeCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public IntakeCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    public void initialize() {}

    public void execute() {
        armSubsystem.setIntakeSpeed(ConfigManager.getInstance().get("intake_speed", 0.2));
    }
}
