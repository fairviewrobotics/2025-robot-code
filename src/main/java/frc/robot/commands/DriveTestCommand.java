/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.ConfigManager;

public class DriveTestCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    public DriveTestCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(
                ConfigManager.getInstance().get("test_drive_x", 0.0),
                ConfigManager.getInstance().get("test_drive_y", 0.0),
                Math.toRadians(ConfigManager.getInstance().get("test_drive_rot", 0.0)),
                true,
                false,
                false);
    }

    @Override
    public void end(boolean i) {
        swerveSubsystem.drive(0.0, 0.0, 0.0, true, false, false);
    }
}
