/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.blackknights.subsystems.ArmSubsystem;
import org.blackknights.subsystems.ElevatorSubsystem;
import org.blackknights.utils.ConfigManager;

/** Default command to keep the elevator and arm at rest */
public class BaseCommand extends Command {
    public ElevatorSubsystem elevatorSubsystem;
    public ArmSubsystem armSubsystem;

    /**
     * Create a new instance of base command
     *
     * @param elevatorSubsystem The instance of {@link ElevatorSubsystem}
     * @param armSubsystem The instance of {@link ArmSubsystem}
     */
    public BaseCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(elevatorSubsystem, armSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.resetPID();
        armSubsystem.resetPID();
    }

    @Override
    public void execute() {
        armSubsystem.setPivotAngle(ConfigManager.getInstance().get("arm_base_angle", 0.1));
        if (armSubsystem.getPivotAngle() <= -Math.PI / 4
                || armSubsystem.getPivotAngle()
                        >= ConfigManager.getInstance().get("arm_movement_max", 0.2)) {
            elevatorSubsystem.holdPosition();
        } else {
            elevatorSubsystem.zeroElevator();
        }
    }
}
