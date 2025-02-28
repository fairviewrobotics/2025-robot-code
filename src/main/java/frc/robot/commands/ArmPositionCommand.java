/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/** Command to position the arm */
public class ArmPositionCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final double position;

    /**
     * Set the position of the arm/hand
     *
     * @param armSubsystem The instance of {@link ArmSubsystem}
     * @param position The target position in radians (0 is straight flat)
     */
    public ArmPositionCommand(ArmSubsystem armSubsystem, double position) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        this.armSubsystem.resetPID();
        addRequirements(armSubsystem);
    }

    @Override
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
