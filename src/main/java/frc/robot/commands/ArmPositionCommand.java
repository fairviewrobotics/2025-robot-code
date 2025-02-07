/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.Controller;

public class ArmPositionCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private double position;
    private Controller controller;

    public ArmPositionCommand(ArmSubsystem armSubsystem, double position, Controller controller) {
        this.armSubsystem = armSubsystem;
        this.position = position;
        this.controller = controller;
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
        if (controller.getRightBumperButton()) {
            this.armSubsystem.setIntakeSpeed(ConfigManager.getInstance().get("intake_speed", 0.2));
        } else {
            this.armSubsystem.setIntakeSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setPivotVoltage(0);
        this.armSubsystem.setIntakeVoltage(0);
    }
}
