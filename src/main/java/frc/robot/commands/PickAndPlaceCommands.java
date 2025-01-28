/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.ConfigManager;

public class PickAndPlaceCommands extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;

    private final Position position;

    public enum Position {
        INTAKE(
                ConfigManager.getInstance()
                        .get("intake_elevator_position", ElevatorConstants.INTAKE_HEIGHT),
                ConfigManager.getInstance().get("intake_arm_angle", ArmConstants.INTAKE_ANGLE)),
        L1(
                ConfigManager.getInstance()
                        .get("l1_elevator_position", ElevatorConstants.L1_HEIGHT),
                ConfigManager.getInstance().get("place_arm_angle", ArmConstants.PLACE_ANGLE)),
        L2(
                ConfigManager.getInstance()
                        .get("l2_elevator_position", ElevatorConstants.L2_HEIGHT),
                ConfigManager.getInstance().get("place_arm_angle", ArmConstants.PLACE_ANGLE)),
        L3(
                ConfigManager.getInstance()
                        .get("l3_elevator_position", ElevatorConstants.L3_HEIGHT),
                ConfigManager.getInstance().get("place_arm_angle", ArmConstants.PLACE_ANGLE)),
        L4(
                ConfigManager.getInstance()
                        .get("l4_elevator_position", ElevatorConstants.L4_HEIGHT),
                ConfigManager.getInstance().get("place_arm_angle", ArmConstants.PLACE_ANGLE));

        private double height;
        private double angle;

        private Position(double height, double angle) {
            this.height = height;
            this.angle = angle;
        }

        protected double getHeight() {
            return this.height;
        }

        protected double getAngle() {
            return this.angle;
        }
    }

    public PickAndPlaceCommands(
            ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, Position pos) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(elevatorSubsystem);
        addRequirements(armSubsystem);
        this.position = pos;
    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetPosition(position.getHeight());
        armSubsystem.setPivotAngle(position.getAngle());
    }
}
