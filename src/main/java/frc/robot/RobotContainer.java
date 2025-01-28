/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.commands.PickAndPlaceCommands.Position;
import frc.robot.constants.*;
import frc.robot.framework.Odometry;
import frc.robot.subsystems.*;
import frc.robot.utils.Camera;
import frc.robot.utils.Controller;
import frc.robot.utils.NetworkTablesUtils;

public class RobotContainer {
    // Subsystems
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();

    // Controllers
    Controller primaryController = new Controller(0);
    Controller secondaryController = new Controller(1);

    private final NetworkTablesUtils NTTune = NetworkTablesUtils.getTable("debug");
    private final Transform3d cameraOffset =
            new Transform3d(0.38, 0, 0.13, new Rotation3d(0.0, Math.toRadians(35), 0.0));
    private final Camera testCamera =
            new Camera("testCam", Camera.CameraType.PHOTONVISION, cameraOffset);

    private Odometry odometry = Odometry.getInstance();
    // Auto Chooser
    SendableChooser<Command> superSecretMissileTech = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // PRIMARY CONTROLLER

        // Default drive command
        swerveSubsystem.setDefaultCommand(
                new DriveCommands(
                        swerveSubsystem,
                        () ->
                                primaryController.getLeftY()
                                        * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                        () ->
                                primaryController.getLeftX()
                                        * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                        () -> primaryController.getRightX() * DrivetrainConstants.MAX_ANGULAR_SPEED,
                        true,
                        true));

        elevatorSubsystem.setDefaultCommand(
            new PickAndPlaceCommands(elevatorSubsystem, armSubsystem, Position.INTAKE)
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
