/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.*;
import frc.robot.constants.DrivetrainConstants;
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

    private final Camera leftCam =
            new Camera(
                    "leftCam", Camera.CameraType.PHOTONVISION, VisionConstants.LOW_CAM_TRANSFORM);

    private final Camera centerCam =
            new Camera(
                    "centerCam",
                    Camera.CameraType.PHOTONVISION,
                    VisionConstants.CENTER_CAM_TRANSFORM);

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
                        false));

        primaryController.xButton.whileTrue(new DriveTestCommand(swerveSubsystem));
        primaryController.yButton.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));
        primaryController.aButton.whileTrue(new ReefAlignCommand(swerveSubsystem));

        primaryController.rightBumper.whileTrue(
                new RunCommand(() -> swerveSubsystem.reconfigure(), swerveSubsystem));
        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        primaryController.aButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        "arm_intake",
                        "elevator_intake"));

        primaryController.yButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        "arm_l3",
                        "elevator_l3"));

        primaryController.xButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        "arm_l2",
                        "elevator_l2"));

        primaryController.bButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        "arm_l1",
                        "elevator_l1"));

        //        primaryController.bButton.whileTrue(new ArmPositionCommand(armSubsystem, -Math.PI
        // / 4));
        //        primaryController.xButton.whileTrue(new ArmPositionCommand(armSubsystem, Math.PI /
        // 4));
        //        primaryController.aButton.whileTrue(new ElevatorPositionCommand(elevatorSubsystem,
        // 0.4));

        //        primaryController.xButton.whileTrue(new )
    }

    public void robotInit() {
        odometry.addCamera(leftCam);
        odometry.addCamera(centerCam);
    }

    public void robotPeriodic() {
        odometry.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
