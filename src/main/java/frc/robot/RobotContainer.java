/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.AlignCommand;
import frc.robot.constants.VisionConstants;
import frc.robot.framework.Odometry;
import frc.robot.subsystems.*;
import frc.robot.utils.*;

public class RobotContainer {
    // Subsystems
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();

    // Controllers
    Controller primaryController = new Controller(0);
    Controller secondaryController = new Controller(1);

    Pose2d targetPose = new Pose2d(12.188, 2.863, Rotation2d.fromRadians(1.052));
    Pose2d intakePose = new Pose2d(12.785, 0.004, Rotation2d.fromRadians(-1.296));

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
                        () -> primaryController.getLeftY() * 2.5,
                        () -> primaryController.getLeftX() * 2.5,
                        () -> -primaryController.getRightX() * Math.PI,
                        true,
                        false));

        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        //        primaryController.aButton.whileTrue(
        //                new SequentialCommandGroup(
        //                        new AlignCommand(
        //                                swerveSubsystem,
        //                                AlignUtils.getFirstPose(
        //                                        targetPose,
        //                                        ConfigManager.getInstance().get("Align/Dist_Back",
        // 0.5)),
        //                                "Rough"),
        //                        new ParallelCommandGroup(
        //                                new AlignCommand(swerveSubsystem, targetPose, "Fine"),
        //                                new InstantCommand() // TODO: Replace with elevator
        // command
        //                                )));

        primaryController.aButton.whileTrue(
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        AlignUtils.getFirstPose(
                                                targetPose,
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                        false,
                                        "rough"),
                                new BaseCommand(elevatorSubsystem, armSubsystem)),
                        new ParallelCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        targetPose,
                                        true,
                                        "fine"),
                                new ElevatorArmCommand(
                                        elevatorSubsystem,
                                        armSubsystem,
                                        primaryController,
                                        "arm_l1",
                                        "elevator_l1"))));

        primaryController.bButton.whileTrue(
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        AlignUtils.getFirstPose(
                                                targetPose,
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                        false,
                                        "rough"),
                                new BaseCommand(elevatorSubsystem, armSubsystem)),
                        new ParallelCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        targetPose,
                                        true,
                                        "fine"),
                                new ElevatorArmCommand(
                                        elevatorSubsystem,
                                        armSubsystem,
                                        primaryController,
                                        "arm_l3",
                                        "elevator_l3"))));

        primaryController.xButton.whileTrue(
                new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        AlignUtils.getFirstPose(
                                                targetPose,
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                        false,
                                        "rough"),
                                new BaseCommand(elevatorSubsystem, armSubsystem)),
                        new ParallelCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        targetPose,
                                        true,
                                        "fine"),
                                new ElevatorArmCommand(
                                        elevatorSubsystem,
                                        armSubsystem,
                                        primaryController,
                                        "arm_l2",
                                        "elevator_l2"))));

        //        primaryController.xButton.whileTrue(new DriveTestCommand(swerveSubsystem));
        primaryController.yButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        "arm_intake",
                        "elevator_intake"));
        primaryController.dpadDown.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

        primaryController.dpadRight.whileTrue(new ElevatorPositionCommand(elevatorSubsystem, 0.4));
        //        primaryController.aButton.whileTrue(new ReefAlignCommand(swerveSubsystem));

        //        primaryController.rightBumper.whileTrue(
        //                new RunCommand(() -> swerveSubsystem.reconfigure(), swerveSubsystem));
        //        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem,
        // armSubsystem));
        //
        //        primaryController.aButton.whileTrue(
        //                new ElevatorArmCommand(
        //                        elevatorSubsystem,
        //                        armSubsystem,
        //                        primaryController,
        //                        "arm_intake",
        //                        "elevator_intake"));
        //
        //        primaryController.yButton.whileTrue(
        //                new ElevatorArmCommand(
        //                        elevatorSubsystem,
        //                        armSubsystem,
        //                        primaryController,
        //                        "arm_l3",
        //                        "elevator_l3"));
        //
        //        primaryController.xButton.whileTrue(
        //                new ElevatorArmCommand(
        //                        elevatorSubsystem,
        //                        armSubsystem,
        //                        primaryController,
        //                        "arm_l2",
        //                        "elevator_l2"));
        //
        //        primaryController.bButton.whileTrue(
        //                new ElevatorArmCommand(
        //                        elevatorSubsystem,
        //                        armSubsystem,
        //                        primaryController,
        //                        "arm_l1",
        //                        "elevator_l1"));

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
