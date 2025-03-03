/* Black Knights Robotics (C) 2025 */
package org.blackknights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;
import org.blackknights.commands.*;
import org.blackknights.constants.ScoringConstants;
import org.blackknights.constants.VisionConstants;
import org.blackknights.framework.*;
import org.blackknights.framework.CoralQueue;
import org.blackknights.framework.Odometry;
import org.blackknights.subsystems.*;
import org.blackknights.utils.*;

public class RobotContainer {
    GenericHID buttonBoard = new GenericHID(2);

    // Subsystems
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    CoralQueue coralQueue = CoralQueue.getInstance();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    ButtonBoardSubsystem buttonBoardSubsystem = new ButtonBoardSubsystem(buttonBoard);

    // Controllers
    Controller primaryController = new Controller(0);
    Controller secondaryController = new Controller(1);

    private final NetworkTablesUtils NTTune = NetworkTablesUtils.getTable("debug");

    private final Camera leftCam =
            new Camera(
                    "leftCam", Camera.CameraType.PHOTONVISION, VisionConstants.LEFT_CAM_TRANSFORM);

    private final Camera rightCam =
            new Camera(
                    "rightCam",
                    Camera.CameraType.PHOTONVISION,
                    VisionConstants.RIGHT_CAM_TRANSFORM);

    private final Odometry odometry = Odometry.getInstance();
    // Auto Chooser
    SendableChooser<Command> superSecretMissileTech = new SendableChooser<>();

    // CQ profile selector
    private final SendableChooser<CoralQueue.CoralQueueProfile> cqProfiles =
            new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        for (String key : ScoringConstants.PROFILES.keySet()) {
            cqProfiles.addOption(key, ScoringConstants.PROFILES.get(key));
        }

        SmartDashboard.putData(cqProfiles);
        SmartDashboard.putData("Auto Chooser", superSecretMissileTech);

        superSecretMissileTech.addOption(
                "Left",
                new SequentialCommandGroup(
                        getAutoPlaceCommand("10L4"),
                        getAutoIntakeCommand(),
                        getAutoPlaceCommand("8L4"),
                        getAutoIntakeCommand(),
                        getAutoPlaceCommand("9L4")));

        superSecretMissileTech.addOption("One pose", getAutoPlaceCommand("10L4"));
    }

    private void configureBindings() {
        // PRIMARY CONTROLLER
        // Coral Queue: .onTrue(InstantCommand)
        // Default drive command
        swerveSubsystem.setDefaultCommand(
                new DriveCommands(
                        swerveSubsystem,
                        () -> primaryController.getLeftY() * 2.5,
                        () -> primaryController.getLeftX() * 2.5,
                        () -> -primaryController.getRightX() * Math.PI,
                        true,
                        true));

        primaryController.rightBumper.whileTrue(
                getPlaceCommand(() -> coralQueue.getCurrentPosition(), () -> coralQueue.getNext()));

        primaryController.leftBumper.whileTrue(
                new ParallelCommandGroup(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> ScoringConstants.ScoringHeights.INTAKE),
                        new IntakeCommand(
                                intakeSubsystem, IntakeCommand.IntakeMode.INTAKE, () -> false)));

        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        climberSubsystem.setDefaultCommand(
                new ClimberCommand(climberSubsystem, secondaryController));

        primaryController.dpadDown.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

        secondaryController.leftBumper.onTrue(new InstantCommand(() -> coralQueue.stepBackwards()));
        secondaryController.rightBumper.onTrue(new InstantCommand(() -> coralQueue.stepForwards()));

        secondaryController.aButton.whileTrue(getAutoPlaceCommand("10L4"));

        //        secondaryController.dpadRight.onTrue(
        //                new InstantCommand(() -> coralQueue.loadQueueFromNT()));
        //
        //        secondaryController.rightBumper.onTrue(new InstantCommand(() ->
        // coralQueue.stepForwards()));
        //        secondaryController.leftBumper.onTrue(new InstantCommand(() ->
        // coralQueue.stepBackwards()));
        //
        //        secondaryController.dpadLeft.whileTrue(
        //                new ElevatorArmCommand(
        //                        elevatorSubsystem,
        //                        armSubsystem,
        //                        () -> ScoringConstants.ScoringHeights.L2));
        //    }
    }

    public void robotInit() {
        odometry.addCamera(leftCam);
        odometry.addCamera(rightCam);
    }

    public void robotPeriodic() {
        odometry.periodic();
        coralQueue.periodic();
    }

    public void teleopInit() {
        CoralQueue.getInstance().loadProfile(cqProfiles.getSelected());
    }

    public Command getAutonomousCommand() {
        return superSecretMissileTech.getSelected();
    }

    private Command getPlaceCommand(
            Supplier<CoralQueue.CoralPosition> currentSupplier,
            Supplier<CoralQueue.CoralPosition> nextSupplier) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AlignCommand(
                                swerveSubsystem,
                                //                                primaryController,
                                () ->
                                        AlignUtils.getXDistBack(
                                                currentSupplier.get().getPose(),
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                false,
                                "rough"),
                        new BaseCommand(elevatorSubsystem, armSubsystem)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        //                                        primaryController,
                                        () -> currentSupplier.get().getPose(),
                                        true,
                                        "fine"),
                                new IntakeCommand(
                                        intakeSubsystem,
                                        IntakeCommand.IntakeMode.OUTTAKE,
                                        () -> armSubsystem.hasGamePiece())),
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> nextSupplier.get().getHeight())));
    }

    public Command getAutoPlaceCommand(String location) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AlignCommand(
                                swerveSubsystem,
                                //                                primaryController,
                                () ->
                                        AlignUtils.getXDistBack(
                                                CoralQueue.CoralPosition.fromString(location)
                                                        .getPose(),
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                false,
                                "rough"),
                        new BaseCommand(elevatorSubsystem, armSubsystem)),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        //
                                        () ->
                                                CoralQueue.CoralPosition.fromString(location)
                                                        .getPose(),
                                        true,
                                        "fine"),
                                new IntakeCommand(
                                                intakeSubsystem,
                                                IntakeCommand.IntakeMode.OUTTAKE,
                                                () -> armSubsystem.hasGamePiece())
                                        .withTimeout(2)),
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> CoralQueue.CoralPosition.fromString(location).getHeight())));
    }

    public Command getAutoIntakeCommand() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new AlignCommand(
                                swerveSubsystem,
                                () -> new Pose2d(15.950, 1.395, new Rotation2d(-0.918)),
                                true,
                                "rough"),
                        new IntakeCommand(
                                        intakeSubsystem,
                                        IntakeCommand.IntakeMode.INTAKE,
                                        () -> armSubsystem.hasGamePiece())
                                .withTimeout(2)),
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        () -> ScoringConstants.ScoringHeights.INTAKE));
    }
}
