/* Black Knights Robotics (C) 2025 */
package org.blackknights;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;
import org.blackknights.commands.*;
import org.blackknights.constants.ScoringConstants;
import org.blackknights.constants.VisionConstants;
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
                        getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("10L4")),
                        getAutoIntakeCommand(),
                        getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("8L4")),
                        getAutoIntakeCommand(),
                        getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("9L4"))));

        superSecretMissileTech.addOption(
                "One pose", getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("10L4")));
    }

    /** Configure controller bindings */
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

        primaryController.dpadDown.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

        // SECONDARY CONTROLLER

        climberSubsystem.setDefaultCommand(
                new ClimberCommand(climberSubsystem, secondaryController));

        //        secondaryController.leftBumper.onTrue(new InstantCommand(() ->
        // coralQueue.stepBackwards()));
        //        secondaryController.rightBumper.onTrue(new InstantCommand(() ->
        // coralQueue.stepForwards()));

        secondaryController.aButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        () -> ScoringConstants.ScoringHeights.INTAKE));

        secondaryController.bButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem, armSubsystem, () -> ScoringConstants.ScoringHeights.L2));

        secondaryController.xButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem, armSubsystem, () -> ScoringConstants.ScoringHeights.L3));

        secondaryController.yButton.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem, armSubsystem, () -> ScoringConstants.ScoringHeights.L4));

        secondaryController.leftBumper.whileTrue(
                new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeMode.INTAKE, () -> false));

        secondaryController.rightBumper.whileTrue(
                new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeMode.OUTTAKE, () -> false));
    }

    /** Runs once when the code starts */
    public void robotInit() {
        odometry.addCamera(leftCam);
        odometry.addCamera(rightCam);
    }

    /** Runs every 20ms while the robot is on */
    public void robotPeriodic() {
        odometry.periodic();
        coralQueue.periodic();
    }

    /** Runs ones when enabled in teleop */
    public void teleopInit() {
        CoralQueue.getInstance().loadProfile(cqProfiles.getSelected());
    }

    /**
     * Get the command to run in auto mode
     *
     * @return The command to run
     */
    public Command getAutonomousCommand() {
        return superSecretMissileTech.getSelected();
    }

    /**
     * Get the full place command. <br>
     * <strong>Steps</strong>
     *
     * <ul>
     *   <li>Go to a position `align_dist_back` (in config manager) meters back from the scoring
     *       position
     *   <li>Raise the elevator the correct height and drive forward to the final position
     *   <li>Score the piece
     * </ul>
     *
     * @param currentSupplier A {@link Supplier} of {@link
     *     org.blackknights.framework.CoralQueue.CoralPosition}s that should not update the current
     *     position
     * @param nextSupplier A {@link Supplier} of {@link
     *     org.blackknights.framework.CoralQueue.CoralPosition}s that should update the current
     *     position
     * @return The full command
     */
    private Command getPlaceCommand(
            Supplier<CoralQueue.CoralPosition> currentSupplier,
            Supplier<CoralQueue.CoralPosition> nextSupplier) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AlignCommand(
                                swerveSubsystem,
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
                                        () -> currentSupplier.get().getPose(),
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
                                () -> nextSupplier.get().getHeight())));
    }

    /**
     * Place at a specific location
     *
     * @param position The target {@link org.blackknights.framework.CoralQueue.CoralPosition}
     * @return The command to place
     */
    private Command getLocationPlaceCommand(CoralQueue.CoralPosition position) {
        return getPlaceCommand(() -> position, () -> position);
    }

    /**
     * Get a command to goto the feeder
     *
     * @return The command to goto the feeder
     */
    private Command getAutoIntakeCommand() {
        return new ParallelDeadlineGroup(
                new SequentialCommandGroup(
                        new AlignCommand(
                                swerveSubsystem,
                                () ->
                                        DriverStation.getAlliance().isPresent()
                                                        && DriverStation.getAlliance().get()
                                                                == DriverStation.Alliance.Blue
                                                ? ScoringConstants.INTAKE_BLUE
                                                : ScoringConstants.INTAKE_RED,
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
