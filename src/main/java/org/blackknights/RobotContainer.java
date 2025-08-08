/* Black Knights Robotics (C) 2025 */
package org.blackknights;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    CommandXboxController primaryController = new CommandXboxController(0);
    CommandXboxController secondaryController = new CommandXboxController(1);

    private final NetworkTablesUtils NTTune = NetworkTablesUtils.getTable("debug");

    private final Camera leftCam =
            new Camera(
                    "leftCam", Camera.CameraType.PHOTONVISION, VisionConstants.LEFT_CAM_TRANSFORM);

    private final Camera rightCam =
            new Camera(
                    "rightCam",
                    Camera.CameraType.PHOTONVISION,
                    VisionConstants.RIGHT_CAM_TRANSFORM);

    private final Camera centerCam =
            new Camera(
                    "centerCam",
                    Camera.CameraType.PHOTONVISION,
                    VisionConstants.CENTER_CAM_TRANSFORM);

    private final Odometry odometry = Odometry.getInstance();
    // Auto Chooser
    SendableChooser<Supplier<Command>> superSecretMissileTech = new SendableChooser<>();

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

        // Autos
        superSecretMissileTech.addOption(
                "LEFT_3",
                () ->
                        new SequentialCommandGroup(
                                getLocationPlaceCommand(
                                        CoralQueue.CoralPosition.fromString("11L4")),
                                getAutoIntakeCommand(IntakeSides.LEFT),
                                getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("9L4")),
                                getAutoIntakeCommand(IntakeSides.LEFT),
                                getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("8L4")),
                                getAutoIntakeCommand(IntakeSides.LEFT),
                                getLocationPlaceCommand(
                                        CoralQueue.CoralPosition.fromString("7L4"))));

        superSecretMissileTech.addOption(
                "RIGHT_3",
                () ->
                        new SequentialCommandGroup(
                                getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("3L4")),
                                getAutoIntakeCommand(IntakeSides.RIGHT),
                                getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("5L4")),
                                getAutoIntakeCommand(IntakeSides.RIGHT),
                                getLocationPlaceCommand(
                                        CoralQueue.CoralPosition.fromString("4L4"))));

        superSecretMissileTech.addOption(
                "CENTER_LEFT",
                () -> getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("12L4")));
        superSecretMissileTech.addOption(
                "CENTER_RIGHT",
                () -> getLocationPlaceCommand(CoralQueue.CoralPosition.fromString("1L4")));
    }

    /** Configure controller bindings */
    private void configureBindings() {
        // PRIMARY CONTROLLER
        // Coral Queue: .onTrue(InstantCommand)
        // Default drive command
        swerveSubsystem.setDefaultCommand(
                new DriveCommands(
                        swerveSubsystem,
                        () ->
                                primaryController.getLeftY()
                                        * ConfigManager.getInstance().get("demo_max_speed", 1.0),
                        () ->
                                primaryController.getLeftX()
                                        * ConfigManager.getInstance().get("demo_max_speed", 1.0),
                        () ->
                                -primaryController.getRightX()
                                        * Math.toRadians(
                                                ConfigManager.getInstance()
                                                        .get("demo_max_speed_rot", 90)),
                        true,
                        true));

        //        primaryController
        //                .leftBumper()
        //                .whileTrue(
        //                        getPlaceCommand(
        //                                () -> coralQueue.getCurrentPosition(), () ->
        // coralQueue.getNext()));

        primaryController
                .rightBumper()
                .whileTrue(
                        new ParallelRaceGroup(
                                new DriveCommands(
                                        swerveSubsystem,
                                        () -> primaryController.getLeftY() / 2.0,
                                        () -> primaryController.getLeftX() / 2.0,
                                        () -> -primaryController.getRightX() * Math.PI / 2,
                                        true,
                                        true),
                                new ElevatorArmCommand(
                                        elevatorSubsystem,
                                        armSubsystem,
                                        () -> ScoringConstants.ScoringHeights.INTAKE),
                                new IntakeCommand(
                                        intakeSubsystem, IntakeCommand.IntakeMode.INTAKE)));

        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        primaryController.povDown().whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

        //        primaryController
        //                .a()
        //                .whileTrue(
        //                        new RunCommand(
        //                                () ->
        //                                        elevatorSubsystem.setVoltage(
        //                                                ConfigManager.getInstance()
        //                                                        .get("elevator_manual_zero",
        // -2.0)),
        //                                elevatorSubsystem));
        //
        //        primaryController
        //                .x()
        //                .whileTrue(new InstantCommand(() -> elevatorSubsystem.resetEncoders()));

        //        primaryController
        //                .povUp()
        //                .whileTrue(
        //                        new InstantCommand(
        //                                () ->
        //                                        elevatorSubsystem.setRightEncoder(
        //                                                ConfigManager.getInstance()
        //                                                        .get("break_right_encoder_pos",
        // -10.0))));

        //        secondaryController
        //                .rightStick()
        //                .onTrue(new InstantCommand(ScoringConstants::recomputeCoralPositions));

        // SECONDARY CONTROLLER

        //        climberSubsystem.setDefaultCommand(
        //                new ClimberCommand(climberSubsystem, secondaryController));

        secondaryController
                .a()
                .whileTrue(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> ScoringConstants.ScoringHeights.L1));

        secondaryController
                .b()
                .whileTrue(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> ScoringConstants.ScoringHeights.L2));

        secondaryController
                .x()
                .whileTrue(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> ScoringConstants.ScoringHeights.L3));

        secondaryController
                .y()
                .whileTrue(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> ScoringConstants.ScoringHeights.L4));

        //        secondaryController
        //                .leftBumper()
        //                .onTrue(new InstantCommand(() -> coralQueue.stepForwards())); //
        //
        //        secondaryController
        //                .rightBumper()
        //                .onTrue(new InstantCommand(() -> coralQueue.stepBackwards()));

        secondaryController
                .leftBumper()
                .whileTrue(
                        getPlaceCommand(
                                () -> coralQueue.getCurrentPosition(), () -> coralQueue.getNext()));

        secondaryController
                .rightBumper()
                .onTrue(
                        new DriveCommands(
                                swerveSubsystem,
                                () ->
                                        secondaryController.getLeftY()
                                                * ConfigManager.getInstance()
                                                        .get("driver_max_speed", 3.5),
                                () ->
                                        secondaryController.getLeftX()
                                                * ConfigManager.getInstance()
                                                        .get("driver_max_speed", 3.5),
                                () ->
                                        -secondaryController.getRightX()
                                                * Math.toRadians(
                                                        ConfigManager.getInstance()
                                                                .get("driver_max_speed_rot", 360)),
                                true,
                                true));

        secondaryController
                .rightTrigger(0.2)
                .whileTrue(new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeMode.OUTTAKE));

        secondaryController
                .leftTrigger(0.2)
                .whileTrue(new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeMode.INTAKE));
    }

    /** Runs once when the code starts */
    public void robotInit() {
        odometry.addCamera(leftCam);
        odometry.addCamera(rightCam);
        odometry.addCamera(centerCam);
    }

    /** Runs every 20ms while the robot is on */
    public void robotPeriodic() {
        odometry.periodic();
        coralQueue.periodic();
    }

    /** Runs ones when enabled in teleop */
    public void teleopInit() {
        buttonBoardSubsystem.bind();

        if (cqProfiles.getSelected() != null)
            CoralQueue.getInstance().loadProfile(cqProfiles.getSelected());
    }

    /**
     * Get the command to run in auto mode
     *
     * @return The command to run
     */
    public Supplier<Command> getAutonomousCommand() {
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
                                true,
                                "rough"),
                        new BaseCommand(elevatorSubsystem, armSubsystem)),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        () -> {
                                            if (currentSupplier.get().getSide()
                                                    == ScoringConstants.ScoringSides.LEFT) {
                                                rightCam.setEnabled(true);
                                                leftCam.setEnabled(false);
                                            } else {
                                                leftCam.setEnabled(true);
                                                rightCam.setEnabled(false);
                                            }
                                        }),
                                new AlignCommand(
                                                swerveSubsystem,
                                                () -> currentSupplier.get().getPose(),
                                                true,
                                                false,
                                                "fine")
                                        .withTimeout(
                                                ConfigManager.getInstance()
                                                        .get("align_fine_max_time", 3.0)),
                                new InstantCommand(
                                        () -> {
                                            rightCam.setEnabled(true);
                                            leftCam.setEnabled(true);
                                        })),
                        new RunCommand(
                                () ->
                                        intakeSubsystem.setSpeed(
                                                ConfigManager.getInstance()
                                                        .get("intake_slow_voltage", -2.0)),
                                intakeSubsystem),
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> currentSupplier.get().getHeight())),
                new ParallelRaceGroup(
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                () -> nextSupplier.get().getHeight()),
                        new IntakeCommand(
                                        intakeSubsystem,
                                        IntakeCommand.IntakeMode.OUTTAKE,
                                        elevatorSubsystem.isAtTargetSupplier())
                                .withTimeout(
                                        ConfigManager.getInstance()
                                                .get("outtake_max_time_sec", 5.0))),
                new ParallelRaceGroup(
                        new AutoEndCommand(),
                        new BaseCommand(elevatorSubsystem, armSubsystem),
                        new RunCommand(
                                        () ->
                                                swerveSubsystem.drive(
                                                        ConfigManager.getInstance()
                                                                .get("back_mps", -1.0),
                                                        0.0,
                                                        0.0,
                                                        false,
                                                        false,
                                                        false),
                                        swerveSubsystem)
                                .withTimeout(
                                        ConfigManager.getInstance().get("back_time_sec", 0.2))));
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
    private Command getAutoIntakeCommand(IntakeSides side) {
        Pose2d intakePose = getPose2d(side);

        Pose2d intakePoseFinal =
                intakePose.plus(new Transform2d(0, 0, Rotation2d.fromRadians(Math.PI)));

        return new ParallelRaceGroup(
                        new IntakeCommand(intakeSubsystem, IntakeCommand.IntakeMode.INTAKE),
                        new ParallelCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        () -> intakePoseFinal,
                                        true,
                                        false,
                                        "rough"),
                                new ElevatorArmCommand(
                                        elevatorSubsystem,
                                        armSubsystem,
                                        () -> ScoringConstants.ScoringHeights.INTAKE)))
                .withTimeout(5);
    }

    private static Pose2d getPose2d(IntakeSides side) {
        Pose2d intakePose;
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            if (side == IntakeSides.LEFT) {
                intakePose = ScoringConstants.INTAKE_BLUE_LEFT;
            } else {
                intakePose = ScoringConstants.INTAKE_BLUE_RIGHT;
            }
        } else {
            if (side == IntakeSides.LEFT) {
                intakePose = ScoringConstants.INTAKE_RED_LEFT;
            } else {
                intakePose = ScoringConstants.INTAKE_RED_RIGHT;
            }
        }
        return intakePose;
    }

    private enum IntakeSides {
        LEFT,
        RIGHT
    }

    private static class AutoEndCommand extends Command {
        private double currTime = Timer.getFPGATimestamp() * 1000;

        @Override
        public void initialize() {
            this.currTime = Timer.getFPGATimestamp() * 1000;
        }

        @Override
        public boolean isFinished() {
            return DriverStation.isAutonomous()
                    && Timer.getFPGATimestamp() * 1000 - this.currTime
                            > ConfigManager.getInstance().get("auto_place_backup_time_ms", 0.2);
        }
    }
}
