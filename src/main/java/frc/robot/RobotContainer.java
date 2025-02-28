/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.AlignCommand;
import frc.robot.constants.ScoringConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.framework.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import java.util.function.Supplier;

public class RobotContainer {
    GenericHID buttonBoard = new GenericHID(2);

    // Subsystems
    SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    CoralQueue coralQueue = CoralQueue.getInstance();
    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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

    public RobotContainer() {
        configureBindings();
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
                        false));

        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        primaryController.dpadLeft.whileTrue(this.getPlaceCommand(coralQueue::getNext));

        primaryController.aButton.whileTrue(
                this.getPlaceCommand(() -> CoralQueue.getCoralPosition("2L2")));
        primaryController.bButton.whileTrue(
                this.getPlaceCommand(() -> CoralQueue.getCoralPosition("3L2")));

        primaryController.xButton.whileTrue(
                this.getPlaceCommand(() -> CoralQueue.getCoralPosition("6L2")));
        primaryController.yButton.whileTrue(
                this.getPlaceCommand(() -> CoralQueue.getCoralPosition("7L2")));

        primaryController.dpadRight.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        () -> ScoringConstants.ScoringHeights.INTAKE));

        primaryController.dpadDown.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

        secondaryController.dpadRight.onTrue(
                new InstantCommand(() -> coralQueue.loadQueueFromNT()));

        secondaryController.rightBumper.onTrue(new InstantCommand(() -> coralQueue.stepForwards()));
        secondaryController.leftBumper.onTrue(new InstantCommand(() -> coralQueue.stepBackwards()));

        secondaryController.dpadLeft.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        primaryController,
                        () -> ScoringConstants.ScoringHeights.L2));
    }

    public void robotInit() {
        odometry.addCamera(leftCam);
        odometry.addCamera(rightCam);
    }

    public void robotPeriodic() {
        odometry.periodic();
        coralQueue.periodic();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private Command getPlaceCommand(Supplier<CoralQueue.CoralPosition> positionSupplier) {
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                        new AlignCommand(
                                swerveSubsystem,
                                primaryController,
                                () ->
                                        AlignUtils.getXDistBack(
                                                positionSupplier.get().getPose(),
                                                ConfigManager.getInstance()
                                                        .get("align_dist_back", 0.5)),
                                false,
                                "rough"),
                        new BaseCommand(elevatorSubsystem, armSubsystem)),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AlignCommand(
                                        swerveSubsystem,
                                        primaryController,
                                        () -> positionSupplier.get().getPose(),
                                        true,
                                        "fine"),
                                new IntakeCommand(
                                        intakeSubsystem,
                                        IntakeCommand.IntakeMode.OUTTAKE,
                                        () -> armSubsystem.hasGamePiece())),
                        new ElevatorArmCommand(
                                elevatorSubsystem,
                                armSubsystem,
                                primaryController,
                                () -> positionSupplier.get().getHeight())));
    }
}
