/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                getPlaceCommand(coralQueue::getNext)
        );

        primaryController.leftBumper.whileTrue(
                new ElevatorArmCommand(
                        elevatorSubsystem,
                        armSubsystem,
                        () -> ScoringConstants.ScoringHeights.INTAKE
                )
        );

        elevatorSubsystem.setDefaultCommand(new BaseCommand(elevatorSubsystem, armSubsystem));

        climberSubsystem.setDefaultCommand(
                new ClimberCommand(climberSubsystem, secondaryController));

        primaryController.dpadDown.whileTrue(new RunCommand(() -> swerveSubsystem.zeroGyro()));

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
                                () -> positionSupplier.get().getHeight())));
    }
}
