/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

/** Command to intake and outtake */
public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeMode mode;
    private final BooleanSupplier hasGamePiece;

    /**
     * Create a new intake command
     *
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param mode The intake mode ({@link IntakeMode})
     * @param hasGamePiece A {@link BooleanSupplier} supplying if we currently have a coral
     */
    public IntakeCommand(
            IntakeSubsystem intakeSubsystem, IntakeMode mode, BooleanSupplier hasGamePiece) {
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;
        this.hasGamePiece = hasGamePiece;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        switch (mode) {
            case INTAKE:
                intakeSubsystem.setSpeed(0.5);
            case OUTTAKE:
                intakeSubsystem.setSpeed(-0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return mode.equals(IntakeMode.INTAKE) && hasGamePiece.getAsBoolean();
    }

    /** Enum of the different intake modes */
    public enum IntakeMode {
        INTAKE,
        OUTTAKE
    }
}
