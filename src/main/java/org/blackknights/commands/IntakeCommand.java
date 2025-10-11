/* Black Knights Robotics (C) 2025 */
package org.blackknights.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import org.blackknights.subsystems.IntakeSubsystem;
import org.blackknights.utils.ConfigManager;
import org.blackknights.utils.NetworkTablesUtils;

/** Command to intake and outtake */
public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final IntakeMode mode;
    private final BooleanSupplier elevatorAtTargetSupplier;

    private double elevatorAtTargetTime;

    /**
     * Create a new intake command
     *
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param mode The intake mode ({@link IntakeMode})
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, IntakeMode mode) {
        this.elevatorAtTargetSupplier = () -> true;
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;
        addRequirements(intakeSubsystem);
    }

    /**
     * Create a new intake command
     *
     * @param intakeSubsystem The instance of {@link IntakeSubsystem}
     * @param mode The intake mode ({@link IntakeMode})
     */
    public IntakeCommand(
            IntakeSubsystem intakeSubsystem,
            IntakeMode mode,
            BooleanSupplier elevatorAtTargetSupplier) {
        this.elevatorAtTargetSupplier = elevatorAtTargetSupplier;
        this.intakeSubsystem = intakeSubsystem;
        this.mode = mode;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.elevatorAtTargetTime = 0;
    }

    @Override
    public void execute() {
        if (this.elevatorAtTargetTime == 0 && elevatorAtTargetSupplier.getAsBoolean()) {
            this.elevatorAtTargetTime = Timer.getFPGATimestamp() * 1000;
        }
        switch (mode) {
            case INTAKE:
                {
                    intakeSubsystem.setVoltage(
                            ConfigManager.getInstance().get("intake_speed", 8.0));
                    break;
                }
            case OUTTAKE:
                {
                    NetworkTablesUtils.getTable("debug/IntakeCmd")
                            .setEntry(
                                    "Time running",
                                    Timer.getFPGATimestamp() * 1000 - this.elevatorAtTargetTime);

                    if (Timer.getFPGATimestamp() * 1000 - this.elevatorAtTargetTime
                                    > ConfigManager.getInstance().get("outtake_wait_time_ms", 250.0)
                            && elevatorAtTargetSupplier.getAsBoolean()) {

                        intakeSubsystem.setVoltage(
                                ConfigManager.getInstance().get("outtake_speed", 8.0));
                    }
                    break;
                }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return (mode.equals(IntakeMode.INTAKE) && intakeSubsystem.getLinebreak())
                || (mode.equals(IntakeMode.OUTTAKE)
                        && !intakeSubsystem.getLinebreak()
                        && Timer.getFPGATimestamp() * 1000 - this.elevatorAtTargetTime
                                > (ConfigManager.getInstance().get("outtaking_time_ms", 200)
                                        + ConfigManager.getInstance()
                                                .get("outtake_wait_time_ms", 250)));
    }

    /** Enum of the different intake modes */
    public enum IntakeMode {
        INTAKE,
        OUTTAKE
    }
}
