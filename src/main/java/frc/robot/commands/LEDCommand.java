/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.AnimationTypes;

public class LEDCommand extends Command {
    private final LEDSubsystem ledSubsystem;
    private final AnimationTypes animationType;

    public LEDCommand(LEDSubsystem ledSubsystem, AnimationTypes animationType) {
        this.ledSubsystem = ledSubsystem;
        this.animationType = animationType;
    }

    @Override
    public void execute() {
        ledSubsystem.setAnimation(animationType);
    }
}
