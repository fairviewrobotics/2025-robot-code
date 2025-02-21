/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.framework.CoralQueue;

public class ButtonBoardSubsystem extends SubsystemBase {
    private final GenericHID hidDevice;

    private CoralQueue coralQueue = CoralQueue.getInstance();

    /**
     * Create a new Instance of the subsystem for the button board
     *
     * @param hidDevice A {@link GenericHID} corresponding to the button board
     */
    public ButtonBoardSubsystem(GenericHID hidDevice) {
        this.hidDevice = hidDevice;
    }

    @Override
    public void periodic() {
        // TODO: Watch for events on every button, then call this.coralQueue.interruptQueue() with
        // the corresponding pose
    }
}
