/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ScoringConstants;
import frc.robot.framework.CoralQueue;

public class ButtonBoardSubsystem extends SubsystemBase {
    private final CoralQueue coralQueue = CoralQueue.getInstance();
    private final EventLoop loop = new EventLoop();

    private Pose2d currentPose = null;
    private ScoringConstants.ScoringHeights currentHeight = null;

    /**
     * Create a new Instance of the subsystem for the button board
     *
     * @param hidDevice A {@link GenericHID} corresponding to the button board
     */
    public ButtonBoardSubsystem(GenericHID hidDevice) {
        boolean isBlue =
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        /*
           Assuming layout looks like this not sure
           ** 15
                           ** 11  ** 0
           ** 14
                     ** 10              ** 1
                     ** 9               ** 2
           ** 13
                     ** 8               ** 3
                     ** 7               ** 4
           ** 12
                            ** 6   ** 5
        */
        for (int b = 0; b < 16; b++) {
            BooleanEvent event = hidDevice.button(b, this.loop);

            int finalB = b;
            loop.bind(
                    () -> {
                        if (hidDevice.getRawButton(finalB)) {
                            if (finalB > 11) {
                                this.currentHeight =
                                        ScoringConstants.ScoringHeights.valueOf(
                                                String.format("L%d", finalB - 11));
                            } else {
                                this.currentPose =
                                        ScoringConstants.CORAL_POSITIONS[
                                                finalB + (isBlue ? 12 : 0)];
                            }
                        }
                    });
        }
    }

    @Override
    public void periodic() {
        loop.poll();

        // Check if both a height and pose button has been pressed, if so, interrupt the queue then
        // reset currentPose and currentHeight
        if (this.currentHeight != null && this.currentPose != null) {
            coralQueue.interruptQueue(
                    new CoralQueue.CoralPosition(
                            "BBInterrupt", this.currentPose, this.currentHeight));
            this.currentPose = null;
            this.currentHeight = null;
        }
    }

    /**
     * Get the current pose
     *
     * @return The current pose from button inputs
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /**
     * The current height
     *
     * @return The current height from button inputs
     */
    public ScoringConstants.ScoringHeights getCurrentHeight() {
        return currentHeight;
    }
}
