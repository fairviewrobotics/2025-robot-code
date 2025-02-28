/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ScoringConstants;
import frc.robot.framework.CoralQueue;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class ButtonBoardSubsystem extends SubsystemBase {
    private final CoralQueue coralQueue = CoralQueue.getInstance();
    private final EventLoop loop = new EventLoop();
    private static final Logger LOGGER = LogManager.getLogger();

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
           ** 16
                           ** 12  ** 1
           ** 15
                     ** 11              ** 2
                     ** 10               ** 3
           ** 14
                     ** 9               ** 4
                     ** 8               ** 5
           ** 13
                            ** 7   ** 6
        */
        for (int b = 1; b < 17; b++) {
            int finalB = b;
            loop.bind(
                    () -> {
                        if (hidDevice.getRawButton(finalB)) {
                            if (finalB > 12) {
                                this.currentHeight =
                                        ScoringConstants.ScoringHeights.valueOf(
                                                String.format("L%d", finalB - 12));
                            } else {
                                int arrId = finalB + (isBlue ? 12 : 0) - 1;
                                this.currentPose = ScoringConstants.CORAL_POSITIONS[arrId];
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
