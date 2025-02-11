/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.CoralQueueConstants;
import frc.robot.utils.ConfigManager;
import java.util.ArrayList;

public class CoralQueue {
    private ArrayList<CoralPosition> queue = new ArrayList<>();

    private CoralPosition currentPos = null;

    public CoralQueue() {}

    /**
     * Add position string to queue. No spaces, separated by commas. (e.g. B10H2,R1H3,R2H2,B1H1,B5H4).
     *
     * @param posStrList
     */
    public void listToQueue(String posStrList) {
        String[] posList = posStrList.split(",");
        for (String i : posList) {
            addToQueue(i);
        }
    }

    /**
     * Return next reef position in the queue
     *
     * @return
     */
    public CoralPosition getNext() {
        currentPos = queue.remove(0);
        return this.currentPos;
    }

    /**
     * Convert reef position IDs to heights and positions.
     *
     * @param posStr
     */
    public void addToQueue(String posStr) {
        if (posStr == null || posStr.isEmpty()) {
            return;
        }

        int splitIdx = posStr.length() - 2;
        String heightString = posStr.substring(splitIdx);
        String posString = posStr.substring(0, splitIdx);

        char heightIdxChar = heightString.charAt(1);

        String posIdxString = posString.substring(1);

        int heightIdx = Character.getNumericValue(heightIdxChar) - 1;
        int posIdx = Integer.parseInt(posIdxString);

        if (posString.charAt(0) == 'B') {
            posIdx = posIdx + 12;
        }

        queue.add(
                new CoralPosition(
                        new Pose2d(
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getX(),
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getY(),
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getRotation()),
                        CoralQueueConstants.REEF_HEIGHTS[heightIdx]));
    }

    /** Skip to the next reef position in the queue */
    public void skipNextValue() {
        queue.add(queue.get(0));
        queue.remove(0);
    }

    /** Add current reef position to the front of the queue. */
    public void addToFront() {
        queue.add(0, currentPos);
    }

    /** Add current reef position to the back of the queue. */
    public void addToBack() {
        queue.add(currentPos);
    }

    /** Clear the queue. */
    public void clearQueue() {
        queue.clear();
    }

    /**
     * Input position string and add it to queue via network tables. No spaces, separated by commas.
     * (e.g. B10H2,R1H3,R2H2,B1H1,B5104).
     */
    public void loadQueueFromNT() {
        listToQueue(ConfigManager.getInstance().get("coral_queue", ""));
    }

    /**
     * @param height TODO: Replace with enum once implemented
     */
    public record CoralPosition(Pose2d pose2d, double height) {}
}
