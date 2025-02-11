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

    public void listToQueue(String posStrList) {
        String[] posList = posStrList.split(",");
        for (String i : posList) {
            addToQueue(i);
        }
    }

    public CoralPosition getNext() {
        currentPos = queue.remove(0);
        return this.currentPos;
    }

    public void addToQueue(String posStr) {
        if (posStr == null || posStr.isEmpty()) {
            return;
        }

        int splitIdx = posStr.length() - 2;
        String heightString = posStr.substring(splitIdx);
        String posString = posStr.substring(0, splitIdx);

        char heightIdxChar = heightString.charAt(1);

        String posIdxString = posString.substring(1, posString.length());



        int heightIdx = Character.getNumericValue(heightIdxChar) - 1;
        int posIdx = Integer.parseInt(posIdxString);

        if (posString.charAt(0) == 'B') {
            posIdx = posIdx + 12;
        }

        queue.add(new CoralPosition(
                new Pose2d(
                        CoralQueueConstants.CORAL_POSITIONS[posIdx].getX(),
                        CoralQueueConstants.CORAL_POSITIONS[posIdx].getY(),
                        CoralQueueConstants.CORAL_POSITIONS[posIdx].getRotation()),
                CoralQueueConstants.REEF_HEIGHTS[heightIdx]));

        posIdx = 0;
        heightIdx = 0;
    }

    public void skipNextValue() {
        queue.add(queue.get(0));
        queue.remove(0);
    }

    public void addToFront() {
        queue.add(0, currentPos);
    }

    public void addToBack() {
        queue.add(currentPos);
    }

    public void clearQueue() {
        queue.clear();
    }

    public void NTaddToQueue() {
        listToQueue(ConfigManager.getInstance().get("Coral_queue", ""));
    }

    /**
     * @param height TODO: Replace with enum once implemented
     */
    public record CoralPosition(Pose2d pose2d, double height) { }
}
