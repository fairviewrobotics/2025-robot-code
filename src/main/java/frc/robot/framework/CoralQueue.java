/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.CoralQueueConstants;
import frc.robot.utils.ConfigManager;
import java.util.ArrayList;

public class CoralQueue {
    private ArrayList<Pose3d> queue;

    private Pose3d currentPos = new Pose3d();

    public void listToQueue(String posStrList) {
        String[] posList = posStrList.split(",");
        for (String i : posList) {
            addToQueue(i);
        }
    }

    public Pose3d getNext() {
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

        String heightIdxString = posStr.substring(heightString.length() - 1, heightString.length());

        String posIdxString;

        if (posString.length() == 3) {
            posIdxString = posStr.substring(posString.length() - 2, posString.length());
        } else {
            posIdxString = posStr.substring(posString.length() - 1, posString.length());
        }

        int heightIdx = Integer.parseInt(heightIdxString) - 1;
        int posIdx = Integer.parseInt(posIdxString);

        if (posString.charAt(0) == 'B') {
            posIdx = posIdx + 12;
        }

        queue.add(
                new Pose3d(
                        CoralQueueConstants.CORAL_POSITIONS[posIdx].getX(),
                        CoralQueueConstants.CORAL_POSITIONS[posIdx].getY(),
                        CoralQueueConstants.REEF_HEIGHTS[heightIdx],
                        new Rotation3d(CoralQueueConstants.CORAL_POSITIONS[posIdx].getRotation())));
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
}
