/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.CoralQueueConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;
import java.util.ArrayList;
import java.util.Optional;

public class CoralQueue {

    private ArrayList<CoralPosition> coralPositionList = new ArrayList<>();

    private int positionListIndex = 0;
    private CoralPosition currentPos = new CoralPosition("1H1", new Pose2d(), 0.0, false);
    private Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

    public CoralQueue() {

        loadQueueFromDefault("PROFILE_2");
    }

    /**
     * Add position string to queue. No spaces, separated by commas. (e.g.
     * B10H2,R1H3,R2H2,B1H1,B5H4).
     *
     * @param posStrList
     */
    public void listToQueue(String posStrList) {
        String[] posList = posStrList.split(",");
        for (String i : posList) {
            addToList(i);
        }
    }

    /**
     * Return next reef position in the queue
     *
     * @return
     */
    public CoralPosition getCurrentPosition() {
        currentPos = coralPositionList.get(positionListIndex);
        return this.currentPos;
    }

    public void stepBackwards() {
        positionListIndex -= 1;
        System.out.println("The button works, silly. -");
        if (positionListIndex < 0) {
            positionListIndex = 0;
        }
        getCurrentPosition();
    }

    public void stepForwards() {

        positionListIndex += 1;
        System.out.print("The button works, silly. +");

        if (positionListIndex > coralPositionList.size() - 1) {
            positionListIndex = coralPositionList.size() - 1;
        }

        getCurrentPosition();
    }

    public void resetListIndex() {
        positionListIndex = 0;
    }

    /**
     * Convert reef position IDs to heights and positions.
     *
     * @param posStr
     */
    public void addToList(String posStr) {
        if (posStr == null || posStr.isEmpty()) {
            return;
        }

        int splitIdx = posStr.length() - 2;
        String heightString = posStr.substring(splitIdx);
        String posString = posStr.substring(0, splitIdx);

        char heightIdxChar = heightString.charAt(1);

        String posIdxString = posString.substring(0);

        int heightIdx = Character.getNumericValue(heightIdxChar) - 1;
        int posIdx = Integer.parseInt(posIdxString);

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            posIdx += 12;
        }

        boolean isAlgae = (heightIdx > 4);

        coralPositionList.add(
                new CoralPosition(
                        posStr,
                        new Pose2d(
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getX(),
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getY(),
                                CoralQueueConstants.CORAL_POSITIONS[posIdx].getRotation()),
                        CoralQueueConstants.REEF_HEIGHTS[heightIdx],
                        isAlgae));
    }

    /** Clear the queue. */
    public void clearList() {
        coralPositionList.clear();
    }

    /**
     * Input position string and add it to queue via network tables. No spaces, separated by commas.
     * (e.g. B10H2,R1H3,R2H2,B1H1,B5104).
     */
    public void loadQueueFromNT() {
        clearList();
        listToQueue(ConfigManager.getInstance().get("Coral_Queue", ""));
    }

    public void loadQueueFromDefault(String profile) {
        String[] profileChosen = CoralQueueConstants.PROFILES.get(profile);

        if (profileChosen == null) {
            return;
        }

        ConfigManager.getInstance().set("Coral_Queue", String.join(",", profileChosen));

        for (String i : profileChosen) {
            addToList(i);
        }
    }

    private final NetworkTablesUtils NTUtils = NetworkTablesUtils.getTable("Coral Queue");

    // private final NetworkTablesUtils NTTelemetry = NetworkTablesUtils.getTable("Telemetry");

    public static class CoralPosition {
        private final String poseName;
        private final Pose2d pose;
        private final double height;
        private final boolean isAlgae;

        public CoralPosition(String poseName, Pose2d pose, double height, boolean isAlgae) {
            this.poseName = poseName;
            this.pose = pose;
            this.height = height;
            this.isAlgae = isAlgae;
        }

        public Pose2d getPose() {
            return this.pose;
        }

        public double getHeight() {
            return this.height;
        }

        public boolean isAlgae() {
            return this.isAlgae;
        }

        public double[] getPoseAsDoubleArray() {
            Pose2d pose = getPose();
            return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        }

        public boolean[] getBooleanHeights() {
            int splitIdx = poseName.length() - 2;
            String heightString = poseName.substring(splitIdx);

            char heightIdxChar = heightString.charAt(1);
            int heightIdx = Character.getNumericValue(heightIdxChar) - 1;

            boolean[] result = new boolean[6];
            result[heightIdx] = true;
            return result;
        }

        public String toString() {
            return poseName;
        }
    }

    public void periodic() {

        NTUtils.setArrayEntry(
                "Current Reef Pose",
                new double[] {
                    this.getCurrentPosition().getPose().getX(),
                    this.getCurrentPosition().getPose().getY(),
                    this.getCurrentPosition().getPose().getRotation().getRadians()
                });

        NTUtils.setEntry("Current Reef Pose Name", currentPos.toString());
        NTUtils.setArrayEntry("Current Reef Height", currentPos.getBooleanHeights());
        NTUtils.setEntry("Position Index", positionListIndex);
    }

    // public record CoralPosition(Pose2d pose2d, double height, boolean isAlgae) {}
}
