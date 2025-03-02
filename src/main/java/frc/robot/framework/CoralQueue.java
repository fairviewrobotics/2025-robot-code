/* Black Knights Robotics (C) 2025 */
package frc.robot.framework;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.ScoringConstants;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;
import java.util.ArrayList;
import java.util.Objects;
import java.util.Optional;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/** Keeps a list of future positions to place coral */
public class CoralQueue {
    private static CoralQueue INSTANCE = null;
    private static final Logger LOGGER = LogManager.getLogger();

    private final ArrayList<CoralPosition> coralPositions = new ArrayList<>();
    private final NetworkTablesUtils NTUtils = NetworkTablesUtils.getTable("CoralQueue");

    private int positionListIndex = 0;
    private boolean interrupt = false;
    private CoralPosition currentPos = new CoralPosition();

    /** Create a new instance of coral queue */
    protected CoralQueue() {}

    /**
     * Get the instance of coral queue, or create a new instance if one does not exist
     *
     * @return The instance of {@link CoralQueue}
     */
    public static CoralQueue getInstance() {
        if (INSTANCE == null) INSTANCE = new CoralQueue();

        return INSTANCE;
    }

    /**
     * Return next reef position in the queue
     *
     * @return Get the current coral queue position
     */
    public CoralPosition getCurrentPosition() {
        if (!coralPositions.isEmpty() && !this.interrupt) {
            this.currentPos = coralPositions.get(positionListIndex);
            return this.currentPos;
        } else if (this.interrupt) {
            return this.currentPos;
        } else {
            return new CoralPosition();
        }
    }

    /**
     * Return the next position to goto and increment the position in queue
     *
     * @return The next position
     */
    public CoralPosition getNext() {
        CoralPosition pos = this.getCurrentPosition();
        this.stepForwards();
        if (this.interrupt) this.interrupt = false;
        return pos;
    }

    /** Step backwards in the queue */
    public void stepBackwards() {
        this.positionListIndex -= 1;
        if (positionListIndex < 0) {
            positionListIndex = 0;
        }
        getCurrentPosition();
    }

    /** Step forwards in the queue */
    public void stepForwards() {
        positionListIndex += 1;
        if (positionListIndex > coralPositions.size() - 1) {
            positionListIndex = coralPositions.size() - 1;
        }

        getCurrentPosition();
    }

    /**
     * Interrupt the queue with a different position, the resume afterward
     *
     * @param position The position to inserted
     */
    public void interruptQueue(CoralPosition position) {
        this.stepBackwards();
        this.interrupt = true;
        this.currentPos = position;
    }

    /** Clear the queue. */
    public void clearList() {
        coralPositions.clear();
        this.positionListIndex = 0;
    }

    /** Load a profile from an entry in NT */
    public void loadQueueFromNT() {
        this.clearList();
        this.loadProfile(
                CoralQueueProfile.fromString(
                        ConfigManager.getInstance().get("Coral_Queue", "10L2,11L1")));
    }

    /**
     * Load a profile into the queue
     *
     * @param profile The {@link CoralQueueProfile}
     */
    public void loadProfile(CoralQueueProfile profile) {
        LOGGER.info("Loading profile: {}", profile);
        this.clearList();
        this.coralPositions.addAll(profile.getPositions());
    }

    /** Runs every 20ms to update NT position */
    public void periodic() {
        NTUtils.setArrayEntry(
                "Current Reef Pose",
                new double[] {
                    this.currentPos.getPose().getX(),
                    this.currentPos.getPose().getY(),
                    this.currentPos.getPose().getRotation().getRadians()
                });

        NTUtils.setEntry("Current Reef Pose Name", currentPos.toString());
        NTUtils.setArrayEntry("Current Reef Height", currentPos.getBooleanHeights());
        NTUtils.setEntry("Position Index", positionListIndex);
        NTUtils.setEntry("Num positions", coralPositions.size());
    }

    /**
     * Represents a position to score including the String id, {@link Pose2d}, and {@link
     * ScoringConstants.ScoringHeights}
     */
    public static class CoralPosition {
        private final String stringId;
        private final Pose2d pose;
        private final ScoringConstants.ScoringHeights height;

        /**
         * Create a new coral position
         *
         * @param stringId The string ID for the pose
         * @param pose A {@link Pose2d} for the soring position
         * @param height The target height
         */
        public CoralPosition(String stringId, Pose2d pose, ScoringConstants.ScoringHeights height) {
            this.stringId = stringId;
            this.pose = pose;
            this.height = height;
        }

        /** Create an empty coral position */
        public CoralPosition() {
            this.stringId = "";
            this.pose = new Pose2d();
            this.height = ScoringConstants.ScoringHeights.L1;
        }

        public static CoralPosition fromString(String string) {
            if (string == null || string.isEmpty()) {
                return null;
            }

            DriverStation.refreshData();
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

            int splitIdx = string.length() - 2;
            String heightString = string.substring(splitIdx);
            String posString = string.substring(0, splitIdx);

            int posIdx = Math.max(Math.min(Integer.parseInt(posString) - 1, 23), 0);

            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                posIdx += 12;
            }

            LOGGER.debug("CoralQ: posIdx = {}, heightString = {}", posIdx, heightString);

            return new CoralPosition(
                    string,
                    new Pose2d(
                            ScoringConstants.CORAL_POSITIONS[posIdx].getX(),
                            ScoringConstants.CORAL_POSITIONS[posIdx].getY(),
                            ScoringConstants.CORAL_POSITIONS[posIdx].getRotation()),
                    ScoringConstants.ScoringHeights.valueOf(heightString.toUpperCase()));
        }

        /**
         * Get the scoring {@link Pose2d}
         *
         * @return The scoring pose
         */
        public Pose2d getPose() {
            return this.pose;
        }

        /**
         * Get the height
         *
         * @return The elevator height as a {@link ScoringConstants.ScoringHeights}
         */
        public ScoringConstants.ScoringHeights getHeight() {
            return this.height;
        }

        /**
         * Return the pose as a double array
         *
         * @return The pose as a double array with a len of 3 (x, y, rads)
         */
        public double[] getPoseAsDoubleArray() {
            Pose2d pose = getPose();
            return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
        }

        public boolean[] getBooleanHeights() {
            boolean[] heights = new boolean[4];

            int index = Integer.parseInt(String.valueOf(this.height.toString().charAt(1))) - 1;
            heights[index] = true;
            return heights;
        }

        @Override
        public String toString() {
            return String.format(
                    "CoralPosition(stringId: %s, pose: %s, height: %s)",
                    this.stringId, this.pose.toString(), this.height);
        }

        @Override
        public int hashCode() {
            return Objects.hash(this.stringId, this.pose, this.height);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) return true;
            if (obj == null || getClass() != obj.getClass()) return false;
            CoralPosition that = (CoralPosition) obj;
            return Objects.equals(this.stringId, that.stringId)
                    && Objects.equals(this.pose, that.pose)
                    && this.height == that.height;
        }
    }

    /** A simple wrapper class for a CQ profile */
    public static class CoralQueueProfile {
        private final ArrayList<CoralPosition> positions;
        private final String rawString;

        /**
         * Coral Queue profile constructor
         *
         * @param rawString The raw string
         */
        private CoralQueueProfile(String rawString) {
            this.rawString = rawString;
            this.positions = this.parsePosString(rawString);
        }

        /**
         * Generate a profile from a string
         *
         * @param string The string (comma seperated) (Ex: 2L4,5L2,10L4)
         * @return The generated {@link CoralQueueProfile}
         */
        public static CoralQueueProfile fromString(String string) {
            return new CoralQueueProfile(string);
        }

        /**
         * Get the array of positions
         *
         * @return The array of {@link CoralPosition}s
         */
        public ArrayList<CoralPosition> getPositions() {
            return this.positions;
        }

        /**
         * Get the raw string used to generate the queue
         *
         * @return The raw string
         */
        public String getRawString() {
            return this.rawString;
        }

        /**
         * Parse a string to a list of coral profile
         *
         * @param posStrList The list of pose strings
         */
        private ArrayList<CoralPosition> parsePosString(String posStrList) {
            ArrayList<CoralPosition> res = new ArrayList<>();
            String[] posList = posStrList.split(",");
            for (String i : posList) {
                res.add(CoralPosition.fromString(i));
            }

            return res;
        }
    }
}
