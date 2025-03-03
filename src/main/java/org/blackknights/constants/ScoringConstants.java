/* Black Knights Robotics (C) 2025 */
package org.blackknights.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.blackknights.framework.CoralQueue;
import org.blackknights.utils.ConfigManager;

/** Scoring related constants */
public class ScoringConstants {
    public static final Map<String, CoralQueue.CoralQueueProfile> PROFILES = new HashMap<>();

    static {
        PROFILES.put(
                "PROFILE_1", CoralQueue.CoralQueueProfile.fromString("6L4,7L4,8L4,9L4,10L4,11L4"));
        PROFILES.put("PROFILE_2", CoralQueue.CoralQueueProfile.fromString("0L3,1L4"));
    }

    private static final List<AprilTag> aprilPoses =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTags();

    public static final Pose2d[] CORAL_POSITIONS =
            new Pose2d[] {
                getPoseFromTag("right", 10), // R1
                getPoseFromTag("left", 9), // R2
                getPoseFromTag("right", 9), // R3
                getPoseFromTag("left", 8), // R4
                getPoseFromTag("right", 8), // R5
                getPoseFromTag("left", 7), // R6
                getPoseFromTag("right", 7), // R7
                getPoseFromTag("left", 6), // R8
                getPoseFromTag("right", 6), // R9
                getPoseFromTag("left", 11), // R10
                getPoseFromTag("right", 11), // R11
                getPoseFromTag("left", 10), // R12
                getPoseFromTag("right", 21), // B1
                getPoseFromTag("left", 22), // B2
                getPoseFromTag("right", 22), // B3
                getPoseFromTag("left", 17), // B4
                getPoseFromTag("right", 17), // B5
                getPoseFromTag("left", 18), // B6
                getPoseFromTag("right", 18), // B7
                getPoseFromTag("left", 19), // B8
                getPoseFromTag("right", 19), // B9
                getPoseFromTag("left", 20), // B10
                getPoseFromTag("right", 20), // B11
                getPoseFromTag("left", 21), // B12
            };

    public static final Pose2d INTAKE_RED = new Pose2d();
    public static final Pose2d INTAKE_BLUE = new Pose2d();


    /**
     * Different scoring heights
     */
    public enum ScoringHeights {
        L1,
        L2,
        L3,
        L4,
        INTAKE,
    }

    /**
     * Get a scoring position from an april tag id
     * @param offsetKey The key in config manager (scoring_{side}_x and scoring_{side}_y) for the x and y offsets
     * @param id The april tag id to base of pose from
     * @return The target scoring position
     */
    public static Pose2d getPoseFromTag(String offsetKey, int id) {
        Pose2d p =
                aprilPoses
                        .get(id - 1)
                        .pose
                        .toPose2d()
                        .transformBy(
                                new Transform2d(
                                        ConfigManager.getInstance()
                                                .get(String.format("scoring_%s_x", offsetKey), -.5),
                                        ConfigManager.getInstance()
                                                .get(String.format("scoring_%s_y", offsetKey), -0.25),
                                        aprilPoses.get(id - 1).pose.getRotation().toRotation2d()));

        return new Pose2d(
                p.getX(),
                p.getY(),
                new Rotation2d(
                        aprilPoses.get(id - 1).pose.getRotation().toRotation2d().getRadians()
                                + Math.PI));
    }
}
