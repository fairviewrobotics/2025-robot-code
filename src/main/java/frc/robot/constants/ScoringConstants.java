/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;

public class ScoringConstants {
    public static final Pose2d[] CORAL_POSITIONS =
            new Pose2d[] {
                new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)), // R0
                new Pose2d(9.0, 0.0, Rotation2d.fromDegrees(5.0)), // R1
                new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.1)), // R2
                new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0.0)), // R3
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.1)), // R4
                new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.0)), // R5
                new Pose2d(6.0, 1.0, Rotation2d.fromDegrees(0.0)), // R6
                new Pose2d(4.0, 10.0, Rotation2d.fromDegrees(0.0)), // R7
                new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0.0)), // R8
                new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0.0)), // R9
                new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0.0)), // R10
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.2)), // R11
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.3)), // B0
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.4)), // B1
                new Pose2d(3.0, 12.0, Rotation2d.fromDegrees(9.0)), // B2
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.5)), // B3
                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)), // B4
                new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.0)), // B5
                new Pose2d(9.0, 0.0, Rotation2d.fromDegrees(0.0)), // B6
                new Pose2d(20.0, 0.0, Rotation2d.fromDegrees(0.0)), // B7
                new Pose2d(30.0, 0.0, Rotation2d.fromDegrees(0.0)), // B8
                new Pose2d(40.0, 0.0, Rotation2d.fromDegrees(0.0)), // B9
                new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)), // B10
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)) // B11
            };

    public enum ScoringHeights {
        L1,
        L2,
        L3,
        L4
    }

    public static final Map<String, String[]> PROFILES = new HashMap<>();

    static {
        PROFILES.put("PROFILE_1", new String[] {"10L2", "9L3", "8L4", "7L2", "6L4", "5L3", "4L2"});
        PROFILES.put("PROFILE_2", new String[] {"5L3", "4L3", "3L3", "2L3", "1L3"});
    }
}
