/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;

public class CoralQueueConstants {
    public static final Pose2d[] CORAL_POSITIONS =
            new Pose2d[] {
                new Pose2d(5.0, 7.0, Rotation2d.fromDegrees(12.0)), // R0
                new Pose2d(9.0, 0.0, Rotation2d.fromDegrees(0.0)), // R1
                new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.0)), // R2
                new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0.0)), // R3
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), // R4
                new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0.0)), // R5
                new Pose2d(6.0, 1.0, Rotation2d.fromDegrees(0.0)), // R6
                new Pose2d(4.0, 10.0, Rotation2d.fromDegrees(0.0)), // R7
                new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0.0)), // R8
                new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0.0)), // R9
                new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0.0)), // R10
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), // R11
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), // B0
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), // B1
                new Pose2d(3.0, 12.0, Rotation2d.fromDegrees(9.0)), // B2
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), // B3
                new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0.0)), // B4
                new Pose2d(5.0, 0.0, Rotation2d.fromDegrees(0.0)), // B5
                new Pose2d(9.0, 0.0, Rotation2d.fromDegrees(0.0)), // B6
                new Pose2d(20.0, 0.0, Rotation2d.fromDegrees(0.0)), // B7
                new Pose2d(30.0, 0.0, Rotation2d.fromDegrees(0.0)), // B8
                new Pose2d(40.0, 0.0, Rotation2d.fromDegrees(0.0)), // B9
                new Pose2d(13.0, 24.0, Rotation2d.fromDegrees(45.0)), // B10
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)) // B11
            };

    public static final double[] REEF_HEIGHTS = new double[] {1.0, 0.0, 0.0, 5.0, 0.0, 0.0};

    public static final Map<String, String[]> PROFILES = new HashMap<>();

    static {
        PROFILES.put("PROFILE_1", new String[] {"10H2", "9H3", "8H4", "7H2", "6H4", "5H3", "4H2"});
        PROFILES.put("PROFILE_2", new String[] {"5H3", "4H3", "3H3", "2H3", "1H3"});
    }
}
