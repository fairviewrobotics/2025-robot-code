/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.AlignConstants;

public class AlignUtils {
    /**
     * Get the pose a distance, dist, back from the original target, as well as the rotation to face
     * the april tag
     *
     * @param targetPose The final target pose
     * @param dist The distance back from the target pose
     * @return The new transformed pose
     */
    public static Pose2d getFirstPose(Pose2d targetPose, double dist) {
        double theta =
                Math.atan2(
                        targetPose.getY() - AlignConstants.REEF_POSE.getY(),
                        targetPose.getX() - AlignConstants.REEF_POSE.getX());

        double distX = dist * Math.cos(theta);
        double distY = dist * Math.sin(theta);

        return new Pose2d(
                targetPose.getX() + distX, targetPose.getY() + distY, targetPose.getRotation());
    }

    /**
     * Get the angle required to look at another pose
     *
     * @param pose1 The current pose
     * @param pose2 The pose to look at
     * @return The angle it radians
     */
    public static double getAngleToTarget(Pose2d pose1, Pose2d pose2) {
        return Math.atan2(pose1.getY() - pose2.getY(), pose1.getX() - pose2.getX());
    }
}
