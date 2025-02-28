/* Black Knights Robotics (C) 2025 */
package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class AlignUtils {
    /**
     * Get a new pose back some distance in meters
     *
     * @param initialPose The initial pose
     * @param distBack The distance to go back
     * @return The new pose
     */
    public static Pose2d getXDistBack(Pose2d initialPose, double distBack) {
        Pose2d p =
                initialPose.transformBy(new Transform2d(-distBack, 0.0, initialPose.getRotation()));

        return new Pose2d(p.getX(), p.getY(), initialPose.getRotation());
    }
}
