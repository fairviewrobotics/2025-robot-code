/* Black Knights Robotics (C) 2025 */
package org.blackknights.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.blackknights.utils.ConfigManager;

/** Vision related constants */
public class VisionConstants {
    public static double WHEEL_TRUST = 0.1;
    public static double VISION_TRUST = 0.5;

    public static final Transform3d LEFT_CAM_TRANSFORM =
            new Transform3d(
                    0.253,
                    0.336,
                    0.229,
                    new Rotation3d(
                            0.0,
                            0.0,
                            Math.toRadians(
                                    ConfigManager.getInstance().get("left_cam_angle", -10.0))));

    public static final Transform3d RIGHT_CAM_TRANSFORM =
            new Transform3d(
                    0.253,
                    -0.3995,
                    0.229,
                    new Rotation3d(
                            0.0,
                            0.0,
                            Math.toRadians(
                                    ConfigManager.getInstance().get("right_cam_angle", 10.0))));

    public static final Transform3d CENTER_CAM_TRANSFORM =
            new Transform3d(
                    0.1, //  0.341122 0.3832
                    0.0,
                    0.2040382,
                    new Rotation3d(
                            0.0,
                            Math.toRadians(
                                    ConfigManager.getInstance().get("center_cam_pitch", 45.0)),
                            0.0));
}
