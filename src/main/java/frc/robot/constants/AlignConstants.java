/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.utils.ConfigManager;

public class AlignConstants {
    public static final double X_AXIS_P = ConfigManager.getInstance().get("align_x_p", 0.1);
    public static final double X_AXIS_I = 0.0;
    public static final double X_AXIS_D = 0.0;
    public static final TrapezoidProfile.Constraints X_AXIS_CONSTRAINTS =
            new TrapezoidProfile.Constraints(0.5, 1);

    public static final double Y_AXIS_P = ConfigManager.getInstance().get("align_y_p", 0.1);
    public static final double Y_AXIS_I = 0.0;
    public static final double Y_AXIS_D = 0.0;
    public static final TrapezoidProfile.Constraints Y_AXIS_CONSTRAINTS =
            new TrapezoidProfile.Constraints(0.5, 1);

    public static final double ROTATION_P = ConfigManager.getInstance().get("align_rot_p", 0.1);
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS =
            new TrapezoidProfile.Constraints(Math.PI, Math.PI);

    private static final Pose2d REEF_POSE_RED =
            new Pose2d(new Translation2d(13.0, 4.0), new Rotation2d());

    private static final Pose2d REEF_POSE_BLUE = new Pose2d();

    public static final Pose2d REEF_POSE =
            DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                    ? REEF_POSE_BLUE
                    : REEF_POSE_RED;
}
