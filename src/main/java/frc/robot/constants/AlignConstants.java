/* Black Knights Robotics (C) 2025 */
package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AlignConstants {
    public static final double X_AXIS_P = 0.0;
    public static final double X_AXIS_I = 0.0;
    public static final double X_AXIS_D = 0.0;
    public static final TrapezoidProfile.Constraints X_AXIS_CONSTRAINTS =
            new TrapezoidProfile.Constraints(3, 3);

    public static final double Y_AXIS_P = 0.0;
    public static final double Y_AXIS_I = 0.0;
    public static final double Y_AXIS_D = 0.0;
    public static final TrapezoidProfile.Constraints Y_AXIS_CONSTRAINTS =
            new TrapezoidProfile.Constraints(3, 3);

    public static final double ROTATION_P = 0.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS =
            new TrapezoidProfile.Constraints(Math.PI, Math.PI);
}
