package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ClimberConstants {

    public static final int JOINT_MOTOR_ID = 0;
    public static final int ROTARY_MOTOR_ID = 1;

    public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS =
            new TrapezoidProfile.Constraints(1, 0.5);
    public static final double JOINT_P = 0;
    public static final double JOINT_I = 0;
    public static final double JOINT_D = 0;
    //sgva
    public static final double JOINT_S = 0;
    public static final double JOINT_G = 0;
    public static final double JOINT_V = 0;
    public static final double JOINT_A = 0;

    public static final double CURRENT_MAX = 0;
    public static final double CLIMBER_TOLERANCE = 0;
    //Current as in electricity.



//

}
