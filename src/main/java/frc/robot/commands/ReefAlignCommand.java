/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignConstants;
import frc.robot.framework.Odometry;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AlignUtils;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;

public class ReefAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private ProfiledPIDController xAxisPid =
            new ProfiledPIDController(
                    AlignConstants.X_AXIS_P,
                    AlignConstants.X_AXIS_I,
                    AlignConstants.X_AXIS_D,
                    AlignConstants.X_AXIS_CONSTRAINTS);

    private ProfiledPIDController yAxisPid =
            new ProfiledPIDController(
                    AlignConstants.Y_AXIS_P,
                    AlignConstants.Y_AXIS_I,
                    AlignConstants.Y_AXIS_D,
                    AlignConstants.Y_AXIS_CONSTRAINTS);

    private ProfiledPIDController rotationPid =
            new ProfiledPIDController(
                    AlignConstants.ROTATION_P,
                    AlignConstants.ROTATION_I,
                    AlignConstants.ROTATION_D,
                    AlignConstants.ROTATION_CONSTRAINTS);

    private final Odometry odometry = Odometry.getInstance();
    private final ConfigManager configManager = ConfigManager.getInstance();

    private Pose2d targetPos = new Pose2d(12.499, 2.922, Rotation2d.fromRadians(1.020));

    private NetworkTablesUtils debug = NetworkTablesUtils.getTable("debug");

    private int step = 0;

    private double stepOneTimestamp = -1;

    public ReefAlignCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.xAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.05));
        this.yAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.05));
        this.rotationPid.setTolerance(
                Math.toRadians(configManager.get("reef_align_rotation_tolerance", 1)));

        this.rotationPid.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.step = 0;
        this.stepOneTimestamp = -1;
        this.targetPos =
                new Pose2d(
                        new Translation2d(
                                configManager.get("align_target_x", 12.499),
                                configManager.get("align_target_y", 2.922)),
                        Rotation2d.fromRadians(configManager.get("align_target_rads", 1.020)));
        // Reset PIDs to starting pose

        Pose3d robotPose = odometry.getRobotPose();
        this.xAxisPid.reset(robotPose.getX());
        this.yAxisPid.reset(robotPose.getY());
        this.rotationPid.reset(robotPose.getRotation().getZ());

        // PID updates
        this.xAxisPid.setP(configManager.get("align_x_p", 0.5));
        this.yAxisPid.setP(configManager.get("align_y_p", 0.5));
        this.rotationPid.setP(configManager.get("align_rot_p", 0.5));

        this.xAxisPid.setD(configManager.get("align_x_d", 0.5));
        this.yAxisPid.setD(configManager.get("align_y_d", 0.5));
        this.rotationPid.setD(configManager.get("align_rot_d", 0.5));

        this.xAxisPid.setI(configManager.get("align_x_i", 0.0));
        this.yAxisPid.setI(configManager.get("align_y_i", 0.0));
        this.rotationPid.setI(configManager.get("align_rot_i", 0.0));

        this.xAxisPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        configManager.get("align_x_max_vel_m", 1.0),
                        configManager.get("align_x_max_accel_mps", 1.0)));
        this.yAxisPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        configManager.get("align_y_max_vel_m", 1.0),
                        configManager.get("align_y_max_accel_mps", 1.0)));
        this.rotationPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        Math.toRadians(configManager.get("align_rot_max_vel_deg", 1.0)),
                        Math.toRadians(configManager.get("align_rot_max_accel_degps", 180))));
    }

    @Override
    public void execute() {
        Pose2d currTarget = new Pose2d();
        debug.setEntry("Step", this.step);
        switch (step) {
            case 0:
                {
                    currTarget =
                            AlignUtils.getFirstPose(
                                    this.targetPos, configManager.get("align_rough_back", .5));

                    this.xAxisPid.setTolerance(configManager.get("align_pos_rough_tol", 0.2));
                    this.yAxisPid.setTolerance(configManager.get("align_pos_rough_tol", 0.2));
                    this.rotationPid.setTolerance(
                            Math.toRadians(configManager.get("align_rot_rough_tol", 1.0)));

                    if (this.xAxisPid.atGoal()
                            && this.yAxisPid.atGoal()
                            && this.rotationPid.atGoal()) {
                        this.step = 1;
                        this.stepOneTimestamp = Timer.getFPGATimestamp() * 1000;
                        return;
                    }
                    break;
                }
            case 1:
                {
                    currTarget = this.targetPos;
                    this.xAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.05));
                    this.yAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.05));
                    this.rotationPid.setTolerance(
                            Math.toRadians(configManager.get("reef_align_rotation_tolerance", 1)));
                    break;
                }
        }

        // Set PIDs to target
        this.xAxisPid.setGoal(currTarget.getX());
        this.yAxisPid.setGoal(currTarget.getY());
        this.rotationPid.setGoal(currTarget.getRotation().getRadians());

        Pose3d robotPose = odometry.getRobotPose();
        double xAxisCalc = this.xAxisPid.calculate(robotPose.getX());
        double yAxisCalc = this.yAxisPid.calculate(robotPose.getY());
        double rotationPidCalc = this.rotationPid.calculate(robotPose.getRotation().getZ());

        debug.setEntry("X Pid Error", this.xAxisPid.getPositionError());
        debug.setEntry("Y Pid Error", this.yAxisPid.getPositionError());
        debug.setEntry("Rot Pid Error", this.rotationPid.getPositionError());

        debug.setEntry("X Pid setpoint", this.xAxisPid.atSetpoint());
        debug.setEntry("X Pid goal", this.xAxisPid.atGoal());

        debug.setEntry("y Pid setpoint", this.xAxisPid.atSetpoint());
        debug.setEntry("Y Pid goal", this.yAxisPid.atGoal());

        debug.setEntry("Rot Pid setpoint", this.rotationPid.atSetpoint());
        debug.setEntry("Rot Pid goal", this.rotationPid.atGoal());

        double finalX = xAxisCalc + (Math.signum(xAxisCalc) * configManager.get("align_ff", 0.1));
        double finalY = yAxisCalc + (Math.signum(yAxisCalc) * configManager.get("align_ff", 0.1));

        debug.setEntry("Xms", finalX);
        debug.setEntry("Yms", finalY);
        debug.setEntry("Rrads", rotationPidCalc);

        this.debug.setArrayEntry(
                "target_pose",
                new double[] {
                    currTarget.getX(), currTarget.getY(), currTarget.getRotation().getRadians()
                });

        swerveSubsystem.drive(finalX, finalY, rotationPidCalc, true, true, true);
    }

    @Override
    public boolean isFinished() {
        return xAxisPid.atGoal()
                && yAxisPid.atGoal()
                && rotationPid.atGoal()
                && this.step == 1
                && Timer.getFPGATimestamp() * 1000 - this.stepOneTimestamp
                        > configManager.get("algin_step_one_time", 200.0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false, true, false);
    }
}
