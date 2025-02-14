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
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;

public class AlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    private final ProfiledPIDController xAxisPid =
            new ProfiledPIDController(
                    AlignConstants.X_AXIS_P,
                    AlignConstants.X_AXIS_I,
                    AlignConstants.X_AXIS_D,
                    AlignConstants.X_AXIS_CONSTRAINTS);

    private final ProfiledPIDController yAxisPid =
            new ProfiledPIDController(
                    AlignConstants.Y_AXIS_P,
                    AlignConstants.Y_AXIS_I,
                    AlignConstants.Y_AXIS_D,
                    AlignConstants.Y_AXIS_CONSTRAINTS);

    private final ProfiledPIDController rotationPid =
            new ProfiledPIDController(
                    AlignConstants.ROTATION_P,
                    AlignConstants.ROTATION_I,
                    AlignConstants.ROTATION_D,
                    AlignConstants.ROTATION_CONSTRAINTS);

    private final Odometry odometry = Odometry.getInstance();
    private final ConfigManager configManager = ConfigManager.getInstance();

    private final Pose2d targetPos;
    private final String profile;

    private final NetworkTablesUtils debug = NetworkTablesUtils.getTable("debug");

    private double timeSenseFinished = -1;

    public AlignCommand(SwerveSubsystem swerveSubsystem, Pose2d targetPose, String profile) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPos = targetPose;
        this.profile = profile;

        this.xAxisPid.setTolerance(
                configManager.get(String.format("Align/%s/Pos_Tolerance", this.profile), 0.05));
        this.yAxisPid.setTolerance(
                configManager.get(String.format("Align/%s/Pos_Tolerance", this.profile), 0.05));
        this.rotationPid.setTolerance(
                Math.toRadians(
                        configManager.get(
                                String.format("Align/%s/Rotation_Tolerance", this.profile), 1)));

        this.rotationPid.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        this.timeSenseFinished = -1;
        //        this.targetPos =
        //                new Pose2d(
        //                        new Translation2d(
        //                                configManager.get(String.format("Align/%s/Target_X",
        // this.profile), 12.499),
        //                                configManager.get(String.format("Align/%s/Target_Y",
        // this.profile), 2.922)),
        //
        // Rotation2d.fromRadians(configManager.get(String.format("Align/%s/Target_Rads",
        // this.profile), 1.020)));
        // Reset PIDs to starting pose

        Pose3d robotPose = odometry.getRobotPose();
        this.xAxisPid.reset(robotPose.getX());
        this.yAxisPid.reset(robotPose.getY());
        this.rotationPid.reset(robotPose.getRotation().getZ());

        // PID updates
        this.xAxisPid.setP(
                configManager.get(String.format("Align/%s/X_Axis_P", this.profile), 0.5));
        this.yAxisPid.setP(
                configManager.get(String.format("Align/%s/Y_Axis_P", this.profile), 0.5));
        this.rotationPid.setP(
                configManager.get(String.format("Align/%s/Rot_P", this.profile), 0.5));

        this.xAxisPid.setD(
                configManager.get(String.format("Align/%s/X_Axis_D", this.profile), 0.5));
        this.yAxisPid.setD(
                configManager.get(String.format("Align/%s/Y_Axis_D", this.profile), 0.5));
        this.rotationPid.setD(
                configManager.get(String.format("Align/%s/Rot_D", this.profile), 0.5));

        this.xAxisPid.setI(
                configManager.get(String.format("Align/%s/X_Axis_I", this.profile), 0.0));
        this.yAxisPid.setI(
                configManager.get(String.format("Align/%s/Y_Axis_I", this.profile), 0.0));
        this.rotationPid.setI(
                configManager.get(String.format("Align/%s/Rot_I", this.profile), 0.0));

        this.xAxisPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        configManager.get(String.format("Align/%s/X_Max_Vel_M", this.profile), 1.0),
                        configManager.get(
                                String.format("Align/%s/X_Max_Accel_Mps", this.profile), 1.0)));
        this.yAxisPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        configManager.get(String.format("Align/%s/Y_Max_Vel_M", this.profile), 1.0),
                        configManager.get(
                                String.format("Align/%s/Y_Max_Accel_Mps", this.profile), 1.0)));
        this.rotationPid.setConstraints(
                new TrapezoidProfile.Constraints(
                        Math.toRadians(
                                configManager.get(
                                        String.format("Align/%s/Rot_Max_Vel_Deg", this.profile),
                                        1.0)),
                        Math.toRadians(
                                configManager.get(
                                        String.format("Align/%s/Rot_Max_Accel_Degps", this.profile),
                                        180))));

        this.xAxisPid.setTolerance(
                configManager.get(String.format("Align/%s/Pos_Tolerance", this.profile), 0.05));
        this.yAxisPid.setTolerance(
                configManager.get(String.format("Align/%s/Pos_Tolerance", this.profile), 0.05));
        this.rotationPid.setTolerance(
                Math.toRadians(
                        configManager.get(
                                String.format("Align/%s/Rotation_Tolerance", this.profile), 1)));
    }

    @Override
    public void execute() {
        // Set PIDs to target
        this.xAxisPid.setGoal(this.targetPos.getX());
        this.yAxisPid.setGoal(this.targetPos.getY());
        this.rotationPid.setGoal(this.targetPos.getRotation().getRadians());

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

        double finalX =
                xAxisCalc
                        + (Math.signum(xAxisCalc)
                                * configManager.get(
                                        String.format("Align/%s/ff", this.profile), 0.1));
        double finalY =
                yAxisCalc
                        + (Math.signum(yAxisCalc)
                                * configManager.get(
                                        String.format("Align/%s/ff", this.profile), 0.1));

        debug.setEntry("Xms", finalX);
        debug.setEntry("Yms", finalY);
        debug.setEntry("Rrads", rotationPidCalc);

        this.debug.setArrayEntry(
                "target_pose",
                new double[] {
                    this.targetPos.getX(),
                    this.targetPos.getY(),
                    this.targetPos.getRotation().getRadians()
                });

        swerveSubsystem.drive(finalX, finalY, rotationPidCalc, true, true, true);

        if (xAxisPid.atGoal() && yAxisPid.atGoal() && rotationPid.atGoal())
            this.timeSenseFinished = Timer.getFPGATimestamp() * 1000;
    }

    @Override
    public boolean isFinished() {
        return xAxisPid.atGoal()
                && yAxisPid.atGoal()
                && rotationPid.atGoal()
                && Timer.getFPGATimestamp() * 1000 - this.timeSenseFinished
                        > configManager.get(
                                String.format("Align/%s/Finish_Time", this.profile), 200.0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0, 0, 0, false, true, false);
    }
}
