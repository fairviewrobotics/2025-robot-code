/* Black Knights Robotics (C) 2025 */
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlignConstants;
import frc.robot.framework.Odometry;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.ConfigManager;

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

    private Pose3d targetPos =
            new Pose3d(497.77, 130.17, 0.0, new Rotation3d(0, 0, Math.toRadians(270)));

    public ReefAlignCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        this.xAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.2));
        this.yAxisPid.setTolerance(configManager.get("reef_align_pos_tolerance", 0.2));
        this.rotationPid.setTolerance(
                Math.toRadians(configManager.get("reef_align_rotation_tolerance", 10)));

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Pose3d robotPose = odometry.getRobotPose();
        double xAxisCalc = this.xAxisPid.calculate(robotPose.getX());
        double yAxisCalc = this.yAxisPid.calculate(robotPose.getY());
        double rotationPidCalc = this.rotationPid.calculate(robotPose.getRotation().getZ());

        swerveSubsystem.drive(xAxisCalc, yAxisCalc, rotationPidCalc, true, true);
    }

    @Override
    public void initialize() {
        // Reset PIDs to starting pose
        Pose3d robotPose = odometry.getRobotPose();
        this.xAxisPid.reset(robotPose.getX());
        this.yAxisPid.reset(robotPose.getY());
        this.rotationPid.reset(robotPose.getRotation().getZ());

        // Set PIDs to target
        this.xAxisPid.setGoal(targetPos.getX());
        this.yAxisPid.setGoal(targetPos.getY());
        this.rotationPid.setGoal(targetPos.getRotation().getZ());
    }

    @Override
    public boolean isFinished() {
        return xAxisPid.atGoal() && yAxisPid.atGoal() && rotationPid.atGoal();
    }
}
