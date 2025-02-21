/* Black Knights Robotics (C) 2025 */
package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.controllers.MAXSwerveModule;
import frc.robot.controllers.MAXSwerveModuleConfig;
import frc.robot.framework.Odometry;
import frc.robot.utils.ConfigManager;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.SwerveUtils;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class SwerveSubsystem extends SubsystemBase {
    private static final Logger log = LogManager.getLogger(SwerveSubsystem.class);
    // Create MAXSwerveModules
    private final MAXSwerveModule frontLeft =
            new MAXSwerveModule(
                    DrivetrainConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    DrivetrainConstants.FRONT_LEFT_TURNING_CAN_ID,
                    DrivetrainConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule frontRight =
            new MAXSwerveModule(
                    DrivetrainConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    DrivetrainConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    DrivetrainConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearLeft =
            new MAXSwerveModule(
                    DrivetrainConstants.REAR_LEFT_DRIVING_CAN_ID,
                    DrivetrainConstants.REAR_LEFT_TURNING_CAN_ID,
                    DrivetrainConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

    private final MAXSwerveModule rearRight =
            new MAXSwerveModule(
                    DrivetrainConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    DrivetrainConstants.REAR_RIGHT_TURNING_CAN_ID,
                    DrivetrainConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

    // The gyro sensor
    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // Slew Rate Constants
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    // Slew Rate Limiters
    private final SlewRateLimiter magnitudeLimiter =
            new SlewRateLimiter(
                    ConfigManager.getInstance()
                            .get(
                                    "drive_magnitude_slew_rate",
                                    DrivetrainConstants.MAGNITUDE_SLEW_RATE));
    private final SlewRateLimiter rotationLimiter =
            new SlewRateLimiter(
                    ConfigManager.getInstance()
                            .get(
                                    "drive_rotational_slew_rate",
                                    DrivetrainConstants.ROTATIONAL_SLEW_RATE));

    // Slew Rate Time
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    /** Creates a new DriveSubsystem. */
    public SwerveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        gyro.setAngleAdjustment(180);

        AutoBuilder.configure(
                () -> Odometry.getInstance().getRobotPose().toPose2d(), // Robot pose supplier
                (Pose2d pose) ->
                        Odometry.getInstance()
                                .resetPose(
                                        new Pose3d(
                                                pose)), // Method to reset odometry (will be called
                // if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT
                // RELATIVE ChassisSpeeds. Also optionally outputs
                // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path
                        // following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                        ),
                DrivetrainConstants.ROBOT_AUTO_CONFIG, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red)
                            .isPresent();
                },
                this // Reference to this subsystem to set requirements
                );
    }

    // Network Tables Telemetry

    private final NetworkTablesUtils NTUtils = NetworkTablesUtils.getTable("debug");
    private final DoubleArrayEntry setpointsTelemetry =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleArrayTopic("Setpoints")
                    .getEntry(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    private final DoubleArrayEntry actualTelemetry =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleArrayTopic("Actual")
                    .getEntry(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    private final DoubleEntry gyroHeading =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleTopic("GyroHeading")
                    .getEntry(getHeadingRad());

    private final DoubleEntry frontrightpos =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleTopic("frpos")
                    .getEntry(frontRight.getPosition().angle.getRadians());

    private final DoubleEntry frontleftpos =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleTopic("flpos")
                    .getEntry(frontLeft.getPosition().angle.getRadians());

    private final DoubleEntry rearrightpos =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleTopic("rrpos")
                    .getEntry(rearRight.getPosition().angle.getRadians());

    private final DoubleEntry rearleftpos =
            NetworkTableInstance.getDefault()
                    .getTable("Swerve")
                    .getDoubleTopic("rlpos")
                    .getEntry(rearLeft.getPosition().angle.getRadians());

    public void reconfigure() {
        ConfigManager cm = ConfigManager.getInstance();

        SparkFlexConfig drivingConfig = MAXSwerveModuleConfig.drivingConfig;
        SparkMaxConfig turningConfig = MAXSwerveModuleConfig.turningConfig;

        drivingConfig.closedLoop.pid(
                cm.get("swerve_drive_p", 0.5),
                cm.get("swerve_drive_i", 0.0),
                cm.get("swerve_drive_d", 0.0));
        turningConfig.closedLoop.pid(
                cm.get("swerve_turning_p", 1),
                cm.get("swerve_turning_i", 0.0),
                cm.get("swerve_turning_d", 0.0));

        frontLeft.reconfigure(drivingConfig, turningConfig);
        rearRight.reconfigure(drivingConfig, turningConfig);
        rearRight.reconfigure(drivingConfig, turningConfig);
        rearLeft.reconfigure(drivingConfig, turningConfig);
    }

    @Override
    public void periodic() {
        NTUtils.setEntry("Gyro Angle", gyro.getAngle());

        NTUtils.setEntry("Acceleration", frontLeft.getState().speedMetersPerSecond);

        NTUtils.setEntry(
                "Gyro Accel",
                Math.sqrt(
                        Math.pow(gyro.getWorldLinearAccelX(), 2)
                                + Math.pow(gyro.getWorldLinearAccelY(), 2)));

        Odometry.getInstance()
                .addWheelOdometry(
                        new Rotation3d(Rotation2d.fromRadians(this.getHeadingRad())),
                        new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            rearRight.getPosition(),
                            rearLeft.getPosition()
                        });

        NTUtils.setArrayEntry(
                "Speeds",
                new double[] {
                    this.getFieldRelativeChassisSpeeds().vxMetersPerSecond,
                    this.getFieldRelativeChassisSpeeds().vyMetersPerSecond,
                    this.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond
                });

        frontrightpos.set(frontRight.getPosition().angle.getRadians());
        frontleftpos.set(frontLeft.getPosition().angle.getRadians());
        rearrightpos.set(rearRight.getPosition().angle.getRadians());
        rearleftpos.set(rearLeft.getPosition().angle.getRadians());

        // Set Network Tables Telemetry
        actualTelemetry.set(
                new double[] {
                    frontLeft.getPosition().angle.getRadians(),
                    frontLeft.getState().speedMetersPerSecond,
                    frontRight.getPosition().angle.getRadians(),
                    frontRight.getState().speedMetersPerSecond,
                    rearLeft.getPosition().angle.getRadians(),
                    rearLeft.getState().speedMetersPerSecond,
                    rearRight.getPosition().angle.getRadians(),
                    rearRight.getState().speedMetersPerSecond
                });

        setpointsTelemetry.set(
                new double[] {
                    frontLeft.getState().angle.getRadians(),
                    frontLeft.getState().speedMetersPerSecond,
                    frontRight.getState().angle.getRadians(),
                    frontRight.getState().speedMetersPerSecond,
                    rearLeft.getState().angle.getRadians(),
                    rearLeft.getState().speedMetersPerSecond,
                    rearRight.getState().angle.getRadians(),
                    rearRight.getState().speedMetersPerSecond
                });

        gyroHeading.set(getHeadingRad());
    }

    /**
     * Method to drive the robot.
     *
     * @param forwardMetersPerSecond Speed of the robot in the x direction (forward) in meters per
     *     second.
     * @param sidewaysMetersPerSecond Speed of the robot in the y direction (sideways) in meters per
     *     second.
     * @param radiansPerSecond Angular rate of the robot in radians per second.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit Whether slew rates should be applied to the commanded speeds.
     * @param useOdometryRotation Whether to use odometry rotation or raw gyro
     */
    public void drive(
            double forwardMetersPerSecond,
            double sidewaysMetersPerSecond,
            double radiansPerSecond,
            boolean fieldRelative,
            boolean rateLimit,
            boolean useOdometryRotation) { // TODO: We should only need odometry rotation

        // Slew Rate Limiting
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {

            // Math that calculates important stuff about where the robot is heading
            double inputTranslationDirection =
                    Math.atan2(sidewaysMetersPerSecond, forwardMetersPerSecond);
            double inputTranslationMagnitude =
                    Math.sqrt(
                            Math.pow(forwardMetersPerSecond, 2.0)
                                    + Math.pow(sidewaysMetersPerSecond, 2.0));

            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate =
                        Math.abs(
                                DrivetrainConstants.DIRECTION_SLEW_RATE
                                        / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // super high number means slew is instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;

            double angleDifference =
                    SwerveUtils.angleDifference(
                            inputTranslationDirection, currentTranslationDirection);
            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection =
                        SwerveUtils.stepTowardsCircular(
                                currentTranslationDirection,
                                inputTranslationDirection,
                                directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude
                        > 1e-4) { // small number avoids floating-point errors
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection =
                            SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude =
                            magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection =
                        SwerveUtils.stepTowardsCircular(
                                currentTranslationDirection,
                                inputTranslationDirection,
                                directionSlewRate * elapsedTime);
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            }

            previousTime = currentTime;

            xSpeedCommanded = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommanded = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(radiansPerSecond);

        } else {
            // If there's no rate limit, robot does the exact inputs given.
            xSpeedCommanded = forwardMetersPerSecond;
            ySpeedCommanded = sidewaysMetersPerSecond;
            currentRotation = radiansPerSecond;
        }

        double xSpeedDelivered = xSpeedCommanded;
        double ySpeedDelivered = ySpeedCommanded;
        double rotationDelivered = currentRotation;

        var swerveModuleStates =
                DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        xSpeedDelivered,
                                        ySpeedDelivered,
                                        rotationDelivered,
                                        Rotation2d.fromRadians(
                                                useOdometryRotation
                                                        ? Odometry.getInstance()
                                                                .getRobotPose()
                                                                .getRotation()
                                                                .getZ()
                                                        : this.getHeadingRad()))
                                : new ChassisSpeeds(
                                        xSpeedDelivered, ySpeedDelivered, rotationDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        rearLeft.setDesiredState(swerveModuleStates[2]);
        rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        rearLeft.setDesiredState(desiredStates[2]);
        rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeft.resetEncoders();
        rearLeft.resetEncoders();
        frontRight.resetEncoders();
        rearRight.resetEncoders();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -pi to pi
     */
    public double getHeadingRad() {
        return Units.degreesToRadians(-1 * (gyro.getAngle() % 360.0));
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Get the speed of the chassis relative to the robot
     *
     * @return {@link ChassisSpeeds} of the current robots speed
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState());
    }

    /**
     * Gets the robots chassis speed relative to the field
     *
     * @return Returns robot speed as a {@link ChassisSpeeds} in meters/second
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return new ChassisSpeeds(
                getRobotRelativeSpeeds().vxMetersPerSecond
                                * Math.cos(
                                        Odometry.getInstance().getRobotPose().getRotation().getZ())
                        - getRobotRelativeSpeeds().vyMetersPerSecond
                                * Math.sin(
                                        Odometry.getInstance().getRobotPose().getRotation().getZ()),
                getRobotRelativeSpeeds().vyMetersPerSecond
                                * Math.cos(
                                        Odometry.getInstance().getRobotPose().getRotation().getZ())
                        + getRobotRelativeSpeeds().vxMetersPerSecond
                                * Math.sin(
                                        Odometry.getInstance().getRobotPose().getRotation().getZ()),
                getRobotRelativeSpeeds().omegaRadiansPerSecond);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        double forward = chassisSpeeds.vxMetersPerSecond;
        double sideways = chassisSpeeds.vyMetersPerSecond;
        double rotation = chassisSpeeds.omegaRadiansPerSecond;

        drive(forward, sideways, rotation, false, false, true); // ratelimit was true, to be tested
    }

    /** Reset the gyro */
    public void zeroGyro() {
        gyro.reset();
    }
}
