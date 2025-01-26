/* Black Knights Robotics (C) 2025 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        robotContainer.robotInit();
        //        Logger.recordMetadata("ProjectName", "2025_Robot");
        //
        //        if (isReal()) {
        //            Logger.addDataReceiver(new NT4Publisher());
        //            new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
        //        } else {
        //            setUseTiming(false);
        //            String logPath = LogFileUtil.findReplayLog();
        //            Logger.setReplaySource(new WPILOGReader(logPath));
        //            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
        // "_sim")));
        //        }
        //
        //        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.robotPeriodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
