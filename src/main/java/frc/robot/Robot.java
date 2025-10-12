// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.momentum4999.molib.prefs.MoPrefsImpl;
import com.momentum4999.motune.PIDTuner;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.component.FieldGeometry;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private IntegerLogEntry commandSchedulerRunTime;

    public Robot() {
        m_robotContainer = new RobotContainer();
        commandSchedulerRunTime = new IntegerLogEntry(DataLogManager.getLog(), "/Time/CommandScheduler");
    }

    @Override
    public void robotInit() {
        MoPrefsImpl.cleanUpPrefs();
        FollowPathCommand.warmupCommand().schedule();
        FieldGeometry.getInstance();
        m_robotContainer.resetFieldOrientedFwd();
    }

    @Override
    public void robotPeriodic() {
        long time = WPIUtilJNI.getSystemTime();
        CommandScheduler.getInstance().run();
        time = WPIUtilJNI.getSystemTime() - time;
        commandSchedulerRunTime.append(time);

        PIDTuner.pollAllStateValues();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.setDefaultCommandsForAuto();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        m_robotContainer.setDefaultCommands();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
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
