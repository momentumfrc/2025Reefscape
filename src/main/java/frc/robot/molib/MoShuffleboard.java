// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.molib;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

public class MoShuffleboard {
    private static MoShuffleboard instance;

    public static synchronized MoShuffleboard getInstance() {
        if (instance == null) {
            try {
                instance = new MoShuffleboard();
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Failed to initialize MoShuffleboard", e.getStackTrace());
                instance = null;
            }
        }

        return instance;
    }

    private enum SysIdMode {
        NONE,
        QUASISTATIC_FORWARD,
        QUASISTATIC_REVERSE,
        DYNAMIC_FORWARD,
        DYNAMIC_REVERSE
    };

    private final SendableChooser<SysIdMode> sysidMode = MoShuffleboard.enumToChooser(SysIdMode.class);

    public final ShuffleboardTab driveTab;
    public final ShuffleboardTab groundIntakeTab;
    public final ShuffleboardTab climberTab;
    public final ShuffleboardTab settingsTab;
    public final ShuffleboardTab autoTab;
    public final Field2d field;

    private final GenericEntry voltRampEntry;
    private final GenericEntry voltEntry;

    public final GenericSubscriber tuneSetpointSubscriber;

    private MoShuffleboard() {
        // Note: if you're getting an IllegalArgumentException here complaining about "title already in use",
        // check that you're not calling MoShuffleboard.getInstance() from within this constructor!
        settingsTab = Shuffleboard.getTab("Settings");
        autoTab = Shuffleboard.getTab("Auto");
        driveTab = Shuffleboard.getTab("Drive");
        groundIntakeTab = Shuffleboard.getTab("Ground Intake");
        climberTab = Shuffleboard.getTab("Climber");
        field = new Field2d();

        driveTab.add(field).withSize(5, 3);

        settingsTab.add("Sysid Mode", sysidMode);

        var sysidGroup = settingsTab
                .getLayout("Sysid Settings", BuiltInLayouts.kList)
                .withSize(2, 1)
                .withProperties(Map.of("Label position", "RIGHT"));
        voltRampEntry = sysidGroup.add("Volts Ramp", 1.5).getEntry();
        voltEntry = sysidGroup.add("Volts Step", 2).getEntry();

        tuneSetpointSubscriber = settingsTab
                .add("Tune Setpoints?", false)
                .withWidget(BuiltInWidgets.kToggleSwitch)
                .getEntry();
    }

    public SysIdRoutine.Config getSysidConfig() {
        var voltsPerSec = Units.Volts.per(Units.Second);
        return new SysIdRoutine.Config(
                voltsPerSec.of(voltRampEntry.getDouble(1.5)),
                Units.Volts.of(voltEntry.getDouble(3)),
                Units.Seconds.of(45));
    }

    public Command getSysidCommand(Supplier<SysIdRoutine> routineSupplier, Subsystem... subsystems) {
        return Commands.defer(
                () -> {
                    var routine = routineSupplier.get();
                    switch (sysidMode.getSelected()) {
                        case QUASISTATIC_FORWARD:
                            return routine.quasistatic(SysIdRoutine.Direction.kForward);
                        case QUASISTATIC_REVERSE:
                            return routine.quasistatic(SysIdRoutine.Direction.kReverse);
                        case DYNAMIC_FORWARD:
                            return routine.dynamic(SysIdRoutine.Direction.kForward);
                        case DYNAMIC_REVERSE:
                            return routine.dynamic(SysIdRoutine.Direction.kReverse);
                        case NONE:
                        default:
                            return Commands.none();
                    }
                },
                Set.of(subsystems));
    }

    public static <T extends Enum<?>> SendableChooser<T> enumToChooser(Class<T> toConvert) {
        boolean setDefault = true;
        var chooser = new SendableChooser<T>();
        for (T entry : toConvert.getEnumConstants()) {
            if (setDefault) {
                chooser.setDefaultOption(entry.name(), entry);
                setDefault = false;
            } else {
                chooser.addOption(entry.name(), entry);
            }
        }
        return chooser;
    }
}
