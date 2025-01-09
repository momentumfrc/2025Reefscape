package frc.robot.molib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.Optional;

public class MoShuffleboard {
    private static Optional<MoShuffleboard> instance = Optional.empty();

    public static synchronized Optional<MoShuffleboard> getInstance() {
        if (instance.isEmpty()) {
            try {
                instance = Optional.of(new MoShuffleboard());
            } catch (IllegalArgumentException e) {
                DriverStation.reportError("Failed to initialize MoShuffleboard", e.getStackTrace());
            }
        }

        return instance;
    }

    public final ShuffleboardTab driveTab;

    private MoShuffleboard() {
        driveTab = Shuffleboard.getTab("Drive");
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
