package frc.robot.utils;

import com.pathplanner.lib.config.PIDConstants;

public class MutablePIDConstants {
    public double kP, kI, kD, iZone;

    public MutablePIDConstants() {
        this.iZone = 1.0;
    }

    public PIDConstants toImmutable() {
        return new PIDConstants(kP, kI, kD, iZone);
    }
}
