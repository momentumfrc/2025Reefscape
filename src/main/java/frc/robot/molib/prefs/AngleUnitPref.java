package frc.robot.molib.prefs;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class AngleUnitPref extends UnitPref<AngleUnit> {
    public AngleUnitPref(String key, AngleUnit storeUnits, Angle defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Angle get() {
        return (Angle) super.get();
    }
}
