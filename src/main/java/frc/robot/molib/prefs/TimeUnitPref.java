package frc.robot.molib.prefs;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;

public class TimeUnitPref extends UnitPref<TimeUnit> {
    public TimeUnitPref(String key, TimeUnit storeUnits, Time defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Time get() {
        return (Time) super.get();
    }
}
