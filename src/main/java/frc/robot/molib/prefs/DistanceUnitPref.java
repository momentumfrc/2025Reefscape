package frc.robot.molib.prefs;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class DistanceUnitPref extends UnitPref<DistanceUnit> {
    public DistanceUnitPref(String key, DistanceUnit storeUnits, Distance defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Distance get() {
        return (Distance) super.get();
    }
}
