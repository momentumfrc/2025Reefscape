package frc.robot.molib.prefs;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularAcceleration;

public class AngularAccelerationUnitPref extends UnitPref<AngularAccelerationUnit> {
    public AngularAccelerationUnitPref(
            String key, AngularAccelerationUnit storeUnits, AngularAcceleration defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public AngularAcceleration get() {
        return (AngularAcceleration) super.get();
    }
}
