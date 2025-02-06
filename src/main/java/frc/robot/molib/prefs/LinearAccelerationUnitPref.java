package frc.robot.molib.prefs;

import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.measure.LinearAcceleration;

public class LinearAccelerationUnitPref extends UnitPref<LinearAccelerationUnit> {
    public LinearAccelerationUnitPref(String key, LinearAccelerationUnit storeUnits, LinearAcceleration defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public LinearAcceleration get() {
        return (LinearAcceleration) super.get();
    }
}
