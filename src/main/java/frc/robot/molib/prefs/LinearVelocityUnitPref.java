package frc.robot.molib.prefs;

import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;

public class LinearVelocityUnitPref extends UnitPref<LinearVelocityUnit> {
    public LinearVelocityUnitPref(String key, LinearVelocityUnit storeUnits, LinearVelocity defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public LinearVelocity get() {
        return (LinearVelocity) super.get();
    }
}
