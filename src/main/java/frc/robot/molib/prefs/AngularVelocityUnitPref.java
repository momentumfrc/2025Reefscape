package frc.robot.molib.prefs;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;

public class AngularVelocityUnitPref extends UnitPref<AngularVelocityUnit> {
    public AngularVelocityUnitPref(String key, AngularVelocityUnit storeUnits, AngularVelocity defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public AngularVelocity get() {
        return (AngularVelocity) super.get();
    }
}
