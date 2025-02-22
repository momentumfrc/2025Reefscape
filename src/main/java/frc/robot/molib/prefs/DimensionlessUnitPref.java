package frc.robot.molib.prefs;

import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.measure.Dimensionless;

public class DimensionlessUnitPref extends UnitPref<DimensionlessUnit> {
    public DimensionlessUnitPref(String key, DimensionlessUnit storeUnits, Dimensionless defaultValue) {
        super(key, storeUnits, defaultValue);
    }

    public Dimensionless get() {
        return (Dimensionless) super.get();
    }
}
