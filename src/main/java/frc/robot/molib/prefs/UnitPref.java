package frc.robot.molib.prefs;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import java.util.function.Consumer;

public class UnitPref<U extends Unit> {
    private final Pref<Double> basePref;
    private final U storeUnits;

    private final MutableMeasure<U, ?, ?> currValue;

    public UnitPref(String key, U storeUnits, Measure<U> defaultValue) {
        String symbol = storeUnits.symbol().replaceAll("/", "_");

        currValue = defaultValue.mutableCopy();

        this.basePref = new Pref<>(
                String.format("%s (%s)", key, symbol),
                defaultValue.in(storeUnits),
                NetworkTableValue::getDouble,
                NetworkTableEntry::setDouble);

        this.storeUnits = storeUnits;
    }

    public Measure<U> get() {
        return currValue.mut_replace(basePref.get(), storeUnits);
    }

    public void set(Measure<U> value) {
        basePref.set(value.in(storeUnits));
    }

    public void subscribe(Consumer<Measure<U>> consumer) {
        subscribe(consumer, false);
    }

    public void subscribe(Consumer<Measure<U>> consumer, boolean notifyImmediately) {
        basePref.subscribe((value) -> consumer.accept(currValue.mut_replace(value, storeUnits)), notifyImmediately);
    }

    public String getKey() {
        return basePref.getKey();
    }
}
