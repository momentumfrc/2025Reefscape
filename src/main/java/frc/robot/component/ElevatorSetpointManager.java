package frc.robot.component;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.subsystem.ElevatorSubsystem.ElevatorPosition;
import frc.robot.utils.SetpointDataStore;
import java.util.EnumMap;

public class ElevatorSetpointManager {

    public static enum ElevatorSetpoint {
        STOW,
        PROCESSOR,
        INTAKE,
        L1,
        L2,
        L3
    };

    private static ElevatorSetpointManager instance;

    public static ElevatorSetpointManager getInstance() {
        if (instance == null) {
            instance = new ElevatorSetpointManager();
        }
        return instance;
    }

    private static class ElevatorSetpointEntry {
        private final String elevatorKey;
        private final String wristKey;

        private final SetpointDataStore store;

        private final MutDistance elevatorDistance = Units.Centimeters.mutable(0);
        private final MutAngle wristAngle = Units.Degrees.mutable(0);

        public ElevatorSetpointEntry(String key, SetpointDataStore store) {
            this.elevatorKey = String.format("%s_elevator", key);
            this.wristKey = String.format("%s_wrist", key);
            this.store = store;
        }

        public ElevatorPosition getValue() {
            return new ElevatorPosition(
                    elevatorDistance.mut_replace(store.getValue(elevatorKey), Units.Centimeters),
                    wristAngle.mut_replace(store.getValue(wristKey), Units.Rotations));
        }

        public void setValue(ElevatorPosition position) {
            store.putValue(elevatorKey, position.elevatorDistance().in(Units.Centimeters));
            store.putValue(wristKey, position.wristAngle().in(Units.Rotations));
        }
    }

    private SetpointDataStore store = SetpointDataStore.getInstance("Elevator_Setpoints");
    private EnumMap<ElevatorSetpoint, ElevatorSetpointEntry> entries = new EnumMap<>(ElevatorSetpoint.class);

    private ElevatorSetpointManager() {
        for (ElevatorSetpoint setpoint : ElevatorSetpoint.values()) {
            entries.put(setpoint, new ElevatorSetpointEntry(setpoint.name(), store));
        }
    }

    public ElevatorPosition getSetpoint(ElevatorSetpoint setpoint) {
        return entries.get(setpoint).getValue();
    }

    public void setSetpoint(ElevatorSetpoint setpoint, ElevatorPosition position) {
        entries.get(setpoint).setValue(position);
    }
}
