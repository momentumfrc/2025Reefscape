package frc.robot.molib.pid;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import frc.robot.molib.encoder.MoDistanceEncoder;
import java.util.function.Consumer;

public class MoTrapezoidElevatorController
        extends MoTrapezoidController<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit> {

    private double kS, kG, kV, kA;
    private ElevatorFeedforward elevatorFF;

    public MoTrapezoidElevatorController(
            String name,
            SparkClosedLoopController pidController,
            ClosedLoopSlot pidSlot,
            MoDistanceEncoder encoder,
            Consumer<Consumer<SparkBaseConfig>> configurator) {
        super(
                name,
                pidController,
                pidSlot,
                encoder,
                configurator,
                Units.Meters,
                Units.MetersPerSecond,
                Units.MetersPerSecondPerSecond);
    }

    public void setKS(double kS) {
        this.kS = kS;
        this.elevatorFF = null;
    }

    public void setKG(double kG) {
        this.kG = kG;
        this.elevatorFF = null;
    }

    public void setKV(double kV) {
        this.kV = kV;
        this.elevatorFF = null;
    }

    public void setKA(double kA) {
        this.kA = kA;
        this.elevatorFF = null;
    }

    @Override
    protected double getFF(Measure<LinearVelocityUnit> velSetpoint, Measure<LinearVelocityUnit> nextVelSetpoint) {
        if (this.elevatorFF == null) {
            this.elevatorFF = new ElevatorFeedforward(kS, kG, kV, kA);
        }

        return this.elevatorFF.calculateWithVelocities(
                velSetpoint.in(Units.MetersPerSecond), nextVelSetpoint.in(Units.MetersPerSecond));
    }
}
