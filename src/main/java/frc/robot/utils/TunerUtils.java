// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.momentum4999.molib.pid.MoSparkMaxArmPID;
import com.momentum4999.molib.pid.MoSparkMaxElevatorPID;
import com.momentum4999.molib.pid.MoSparkMaxPID;
import com.momentum4999.molib.pid.MoTalonFxPID;
import com.momentum4999.molib.pid.MoTrapezoidArmController;
import com.momentum4999.molib.pid.MoTrapezoidController;
import com.momentum4999.molib.pid.MoTrapezoidElevatorController;
import com.momentum4999.motune.PIDTuner;
import com.momentum4999.motune.PIDTunerBuilder;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import frc.robot.Constants;

public class TunerUtils {

    private static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTunerBuilder moSparkBase(
            MoSparkMaxPID<Dim, VDim> sparkMax, String controllerName) {
        PIDTunerBuilder builder = PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(sparkMax::setP)
                .withI(sparkMax::setI)
                .withD(sparkMax::setD)
                .withIZone(sparkMax::setIZone)
                .withSetpoint(sparkMax::getSetpoint)
                .withMeasurement(sparkMax::getLastMeasurement);

        if (sparkMax.getType() == MoSparkMaxPID.Type.SMARTMOTION) {
            builder = builder.withProperty(
                            "maxVel",
                            (v) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxVelocity(v, sparkMax.getPidSlot())))
                    .withProperty(
                            "maxAccel",
                            (a) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxAcceleration(a, sparkMax.getPidSlot())))
                    .withProperty(
                            "allowedError",
                            (e) -> sparkMax.setConfigOption(c -> c.closedLoop.maxMotion.allowedClosedLoopError(e)));
        } else if (sparkMax.getType() == MoSparkMaxPID.Type.SMARTVELOCITY) {
            builder = builder.withProperty(
                            "maxAccel",
                            (a) -> sparkMax.setConfigOption(
                                    c -> c.closedLoop.maxMotion.maxAcceleration(a, sparkMax.getPidSlot())))
                    .withProperty(
                            "allowedError",
                            (e) -> sparkMax.setConfigOption(c -> c.closedLoop.maxMotion.allowedClosedLoopError(e)));
        }

        return builder;
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTuner forMoSparkMax(
            MoSparkMaxPID<Dim, VDim> sparkMax, String controllerName) {
        return moSparkBase(sparkMax, controllerName).withFF(sparkMax::setFF).safeBuild();
    }

    public static PIDTuner forMoSparkArm(MoSparkMaxArmPID armPID, String controllerName) {
        return moSparkBase(armPID, controllerName)
                .withProperty("ff_builtin", armPID::setFF)
                .withProperty("ff_kS", armPID::setKS)
                .withProperty("ff_kG", armPID::setKG)
                .withProperty("ff_kV", armPID::setKV)
                .withStateValue("calculated_ff", armPID::getLastFF)
                .safeBuild();
    }

    public static PIDTuner forMoSparkElevator(MoSparkMaxElevatorPID controller, String controllerName) {
        return moSparkBase(controller, controllerName)
                .withProperty("ff_builtin", controller::setFF)
                .withProperty("ff_kS", controller::setKS)
                .withProperty("ff_kG", controller::setKG)
                .withProperty("ff_kV", controller::setKV)
                .withStateValue("calculated_ff", controller::getLastFF)
                .safeBuild();
    }

    public static <Dim extends Unit, VDim extends PerUnit<Dim, TimeUnit>> PIDTuner forMoTalonFx(
            MoTalonFxPID<Dim, VDim> talon, String controllerName) {
        return PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(talon::setP)
                .withI(talon::setI)
                .withD(talon::setD)
                .withFF(talon::setFF)
                .withIZone(talon::setIZone)
                .withSetpoint(talon::getSetpoint)
                .withMeasurement(talon::getLastMeasurement)
                .safeBuild();
    }

    public static PIDTuner forPathPlanner(MutablePIDConstants constants, String controllerName) {
        return PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(v -> constants.kP = v)
                .withI(v -> constants.kI = v)
                .withD(v -> constants.kD = v)
                .withIZone(v -> constants.iZone = v)
                .safeBuild();
    }

    private static PIDTunerBuilder forMoTrapezoidController(
            MoTrapezoidController<?, ?, ?> controller, String controllerName) {
        return PIDTuner.builder(controllerName)
                .withDataStoreFile(Constants.DATA_STORE_FILE)
                .withP(controller::setP)
                .withI(controller::setI)
                .withD(controller::setD)
                .withIZone(controller::setIZone)
                .withProperty("dFilter", controller::setDFilter)
                .withProperty("iMaxAccum", controller::setIMaxAccum)
                .withProperty("maxVelocity", controller::setMaxVelocity)
                .withProperty("maxAcceleration", controller::setMaxAcceleration)
                .withProperty("errorZone", controller::setErrorZone)
                .withSetpoint(controller::getLastReference)
                .withMeasurement(controller::getLastMeasurement)
                .withStateValue("calculated_ff", controller::getLastFF);
    }

    public static PIDTuner forMoTrapezoidElevator(MoTrapezoidElevatorController elevator, String controllerName) {
        return forMoTrapezoidController(elevator, controllerName)
                .withProperty("kS", elevator::setKS)
                .withProperty("kG", elevator::setKG)
                .withProperty("kV", elevator::setKV)
                .withProperty("kA", elevator::setKA)
                .safeBuild();
    }

    public static PIDTuner forMoTrapezoidArm(MoTrapezoidArmController arm, String controllerName) {
        return forMoTrapezoidController(arm, controllerName)
                .withProperty("kS", arm::setKS)
                .withProperty("kG", arm::setKG)
                .withProperty("kV", arm::setKV)
                .withProperty("kA", arm::setKA)
                .safeBuild();
    }
}
