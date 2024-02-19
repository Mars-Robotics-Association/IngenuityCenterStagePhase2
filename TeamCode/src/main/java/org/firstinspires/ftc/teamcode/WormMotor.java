package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WormMotor extends Motor {
    private final PIDController wormPositionController;
    private RunMode wormRunMode;
    private boolean wormTargetIsSet;
    private TelemetryPacket packet;
    private String stepName;

    protected WormMotor() {
        wormPositionController = null;
    }

    public WormMotor(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
        wormPositionController = new PIDController(0.001, 0.15, 0);
    }

    @Override
    public void setRunMode(RunMode runmode) {
        this.wormRunMode = runmode;
        super.setRunMode(runmode == RunMode.PositionControl ? RunMode.RawPower : runmode);
        if (runmode == RunMode.PositionControl && !wormTargetIsSet) {
            setTargetPosition(getCurrentPosition());
            wormTargetIsSet = false;
        }
    }

    @Override
    public double getPositionCoefficient() {
        return wormPositionController.getP();
    }

    public double getPositionIntegralCoefficient() {
        return wormPositionController.getI();
    }

    @Override
    public void setPositionCoefficient(double kp) {
        wormPositionController.setP(kp);
    }

    public void setPositionIntegralCoefficient(double ki) {
        wormPositionController.setI(ki);
    }

    public void setPositionPI(double kp, double ki) {
        wormPositionController.setPIDF(kp, ki, 0, 0);
    }

    @Override
    public void setTargetPosition(int target) {
        this.setTargetDistance(target);
    }

    @Override
    public void setTargetDistance(double target) {
        wormTargetIsSet = true;
        wormPositionController.setSetPoint(target);
    }

    @Override
    public void setPositionTolerance(double tolerance) {
        wormPositionController.setTolerance(tolerance);
    }

    public void setTelemetry(TelemetryPacket packet, String stepName) {
        this.packet = packet;
        this.stepName = stepName;
        this.packet.put(stepName + "testing", true);
    }

    @Override
    public void set(double output) {
        if (packet != null) {
            packet.put(stepName + "mode", wormRunMode);
        }
        if (wormRunMode == RunMode.VelocityControl) {
            super.set(output);
        } else if (wormRunMode == RunMode.PositionControl) {
            double error = calcPosError(getDistance());
            double power = output * error;
            if (packet != null) {
                packet.put(stepName + "calc err", error);
                packet.put(stepName + "power", power);
            }
            this.setRaw(power);
        } else {
            super.set(output);
        }
    }

    private void setRaw(double output) {
        super.set(output);
    }

    private double calcPosError(double distance) {
        return wormPositionController.calculate(distance);
    }

    @Override
    public boolean atTargetPosition() {
        return wormPositionController.atSetPoint();
    }
}
