package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoTunePIDFShooter {
    private double kp = 0.0;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0;

    public double setpoint = 0.0;
    private double previousError = 0.0;
    private double integral = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0.0;

    private boolean isTuning = false;
    private double relayAmplitude = 0.0;
    private double outputHigh;
    private double outputLow;
    private double ultimateGain = 0.0;
    private double oscillationPeriod = 0.0;
    private int cycleCount = 0;
    private final int minCycles = 3;
    private double lastOutputTime = 0.0;
    private double lastErrorSignChangeTime = 0.0;
    private boolean lastOutputHigh = false;
    private double totalPeriod = 0.0;
    private int signChangeCount = 0;
    private double minError = Double.MAX_VALUE;
    private double maxError = Double.MIN_VALUE;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    private double nominalVoltage = 12.0;
    private double batteryVoltage = 12.0;

    public AutoTunePIDFShooter() {
        timer.reset();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void startAutoTune(double relayAmplitude) {
        this.relayAmplitude = relayAmplitude;
        this.outputHigh = relayAmplitude;
        this.outputLow = -relayAmplitude;
        this.isTuning = true;
        this.cycleCount = 0;
        this.totalPeriod = 0.0;
        this.signChangeCount = 0;
        this.integral = 0.0;
        this.previousError = 0.0;
        this.lastTime = timer.seconds();
        this.lastOutputTime = lastTime;
        this.lastErrorSignChangeTime = lastTime;
        this.lastOutputHigh = true;
        this.minError = Double.MAX_VALUE;
        this.maxError = Double.MIN_VALUE;
    }

    public boolean isTuningComplete() {
        return !isTuning;
    }

    public double update(double currentValue, double currentBatteryVoltage) {
        this.batteryVoltage = currentBatteryVoltage;
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        if (dt <= 0) return 0.0;

        double error = setpoint - currentValue;

        double output;
        if (isTuning) {
            output = lastOutputHigh ? outputHigh : outputLow;

            minError = Math.min(minError, error);
            maxError = Math.max(maxError, error);

            if (Math.signum(error) != Math.signum(previousError) && previousError != 0) {
                double period = (currentTime - lastErrorSignChangeTime) * 2;
                lastErrorSignChangeTime = currentTime;
                signChangeCount++;
                if (signChangeCount % 2 == 0) {
                    totalPeriod += period;
                    cycleCount++;
                }
                lastOutputHigh = !lastOutputHigh;
                lastOutputTime = currentTime;
            }

            if (cycleCount >= minCycles) {
                oscillationPeriod = totalPeriod / cycleCount;
                double amplitude = (maxError - minError) / 2;
                ultimateGain = 4.0 * relayAmplitude / (Math.PI * amplitude);
                applyZieglerNichols();
                isTuning = false;
            }
        } else {
            integral += error * dt;
            double derivative = (error - previousError) / dt;
            output = kp * error + ki * integral + kd * derivative + kf * setpoint;
        }

        previousError = error;
        lastTime = currentTime;

        double compensatedOutput = output * (nominalVoltage / batteryVoltage);
        return clamp(compensatedOutput);
    }

    private void applyZieglerNichols() {
        kp = 0.6 * ultimateGain;
        ki = 2 * kp / oscillationPeriod;
        kd = kp * oscillationPeriod / 8;
    }

    private double clamp(double value) {
        return Math.max(minOutput, Math.min(maxOutput, value));
    }

    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }
    public double getKf() { return kf; }

    public void setGains(double kp, double ki, double kd, double kf) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }
}
