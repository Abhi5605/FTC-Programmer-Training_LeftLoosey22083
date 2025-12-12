package org.firstinspires.ftc.teamcode.HardwareMapping;

//Square Root Damped Controller
public class SQUIDController {
    private double kSQ;
    public SQUIDController(double kSQ) {
        this.kSQ = kSQ;
    }

    /**
     * @param error The error of the system (usually target - measured value)
     * @return The computed output power calculated based on the sqrt of the error
     */
    public double calculate(double error) {
        return sqrComponent(error);
    }

    private double sqrComponent(double error) {
        return Math.sqrt(Math.abs(error * kSQ)) * Math.signum(error);
    }

    public void setConstants(double kSQ) {
        this.kSQ = kSQ;
    }
}