package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareMapping.SQUIDController;

@TeleOp
public class TuneSquID extends OpMode {
    private SQUIDController controller;
    private DcMotor motor;
    private double target = 0;
    private double gain = 0.01;

    @Override
    public void init() {
        controller = new SQUIDController(gain);
        motor = hardwareMap.get(DcMotor.class, "motor name");
    }

    @Override
    public void loop() {
        if (gamepad1.a) target = 250;
        if (gamepad1.b) target = 0;
        double error = target - motor.getCurrentPosition();
        motor.setPower(controller.calculate(error));
        telemetry.addData("Current Error:", target - motor.getCurrentPosition());
        telemetry.addData("Gain: ", gain);
        if (Math.abs(error) > 500) telemetry.addLine("Direction of motor might need to be reversed!");
        gain += gamepad1.right_stick_y * 0.2; // Adjust gain by moving the right stick.
    }
}