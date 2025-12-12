package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMapping.AutoTunePIDFShooter;

@TeleOp(name = "ShooterPIDAutoTune", group = "Testing")
public class ShooterPIDAutoTune extends LinearOpMode {

    private DcMotorEx ShotM;
    //private DcMotorEx flywheelRight;
    private VoltageSensor voltageSensor;

    private AutoTunePIDFShooter pidf;

    private final double TICKS_PER_REV = 537.7;
    private final double TARGET_VELOCITY = -1475;
    private final double RELAY_AMPLITUDE = 0.25;

    private boolean isAutotuning = false;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        ShotM = hardwareMap.get(DcMotorEx.class, "ShotM");
        //flywheelRight = hardwareMap.get(DcMotorEx.class, "flywheelRight");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        //ShotM.setDirection(DcMotorSimple.Direction.REVERSE);
        //flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShotM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidf = new AutoTunePIDFShooter();
        pidf.setNominalVoltage(12.0);
        pidf.setOutputLimits(-1.0, 1.0);

        telemetry.addData("Status", "Initialized. Press PLAY to start.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double batteryVoltage = voltageSensor.getVoltage();
            double currentVelocity = (ShotM.getVelocity());

            if (gamepad1.y && !isAutotuning) {
                pidf.startAutoTune(RELAY_AMPLITUDE);
                isAutotuning = true;
                telemetry.addData("Status", "Autotuning started...");
            } else if (gamepad1.a) {
                pidf.setSetpoint(TARGET_VELOCITY);
                telemetry.addData("Status", "Targeting velocity: " + TARGET_VELOCITY);
            }

            double power = pidf.update(currentVelocity, batteryVoltage);
            ShotM.setPower(power);
            //flywheelRight.setPower(power);

            if (isAutotuning && pidf.isTuningComplete()) {
                isAutotuning = false;
                double kp = pidf.getKp();
                double ki = pidf.getKi();
                double kd = pidf.getKd();
                telemetry.addData("Tuning Complete", "Kp: %.4f, Ki: %.4f, Kd: %.4f", kp, ki, kd);
            }

            telemetry.addData("Current Velocity", "%.2f ticks/sec", currentVelocity);
            telemetry.addData("Setpoint", "%.2f", pidf.setpoint);
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addData("Status", isAutotuning ? "Autotuning..." : "Running");
            telemetry.update();
        }

        //flywheelLeft.setPower(0);
        ShotM.setPower(0);
    }
}