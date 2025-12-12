package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

    @TeleOp(name = "ServoTesting", group = "TeleOp")
    public class ServoTesting extends LinearOpMode {

        // ===== Drive Motors =====
        private Servo S;

        @Override
        public void runOpMode() {
            // ===== Hardware Mapping =====
            S = hardwareMap.get(Servo.class, "S");


            telemetry.addLine("✅ Initialized — Waiting for Start...");
            telemetry.update();

            //TransSBL.setPosition(0.0);
            S.setPosition(0.0);
            //TransSBR.setPosition(0.0);
            //TransSFR.setPosition(0.0);

            waitForStart();

            while (opModeIsActive()) {
                if(gamepad1.a) {
                    S.setPosition(0.0);
                }
                if(gamepad1.b) {
                    S.setPosition(0.3);
                }
                if(gamepad1.x) {
                    S.setPosition(0.7);
                }
                if(gamepad1.y) {
                    S.setPosition(1.0);
                }


            }
        }
    }
