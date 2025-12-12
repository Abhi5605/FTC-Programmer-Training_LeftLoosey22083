package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareMapping.HardwareMap1;

import java.util.List;

    @TeleOp(name = "RevTestPos", group = "TeleOp")
    public class RevTestPos extends LinearOpMode {

        private DcMotorEx RevM;
        private Servo BallLiftS;
        private boolean rightTriggerReleased = true;
        private boolean leftTriggerReleased = true;

        // ============================================================
        // REVOLVER POSITIONS
        // ============================================================
        private static final double TPR = 288;
        private final int[] revPositions = {0, 120, 240};
        private int revIndex = 0;

        private double getMotorAngle(DcMotorEx m, double ticksPerRev) {
            double angle = (m.getCurrentPosition() % ticksPerRev) * (360.0 / ticksPerRev);
            return (angle < 0) ? angle + 360 : angle;
        }

        private void RevPos(int targetAngle) {
            double currentAngle = getMotorAngle(RevM, TPR);
            double delta = targetAngle - currentAngle;

            if (delta > 180) delta -= 360;
            else if (delta < -180) delta += 360;

            int deltaTicks = (int) ((delta / 360.0) * TPR);
            int newTarget = RevM.getCurrentPosition() + deltaTicks;

            RevM.setTargetPosition(newTarget);
            RevM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RevM.setPower(0.25);

            while (RevM.isBusy() && opModeIsActive()) {
            }

            RevM.setPower(0);
            RevM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        @Override
        public void runOpMode() {
            RevM = hardwareMap.get(DcMotorEx.class, "RevM");
            BallLiftS = hardwareMap.get(Servo.class, "BallLiftS");
            RevM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            RevM.setPower(0);
            telemetry.addLine("Initialized");
//            telemetry.update();
//            BallLiftS.setPosition(1.0);

            waitForStart();
            // Reset encoder ONLY once start is pressed
            RevM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RevM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addLine("Encoder initialized at Slot 0 (0°).");
            telemetry.update();

            BallLiftS.setPosition(1.0);
            while (opModeIsActive()) {

                if (gamepad1.right_trigger > 0.3 && rightTriggerReleased) {
                    revIndex = (revIndex + 1) % revPositions.length;
                    RevPos(revPositions[revIndex]);
                    rightTriggerReleased = false;
                } else if (gamepad1.right_trigger < 0.5) {
                    rightTriggerReleased = true;
                }

                if (gamepad1.left_trigger > 0.3 && leftTriggerReleased) {
                    revIndex = (revIndex - 1 + revPositions.length) % revPositions.length;
                    RevPos(revPositions[revIndex]);
                    leftTriggerReleased = false;
                } else if (gamepad1.left_trigger < 0.5) {
                    leftTriggerReleased = true;
                }

                // Manual debug power buttons
                if (gamepad1.a) {
                    RevPos(0);
                }
                if (gamepad1.b) {
                    RevPos(120);
                }
//                if (gamepad1.y) {
//                    RevM.setPower(0.2);
//                }
                if (gamepad1.x) {
                    RevPos(240);
                }

                if (gamepad1.dpad_down) BallLiftS.setPosition(1.0);
                if (gamepad1.dpad_up) BallLiftS.setPosition(0.7);

//                // ===========================
//                // IMPORTANT FIX:
//                // Only apply stick control if we are NOT in RUN_TO_POSITION
//                // ===========================
//                if (RevM.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//                    double RevSpeed = -gamepad1.left_stick_y;
//                    RevM.setPower(RevSpeed);
//                }

                telemetry.addData("Target Index", revIndex);
                telemetry.addData("Target Angle", revPositions[revIndex]);
                telemetry.addData("Current Angle", getMotorAngle(RevM, TPR));
                telemetry.update();
            }

        }
    }