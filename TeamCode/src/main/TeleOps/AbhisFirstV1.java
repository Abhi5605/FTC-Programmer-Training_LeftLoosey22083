//package org.firstinspires.ftc.teamcode.TeleOps;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.HardwareMapping.HardwareMap1;
//
//@TeleOp(name = "AbhisFirstV1", group = "TeleOp")
//public class AbhisFirstV1 extends LinearOpMode {
//
//    private final HardwareMap1 robot = new HardwareMap1();
//    private IMU imu;
//    private boolean isSlowMode = false;
//    private boolean isIntakeOn = false;
//    private boolean lastA = false;
//    boolean rightTriggerReleased = true;
//    boolean leftTriggerReleased = true;
//    private boolean isServoAtZero = true;
//
//    private boolean toggle(boolean state, boolean button, boolean[] latch) {
//        if (button && !latch[0]) {
//            state = !state;
//        }
//        latch[0] = button;
//        return state;
//    }
//
//    int[] revPositions = {0, 120, 240};
//    int revIndex = 0;
//
//
//    /// METHODS WILL GO HERE
//    /// Fix the:TPR
//    /// METHODS WILL GO HERE
//    /// TPR constant (output shaft PPR)
//    private static final double TPR = 1425.1;
//
//    public void intakeOn() {
//        robot.InM.setPower(1.0);
//    }
//
//    public void intakeOff() {
//        robot.InM.setPower(0);
//    }
//
//    public void intakePushOut() {
//        robot.InM.setPower(-0.5);
//    }
//
//    public void FastLuanch() {
//        robot.RevM.setPower(0.3);
//    }
//
//    public void RevStop() {
//        robot.RevM.setPower(0);
//    }
//
//    // Returns motor angle in 0–360 range
//    public double getMotorAngle(DcMotorEx RevM, double ticksPerRev) {
//        double angle = (RevM.getCurrentPosition() % ticksPerRev) * (360.0 / ticksPerRev);
//        if (angle < 0) angle += 360;
//        return angle;
//    }
//
//    // Moves RevM motor to target angle using shortest rotation path
//    public void RevPos(int targetAngle) {
//        // Get current angle (0–360)
//        double currentAngle = getMotorAngle(robot.RevM, TPR);
//
//        // Find smallest rotation difference
//        double delta = targetAngle - currentAngle;
//
//        // Wrap around so we always take the shortest path
//        if (delta > 180) {
//            delta -= 360;
//        } else if (delta < -180) {
//            delta += 360;
//        }
//
//        // Convert degrees to ticks
//        int deltaTicks = (int) ((delta / 360.0) * TPR);
//
//        // Compute new target encoder position
//        int newTarget = robot.RevM.getCurrentPosition() + deltaTicks;
//
//        // Command motor to move there
//        robot.RevM.setTargetPosition(newTarget);
//        robot.RevM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.RevM.setPower(0.5);
//
//        // Optional: wait until motor reaches the target
//        while (robot.RevM.isBusy() && robot.RevM.getPower() > 0) {
//            // Can add telemetry here if needed
//        }
//
//        // Stop motor and return to using encoder mode
//        robot.RevM.setPower(0);
//        robot.RevM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//    @Override
//    public void runOpMode() {
//        // Initialize hardware
//        robot.init(hardwareMap);
//
//
//        // Initialize IMU
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        );
//        imu.initialize(parameters);
//
//        Orientation angles = imu.getRobotOrientation(
//                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS
//        );
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // -------------------- Drive Control --------------------
//            double y = -gamepad1.left_stick_y;  // forward/backward
//            double x = gamepad1.left_stick_x * 1.1;  // strafe
//            double rx = -gamepad1.right_stick_x;  // rotation
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            // Toggle slow mode
//            if (gamepad1.left_bumper) isSlowMode = true;
//            else if (gamepad1.right_bumper) isSlowMode = false;
//
//            if (isSlowMode) {
//                frontLeftPower *= 0.32;
//                backLeftPower *= 0.32;
//                frontRightPower *= 0.32;
//                backRightPower *= 0.32;
//            }
//
//            robot.FLW.setPower(Range.clip(frontLeftPower, -1.0, 1.0));
//            robot.BLW.setPower(Range.clip(backLeftPower, -1.0, 1.0));
//            robot.FRW.setPower(Range.clip(frontRightPower, -1.0, 1.0));
//            robot.BRW.setPower(Range.clip(backRightPower, -1.0, 1.0));
//
//
//            // Move forward through positions with right trigger
//            if (gamepad1.right_trigger > 0.5 && rightTriggerReleased) {
//                revIndex = (revIndex + 1) % revPositions.length;
//                RevPos(revPositions[revIndex]);
//                rightTriggerReleased = false;
//            }
//            if (gamepad1.right_trigger < 0.5) {
//                rightTriggerReleased = true;
//            }
//
//// Move backward through positions with left trigger
//            if (gamepad1.left_trigger > 0.5 && leftTriggerReleased) {
//                revIndex = (revIndex - 1 + revPositions.length) % revPositions.length;
//                RevPos(revPositions[revIndex]);
//                leftTriggerReleased = false;
//            }
//            if (gamepad1.left_trigger < 0.5) {
//                leftTriggerReleased = true;
//            }
//
//            if (gamepad1.dpad_up) {
//                robot.ShotM.setPower(1.0);
//                sleep(1500);
//                FastLuanch();
//
//            }
//
//            if (gamepad1.dpad_down) {
//                RevStop();
//            }
//
//            //Toggle Intake?
//            if (gamepad1.a) {
//                if (isIntakeOn) {
//                    robot.InM.setPower(1.0);
//                } else {
//                    robot.InM.setPower(0);
//                }
//            }
//            isIntakeOn = !isIntakeOn;
//
//            if (gamepad1.dpad_left) {  // or any button you want
//                if (isServoAtZero) {
//                    robot.TransSBL.setPosition(0.7);
//                    robot.TransSFL.setPosition(0.7);
//                    robot.TransSBR.setPosition(0.7);
//                    robot.TransSFR.setPosition(0.7);
//                } else {
//                    robot.TransSBL.setPosition(0.0);
//                    robot.TransSFL.setPosition(0.0);
//                    robot.TransSBR.setPosition(0.0);
//                    robot.TransSFR.setPosition(0.0);
//                }
//
//                isServoAtZero = !isServoAtZero; // flip the state
//                sleep(50); // small debounce (optional but helps)
//// optional info on driver hub
//                //telemetry.addData("Revolver Position Index", revIndex);
//                //telemetry.addData("Revolver Target Angle", revPositions[revIndex]);
//
//
//            }
//        }
//    }
//}
//
