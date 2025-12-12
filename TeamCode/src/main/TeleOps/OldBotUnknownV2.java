package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "OldBotUnknownV2", group = "TeleOp")
public class OldBotUnknownV2 extends LinearOpMode {

    // ===== Drive Motors =====
    private DcMotor fLeft, fRight, bLeft, bRight;

    // ===== Viper Slide Motors =====
    private DcMotor rightVS, leftVS;

    // ===== Servos =====
    private Servo leftExt, rightExt, intakeClaw, rIntakeFlipper, lIntakeFlipper, intakeWrist;

    // ===== Color Sensor =====
    private ColorSensor colorSensor;

    // ===== Variables =====
    private double dampener = 1.0;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // ===== Viper Slide Presets =====
    private final int GROUND_POS = 0;
    private final int LOW_POS = 1500;
    private final int MID_POS = 2250;
    private final int HIGH_POS = 3500;
    private final double SLIDE_POWER = 0.75;

    // ===== Helper Methods =====
    private void openIntakeClaw() { intakeClaw.setPosition(0.18); }
    private void closeIntakeClaw() { intakeClaw.setPosition(0.04); }

    private void retractExtensionServos() {
        leftExt.setPosition(0.09);
        rightExt.setPosition(0.93);
    }

    private void extendExtensionServos() {
        leftExt.setPosition(0.32);
        rightExt.setPosition(0.7);
    }

    private void flipIntakeUp() {
        rIntakeFlipper.setPosition(0.5);
        lIntakeFlipper.setPosition(0.5);
    }

    private void flipIntakeDown() {
        rIntakeFlipper.setPosition(0.75);
        lIntakeFlipper.setPosition(0.25);
    }

    private void flipIntakePartialDown() {
        rIntakeFlipper.setPosition(0.65);
        lIntakeFlipper.setPosition(0.35);
    }

    // ----- VIPER CONTROL -----
    private void moveViperTo(int targetPosition) {
        // Set target position for both motors
        leftVS.setTargetPosition(targetPosition);
        rightVS.setTargetPosition(targetPosition);

        // Switch both to RUN_TO_POSITION
        leftVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power
        leftVS.setPower(SLIDE_POWER);
        rightVS.setPower(SLIDE_POWER);
    }

    @Override
    public void runOpMode() {
        // ===== Hardware Mapping =====
        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        bRight = hardwareMap.get(DcMotor.class, "bRight");

        rightVS = hardwareMap.get(DcMotor.class, "rightVS");
        leftVS = hardwareMap.get(DcMotor.class, "leftVS");

        leftExt = hardwareMap.get(Servo.class, "leftExt");
        rightExt = hardwareMap.get(Servo.class, "rightExt");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        rIntakeFlipper = hardwareMap.get(Servo.class, "rIntakeFlipper");
        lIntakeFlipper = hardwareMap.get(Servo.class, "lIntakeFlipper");
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // ===== Motor Directions =====
        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rightVS.setDirection(DcMotorSimple.Direction.REVERSE);

        // ===== Zero Power Behavior =====
        DcMotor.ZeroPowerBehavior BRAKE = DcMotor.ZeroPowerBehavior.BRAKE;
        fLeft.setZeroPowerBehavior(BRAKE);
        fRight.setZeroPowerBehavior(BRAKE);
        bLeft.setZeroPowerBehavior(BRAKE);
        bRight.setZeroPowerBehavior(BRAKE);
        leftVS.setZeroPowerBehavior(BRAKE);
        rightVS.setZeroPowerBehavior(BRAKE);

        // ===== Reset & Initialize Encoders =====
        leftVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== Init Servos =====
        retractExtensionServos();
        flipIntakeUp();
        openIntakeClaw();
        intakeWrist.setPosition(0.5);

        telemetry.addLine("✅ Initialized — Waiting for Start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== Drive Control =====
            double xPower = gamepad1.left_stick_x;
            double yPower = -gamepad1.left_stick_y;
            double zPower = gamepad1.right_stick_x;

            dampener = gamepad1.right_bumper ? 0.4 : 1.0;

            fLeft.setPower(dampener * Range.clip(yPower + xPower + zPower, -0.6, 0.6));
            fRight.setPower(dampener * Range.clip(yPower - xPower - zPower, -0.6, 0.6));
            bLeft.setPower(dampener * Range.clip(yPower - xPower + zPower, -0.6, 0.6));
            bRight.setPower(dampener * Range.clip(yPower + xPower - zPower, -0.6, 0.6));

            // ===== Viper Manual Override =====
            double viperPower = -gamepad2.left_stick_y;
            if (Math.abs(viperPower) > 0.05) {
                leftVS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightVS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftVS.setPower(viperPower * SLIDE_POWER);
                rightVS.setPower(viperPower * SLIDE_POWER);
            }

            // ===== Viper Presets =====
            // ===== Viper Slides Presets (Encoder-Controlled) =====
            if (gamepad1.a) {
                // Lowest position
                leftVS.setTargetPosition(0);
                rightVS.setTargetPosition(0);
                leftVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVS.setPower(0.8);
                rightVS.setPower(0.8);
            }

            if (gamepad1.y) {
                // Mid position
                leftVS.setTargetPosition(-1000);
                rightVS.setTargetPosition(-1000);
                leftVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVS.setPower(0.8);
                rightVS.setPower(0.8);
            }

            if (gamepad1.x) {
                // High position
                leftVS.setTargetPosition(-2000);
                rightVS.setTargetPosition(-2000);
                leftVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightVS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVS.setPower(0.8);
                rightVS.setPower(0.8);
            }

            // ===== Extension Servos =====
            if (gamepad2.a) extendExtensionServos();
            if (gamepad2.b) retractExtensionServos();

            // ===== Flipper Control =====
            if (gamepad2.y) flipIntakeUp();
            if (gamepad2.x) flipIntakeDown();

            // ===== Claw Toggle =====
            boolean leftBumperPressed = gamepad2.dpadLeftWasPressed();
            boolean rightBumperPressed = gamepad2.dpadRightWasPressed();
            if (leftBumperPressed && !lastLeftBumper) closeIntakeClaw();
            if (rightBumperPressed && !lastRightBumper) openIntakeClaw();
            lastLeftBumper = leftBumperPressed;
            lastRightBumper = rightBumperPressed;

            // ===== Color Sensor =====
            telemetry.clear();

            int rRaw = colorSensor.red();
            int gRaw = colorSensor.green();
            int bRaw = colorSensor.blue();

            int total = rRaw + gRaw + bRaw;
            String detectedColor = "None";

// Only detect if bright enough
            if (total > 50) {
                // Normalize
                double r = rRaw / (double)total;
                double g = gRaw / (double)total;
                double b = bRaw / (double)total;

                // Define color targets (normalized)
                double[][] targets = {
                        {1.0, 0.0, 0.0}, // Red
                        {0.0, 1.0, 0.0}, // Green
                        {0.0, 0.0, 1.0}, // Blue
                        {0.5, 0.0, 0.5}  // Purple
                };
                String[] names = {"Red", "Green", "Blue", "Purple"};

                double minDistance = Double.MAX_VALUE;
                int bestIndex = -1;

                for (int i = 0; i < targets.length; i++) {
                    double dr = r - targets[i][0];
                    double dg = g - targets[i][1];
                    double db = b - targets[i][2];
                    double distance = Math.sqrt(dr*dr + dg*dg + db*db);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestIndex = i;
                    }
                }

                // Only pick a color if close enough
                if (minDistance < 0.5) {
                    detectedColor = names[bestIndex];
                } else {
                    detectedColor = "Unknown";
                }
            }

// Output to telemetry (only latest reading)
            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("R,G,B", "%d, %d, %d", rRaw, gRaw, bRaw);
            telemetry.update();
        }
    }
}

