package org.firstinspires.ftc.teamcode.TeleOps;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "ChassisAndTransCode", group = "TeleOp")
public class ChassisAndTransCode extends LinearOpMode {

    // ===== Drive Motors =====
    ElapsedTime pidTimer = new ElapsedTime();
    private DcMotor FLW, BLW, FRW, BRW, InM;
    private Servo TransSFL, TransSFR, TransSBL, TransSBR;
    //public Servo InPWM;
    private Limelight3A limelight;
    private DcMotorEx shotMotor;
    private Servo PitchS;
    private DcMotorEx ShotRotateM;
    private IMU imu;

    // Toggle
    private boolean applySolution = false;

    // ===== Constants =====
    private final double MIN_DIST_IN = 35.0;
    private final double MAX_DIST_IN = 150.0;

    private final double MIN_POWER = 0.6;
    private final double MAX_POWER = 1.0;

    private final double MAX_ANGLE_DEG = 22.5;
    private final double MIN_ANGLE_DEG = 40.0;

    private final double SERVO_MIN_POS = 1.0;
    private final double SERVO_MAX_POS = 0.0;

    final double SHOT_TPR = 537.7;
    final double MAX_ROT_POWER = 0.5;
    final double kP_rot = 0.025;
    final double deadband = 0.5;

    private boolean isServoAtZero = true;
    private boolean isMotorSpinning = false;
    private boolean isSlowMode = false;

    // PID constants — TUNE THESE BASED ON YOUR MOTOR
    private final double kP_shooter = 0.0015;
    private final double kI_shooter = 0.0002;
    private final double kD_shooter = 0.0001;

    // PID internal state
    private double shooterIntegral = 0.0;
    private double lastShooterError = 0.0;
    private double updateShooterPID(DcMotorEx motor, double targetPower, double dt) {

        final double MAX_TICKS_PER_SEC = 53770.0;
        double targetVelocity = targetPower * MAX_TICKS_PER_SEC;
        double currentVelocity = motor.getVelocity();

        double error = targetVelocity - currentVelocity;

        // Integral
        shooterIntegral += error * dt;
        // Derivative
        double derivative = (error - lastShooterError) / Math.max(dt, 1e-6);
        lastShooterError = error;
        // PID (pure PID, does NOT add the target directly)
        double pid = (kP_shooter * error)
                + (kI_shooter * shooterIntegral)
                + (kD_shooter * derivative);
        // Convert PID ticks/sec → fractional power
        double correction = pid / MAX_TICKS_PER_SEC;

        // Base power to hit target velocity
        double basePower = targetVelocity / MAX_TICKS_PER_SEC;

        double newPower = basePower + correction;
        newPower = Range.clip(newPower, MIN_POWER, MAX_POWER);

        telemetry.addData("ShooterPID","TGT:%.0f CUR:%.0f PWR:%.3f ERR:%.0f",
                targetVelocity, currentVelocity, newPower, error);
        return newPower;
    }

    // ShotRotateM limits
    private int centerTicks;
    private int minTicks;
    private int maxTicks;

    private double angleToServo(double angleDeg) {
        double n = (angleDeg - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double pos = SERVO_MIN_POS + n * (SERVO_MAX_POS - SERVO_MIN_POS);
        return Range.clip(pos, SERVO_MAX_POS, SERVO_MIN_POS);
    }

    @Override
    public void runOpMode() {

        // ===== Hardware Mapping =====
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        InM = hardwareMap.get(DcMotor.class, "InM");
        //InPWM = hardwareMap.get(Servo.class, "InPWM");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shotMotor = hardwareMap.get(DcMotorEx.class, "ShotM");
        PitchS = hardwareMap.get(Servo.class, "PitchS");
        ShotRotateM = hardwareMap.get(DcMotorEx.class, "ShotRotateM");
        imu = hardwareMap.get(IMU.class, "imu");

        TransSBL = hardwareMap.get(Servo.class, "TransSBL");
        TransSFL = hardwareMap.get(Servo.class, "TransSFL");
        TransSBR = hardwareMap.get(Servo.class, "TransSBR");
        TransSFR = hardwareMap.get(Servo.class, "TransSFR");

        limelight.setPollRateHz(75);
        limelight.pipelineSwitch(0);
        limelight.start();

        shotMotor.setDirection(DcMotor.Direction.REVERSE);
        shotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shotMotor.setPower(0.0);

        ShotRotateM.setDirection(DcMotor.Direction.REVERSE);


//        InM.setZeroPowerBehavior(BRAKE);
//        InM.setDirection(DcMotor.Direction.REVERSE);

        // ===== Motor Directions =====
        FLW.setDirection(DcMotorSimple.Direction.REVERSE);
        BLW.setDirection(DcMotorSimple.Direction.REVERSE);
        FRW.setDirection(DcMotorSimple.Direction.FORWARD);
        BRW.setDirection(DcMotorSimple.Direction.FORWARD);

        // ===== Zero Power Behavior =====
        FLW.setZeroPowerBehavior(BRAKE);
        FRW.setZeroPowerBehavior(BRAKE);
        BLW.setZeroPowerBehavior(BRAKE);
        BRW.setZeroPowerBehavior(BRAKE);

        telemetry.addLine("Initialized — Waiting for Start...");
        telemetry.update();

        TransSBL.setPosition(1.0);
        TransSFL.setPosition(1.0);
        TransSBR.setPosition(0.0);
        TransSFR.setPosition(0.0);


        PitchS.setPosition(1.0);

        // ===== ShotRotateM center & limits (fixed) =====

        centerTicks = ShotRotateM.getCurrentPosition();
        int ninetyDegTicks = (int) ((300.0 / 360.0) * SHOT_TPR); ///This is very changed
        minTicks = centerTicks - ninetyDegTicks;
        maxTicks = centerTicks + ninetyDegTicks;

        // ===== Limelight fallback tracking variables =====
        double lastX = 0, lastY = 0, lastZ = 0;
        pidTimer.reset();

        waitForStart();

        while (opModeIsActive()) {
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // ===== Drive Control =====
            double xPower = gamepad1.left_stick_x;
            double yPower = -gamepad1.left_stick_y;
            double zPower = gamepad1.right_stick_x;

            if (gamepad1.left_bumper) isSlowMode = true;
            if (gamepad1.right_bumper) isSlowMode = false;
            double Slow = isSlowMode ? 0.32 : 1.0;

            FLW.setPower(Range.clip(yPower + xPower + zPower, -1.0, 1.0) * Slow);
            FRW.setPower(Range.clip(yPower - xPower - zPower, -1.0, 1.0) * Slow);
            BLW.setPower(Range.clip(yPower - xPower + zPower, -1.0, 1.0) * Slow);
            BRW.setPower(Range.clip(yPower + xPower - zPower, -1.0, 1.0) * Slow);

            // ===== Transfer System Toggle =====
            if (gamepad1.dpad_up) {
                if (isServoAtZero) {
                    TransSBL.setPosition(0.2);
                    TransSFL.setPosition(0.2);
                    TransSBR.setPosition(0.8);
                    TransSFR.setPosition(0.8);
                } else {
                    TransSBL.setPosition(1.0);
                    TransSFL.setPosition(1.0);
                    TransSBR.setPosition(0.0);
                    TransSFR.setPosition(0.0);
                    isServoAtZero = !isServoAtZero;
                    sleep(350);
                    isMotorSpinning = !isMotorSpinning;
                }
            }
            // ===== Intake Control =====
            if (gamepad1.x) {
                InM.setPower(1.0);
            }
            //InM.setPower(0.7);
            if (gamepad1.y) {
                InM.setPower(0.0);
            }

            // ===== LIMELIGHT TRACKING =====
            LLResult result = limelight.getLatestResult();
            double tx = 0;
            boolean foundTag20 = true;

            double robotX = lastX, robotY = lastY, robotZ = lastZ;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getFiducialId() == 20) {
                            foundTag20 = true;
                            tx = f.getTargetXDegrees();

                            Pose3D p = f.getRobotPoseTargetSpace();
                            if (p != null) {
                                robotX = p.getPosition().x;
                                robotY = p.getPosition().y;
                                robotZ = p.getPosition().z;

                                lastX = robotX;
                                lastY = robotY;
                                lastZ = robotZ;
                            }
                        }
                    }
                }
            }
// ===== Auto-align ShotRotateM =====
                    // ===== AUTO CENTER ROTATOR (UNCHANGED) =====
            int currentTicks = ShotRotateM.getCurrentPosition();
            double steerPower = 0;

            if (foundTag20) {
                steerPower = -tx * kP_rot;
                steerPower = Range.clip(steerPower, -MAX_ROT_POWER, MAX_ROT_POWER);

                // Only stop motion that goes past the min/max
                if (currentTicks <= minTicks && steerPower < 0) {
                    steerPower = 0; // trying to go below min
                } else if (currentTicks >= maxTicks && steerPower > 0) {
                    steerPower = 0; // trying to go above max
                }

                // If the error is within deadband, stop
                if (Math.abs(tx) < deadband) {
                    steerPower = 0;
                }
            }
            ShotRotateM.setPower(steerPower);

            double horizDistM = Math.hypot(robotX, robotZ);
            double horizDistIn = horizDistM / 0.0254;

            double distClamped = Range.clip(horizDistIn, MIN_DIST_IN, MAX_DIST_IN);

            double normalized = (distClamped - MIN_DIST_IN) / (MAX_DIST_IN - MIN_DIST_IN);

            double targetPower = MIN_POWER + normalized * (MAX_POWER - MIN_POWER);

            double targetAngleDeg = MIN_ANGLE_DEG + normalized * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

            double servoPos =
                    SERVO_MIN_POS + normalized * (SERVO_MAX_POS - SERVO_MIN_POS);

            servoPos = Range.clip(servoPos, SERVO_MAX_POS, SERVO_MIN_POS);

// helps for PID: clear integrator when PID not active
            if (!applySolution) {
                shooterIntegral = 0;
                lastShooterError = 0;
            }

            // === PID CONTROLLED SHOOTER POWER ===
            double newPower = updateShooterPID(shotMotor, targetPower, dt);
            //shotMotor.setPower(newPower); // PID output applied


            // ===== Apply/Remove Shot Solution =====
                    if (gamepad1.a) applySolution = true;
                    if (gamepad1.b) {
                        applySolution = false;
                        shotMotor.setPower(0);
                        FLW.setZeroPowerBehavior(BRAKE);
                        FRW.setZeroPowerBehavior(BRAKE);
                        BLW.setZeroPowerBehavior(BRAKE);
                        BRW.setZeroPowerBehavior(BRAKE);
                    }

                    if (applySolution) {
                        PitchS.setPosition(servoPos);
                        shotMotor.setPower(newPower);
                        FLW.setZeroPowerBehavior(FLOAT);
                        FRW.setZeroPowerBehavior(FLOAT);
                        BLW.setZeroPowerBehavior(FLOAT);
                        BRW.setZeroPowerBehavior(FLOAT);
                        telemetry.addData("Applied", "YES");
                    } else {
                        telemetry.addData("Applied", "NO (showing only)");
                    }

                    telemetry.addData("Distance (in)", "%.1f", horizDistIn);
                    telemetry.addData("Target Power", "%.2f", targetPower);
            telemetry.addData("NEwPower", "%.2f", newPower);

            //telemetry.addData("Target Angle", "%.1f deg",);
                    telemetry.addData("Servo Pos", "%.3f", servoPos);
                    telemetry.addData("ShotRot Ticks", ShotRotateM.getCurrentPosition());
                    telemetry.addData("Found Tag20", foundTag20);
                    telemetry.addData("tx", tx);

                    telemetry.update();
                }
            }
        }