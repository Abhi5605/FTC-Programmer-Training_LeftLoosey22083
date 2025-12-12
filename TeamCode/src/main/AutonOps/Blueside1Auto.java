package org.firstinspires.ftc.teamcode.AutonOps;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareMapping.HardwareMap1;
import org.firstinspires.ftc.teamcode.HardwareMapping.ColorSensor;
import org.firstinspires.ftc.teamcode.TeleOps.BlueSideTeleOpV1;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "BackBlueworks?", group = "Autonomous")


public class Blueside1Auto extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(BlueSideTeleOpV1.class);

    private final HardwareMap1 robot = new HardwareMap1();
    private final ColorSensor colorSensors = new ColorSensor();

    private IMU imu;
    public Limelight3A limelight;

    // ===== Auto Shoot State Machine =====
    private enum AutoShootState {
        IDLE,
//        SPINUP,
//        MOVE_REVOLVER,
//        FIRE,
//        ADVANCE,
//        DONE
    }

    //private BlueSideTeleOpV1.AutoShootState autoState = BlueSideTeleOpV1.AutoShootState.IDLE;
    private BlueSideTeleOpV1.BallColor[] colorOrder = null;

    private int findSlotWithColor(BlueSideTeleOpV1.BallColor targetColor) {
        BlueSideTeleOpV1.BallColor[] slots = getRevolverColors();  // your existing function that reads 3 sensors
        for (int i = 0; i < 3; i++) {
            if (slots[i] == targetColor)
                return i;
        }
        // Not found:
        return -1;
    }

    // No new values added — reusing:
    // spinUpTime, ballPitchTimer, revIndex, etc.


    // ------------------
    // Shooter state
    // ------------------
    enum ShootState
    { IDLE, MOVE_TO_SLOT, FIRE, WAIT_POST_FIRE }
    //BlueSideTeleOpV1.ShootState shootState = BlueSideTeleOpV1.ShootState.IDLE;

    // Auto shoot modes
    private boolean autoColorShoot = false;
    private boolean autoSimpleShoot = false;

    private int autoStep = 0; // 0,1,2 for each ball
    private ElapsedTime autoTimer = new ElapsedTime();

    // Reuse existing revIndex, BP_FIRE_POS, BP_REST_POS, etc.

    int[] queuedSlots = new int[0];
    int shootStep = 0;

    ElapsedTime shootTimer = new ElapsedTime();
    final double SLOT_SETTLE_TIME = 0.20;   // 200ms
    final double FIRE_TIME = 0.20;          // 200ms

    // ------------------
    // Ball colors
    // ------------------
    public enum BallColor { GREEN, PURPLE }
    private BlueSideTeleOpV1.BallColor[] activePattern = null;

    // ------------------
    // Teleop variables
    // ------------------
    private boolean isSlowMode = false;
    //private boolean ShootOn = false;
    private boolean InTakeOn = true;
    private boolean LiftUp = false;
    private boolean Gamepad2x = false;
    private boolean rightStickButton = false;
    private boolean leftStickButton = false;

    // Edge-detection booleans
    private boolean prevRightTriggerPressed = false;
    private boolean prevLeftTriggerPressed = false;

    // Revolver
    private static final double RevTPR = 288;
    private final int[] revPositions = {0, 120, 240};
    private int revIndex = 0;
    private ElapsedTime rightTriggerTimer = new ElapsedTime();
    private ElapsedTime leftTriggerTimer = new ElapsedTime();
    private final double REV_TRIGGER_DEBOUNCE = 0.25;

    // Ball pitch
    private ElapsedTime ballPitchTimer = new ElapsedTime();
    private boolean ballPitchActive = false;
    private final double BALL_PITCH_DURATION = 0.15;

    // Intake toggle
    private ElapsedTime intakeToggleTimer = new ElapsedTime();
    private final double TOGGLE_DELAY = 0.5;

    // Lift toggle
    private ElapsedTime liftToggleTimer = new ElapsedTime();

    // Pattern-lock
    private int pendingPattern = -1;
    private int detectedPattern = -1;
    private ElapsedTime patternTimer = new ElapsedTime();

    // Shooter PID
    private final double MIN_DIST_IN = 25.0;
    private final double MAX_DIST_IN = 185.0;
    private final double MIN_POWER = 0.65;
    private final double MAX_POWER = 1.0;
    private final double MIN_ANGLE_DEG = 22.5;
    private final double MAX_ANGLE_DEG = 40.0;
    private final double SERVO_MIN_POS = 1.0;
    private final double SERVO_MAX_POS = 0.10;
    private final double kP_shooter = 0.0006;
    private final double kI_shooter = 0.0001;
    private final double kD_shooter = 0.0006;
    private double shooterIntegral = 0.0;
    private double lastShooterError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean applySolution = false;

    // Turret
    final double SHOT_TPR = 537.7;
    final double MAX_ROT_POWER = 0.22;
    final double kP_rot = 0.03;
    final double deadband = 0.5;
    private int centerTicks, minTicks, maxTicks;

    private int nextSlotSingleShot = 0;

    // ------------------
    // Utility functions
    // ------------------
    public double[] getNormalizedRGB(RevColorSensorV3 sensor) {
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();
        double total = r + g + b;
        if (total == 0) return new double[]{0,0,0};
        return new double[]{ r/total, g/total, b/total };
    }

    private BlueSideTeleOpV1.BallColor getColorFromSensor(RevColorSensorV3 sensor) {
        double[] rgb = getNormalizedRGB(sensor);
        double r = rgb[0], g = rgb[1], b = rgb[2];
        if (g > 0.44 && r < 0.235) return BlueSideTeleOpV1.BallColor.GREEN;
        if (r > 0.24 && b > 0.33) return BlueSideTeleOpV1.BallColor.PURPLE;
        return null;
    }

    private BlueSideTeleOpV1.BallColor[] getRevolverColors() {
        return new BlueSideTeleOpV1.BallColor[] {
                getColorFromSensor(colorSensors.ColS0),
                getColorFromSensor(colorSensors.ColS120),
                getColorFromSensor(colorSensors.ColS240)
        };
    }

    private int[] buildRevolverSteps() {
        if (activePattern == null) return new int[]{0,1,2};
        BlueSideTeleOpV1.BallColor[] colorsOnWheel = getRevolverColors();
        int[] steps = new int[3];
        boolean[] usedSlots = new boolean[3];

        for (int i = 0; i < 3; i++) {
            BlueSideTeleOpV1.BallColor needed = activePattern[i];
            int targetSlot = -1;
            for (int slot = 0; slot < 3; slot++) {
                if (!usedSlots[slot] && colorsOnWheel[slot] == needed) {
                    targetSlot = slot;
                    usedSlots[slot] = true;
                    break;
                }
            }
            if (targetSlot == -1) {
                for (int slot = 0; slot < 3; slot++) {
                    if (!usedSlots[slot]) {
                        targetSlot = slot;
                        usedSlots[slot] = true;
                        break;
                    }
                }
            }
            steps[i] = targetSlot;
        }
        return steps;
    }

    // =========================
// Angle helpers (unchanged)
// =========================

    private double getMotorAngle(DcMotorEx m, double ticksPerRev) {
        double angle = (m.getCurrentPosition() % (int)ticksPerRev) * (360.0 / ticksPerRev);
        return angle < 0 ? angle + 360 : angle;
    }

    private int lastCommandedTarget = 0;     // new: prevents constant retargeting
    private boolean targetInitialized = false;

    // Slot-selection helper (unchanged externally)
    private void goToSlot(int slot) {
        if (slot < 0 || slot > 2) return;
        revIndex = slot;
        RevPos(revPositions[slot]);   // still uses angle targets
    }
//  NEW RevPos — SAME OUTPUT
// =========================

    private void RevPos(int targetAngle) {

        // Convert targetAngle into an absolute tick target ONCE
        double currentAngle = getMotorAngle(robot.RevM, RevTPR);
        double delta = targetAngle - currentAngle;

        // Shortest angular direction
        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;

        int deltaTicks = (int)((delta / 360.0) * RevTPR);

        // Compute the *absolute* final tick target
        int absoluteTarget = robot.RevM.getCurrentPosition() + deltaTicks;

        if (!targetInitialized || absoluteTarget != lastCommandedTarget) {
            lastCommandedTarget = absoluteTarget;
            targetInitialized = true;

            robot.RevM.setTargetPosition(absoluteTarget);
            robot.RevM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            robot.RevM.setPower(0.2);    // lower soft approach for zero overshoot
        }

        // (Motor holds target once reached; no repeated retarget)
    }


    private double angleToServo(double angleDeg) {
        double n = (angleDeg - MIN_ANGLE_DEG)/(MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double pos = SERVO_MIN_POS + n*(SERVO_MAX_POS - SERVO_MIN_POS);
        return Range.clip(pos, SERVO_MAX_POS, SERVO_MIN_POS);
    }

    private void updateDistanceFromTag(double robotX, double robotZ) { // 1️⃣ Compute straight-line distance to tag
        double distance = Math.sqrt(robotX * robotX + robotZ * robotZ);
    }

    private double updateShooterPID(DcMotorEx motor, double targetPower, double dt) {
        final double MAX_TICKS_PER_SEC = 53770.0;
        double targetVelocity = targetPower*MAX_TICKS_PER_SEC;
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        shooterIntegral += error*dt;
        shooterIntegral = Range.clip(shooterIntegral, -1e6, 1e6);

        double derivative = (error - lastShooterError)/Math.max(dt, 1e-6);
        lastShooterError = error;

        double pid = (kP_shooter*error) + (kI_shooter*shooterIntegral) + (kD_shooter*derivative);

        double correction = pid/MAX_TICKS_PER_SEC;
        double basePower = targetVelocity/MAX_TICKS_PER_SEC;

        double newPower = basePower + correction;
        newPower = Range.clip(newPower, MIN_POWER, MAX_POWER);

        telemetry.addData("ShooterPID","TGT:%.0f CUR:%.0f PWR:%.3f ERR:%.0f",
                targetVelocity, currentVelocity, newPower, error);
        return newPower;
    }

    /**
     * Gentle ramp to zero for the shooter motor to mimic a "light brake".
     * Duration is in seconds (small, e.g. 0.15 - 0.3)
     */
//    private void rampStopShooter(double durationSec) {
//        if (durationSec <= 0) {
//            robot.ShotM.setPower(0.0);
//            return;
//        }
//        double startPower = Math.abs(robot.ShotM.getPower());
//        ElapsedTime t = new ElapsedTime();
//        while (opModeIsActive() && t.seconds() < durationSec) {
//            double frac = 1.0 - (t.seconds()/durationSec);
//            double p = startPower * Math.max(frac, 0.0);
//            // keep the sign if you ever set negative; here shooter is positive
//            robot.ShotM.setPower(p);
//            // short yielding to allow other hardware to update
//            sleep(10);
//        }
//        robot.ShotM.setPower(0.0);
//    }

    private boolean ballLiftActive = false;
    private ElapsedTime ballLiftTimer = new ElapsedTime();
    private boolean ballLiftRetracting = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(75);
        limelight.pipelineSwitch(0);
        limelight.start();

        robot.RevM.setZeroPowerBehavior(BRAKE);
        robot.RevM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //robot.RevM.setPower(0);
        robot.PitchS.setPosition(SERVO_MIN_POS);
        robot.BallLiftS.setPosition(1.0);

        centerTicks = robot.ShotRotateM.getCurrentPosition();
        int ninetyDegTicks = (int)((330.0/360.0)*SHOT_TPR);
        minTicks = centerTicks - ninetyDegTicks;
        maxTicks = centerTicks + ninetyDegTicks;

        robot.TransSBL.setPosition(1.0);
        robot.TransSFL.setPosition(1.0);
        robot.TransSBR.setPosition(0.0);
        robot.TransSFR.setPosition(0.0);

        telemetry.addLine("Initialized");
        telemetry.update();

        double lastX = 0, lastY = 0, lastZ = 0;
        pidTimer.reset();
        patternTimer.reset();
        waitForStart();
        while (opModeIsActive()) {
            sleep(2000);
            RevPos(0);
            robot.ShotM.setPower(0.72);
            robot.PitchS.setPosition(0.14);
            sleep(10000);

            robot.BallLiftS.setPosition(0.7);
            sleep(750);
            robot.BallLiftS.setPosition(1.0);
            sleep(500);
            RevPos(120);
            sleep(2500);
            robot.BallLiftS.setPosition(0.7);
            sleep(300);
            robot.BallLiftS.setPosition(1.0);
            sleep(500);
            RevPos(240);
            sleep(2500);
            robot.BallLiftS.setPosition(0.7);
            sleep(300);
            robot.BallLiftS.setPosition(1.0);
            sleep(300);
            RevPos(0);
            sleep(2500);
            robot.BallLiftS.setPosition(0.7);
            sleep(250);
            robot.BallLiftS.setPosition(1.0);
            robot.ShotM.setPower(0.0);
            robot.PitchS.setPosition(1.0);
            robot.BLW.setPower(-0.7);
            robot.BRW.setPower(-0.7);
            robot.FLW.setPower(-0.7);
            robot.FRW.setPower(-0.7);
            sleep(550);
            robot.BLW.setPower(0.0);
            robot.BRW.setPower(0.0);
            robot.FLW.setPower(0.0);
            robot.FRW.setPower(0.0);
            break;
        }
    }
}
