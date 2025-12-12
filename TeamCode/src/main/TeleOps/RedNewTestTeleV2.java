package org.firstinspires.ftc.teamcode.TeleOps;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareMapping.HardwareMap1;
import org.firstinspires.ftc.teamcode.HardwareMapping.ColorSensor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RedNewTestTeleV2", group = "TeleOp")
public class RedNewTestTeleV2 extends LinearOpMode {

    private static final Logger log = LoggerFactory.getLogger(RedNewTestTeleV2.class);

    private final HardwareMap1 robot = new HardwareMap1();
    private final ColorSensor colorSensors = new ColorSensor();

    private IMU imu;
    public Limelight3A limelight;

    // ===== Auto Shoot State Machine =====
    private enum AutoShootState {
        IDLE,
        SPINUP,
        MOVE_REVOLVER,
        FIRE,
        ADVANCE,
        DONE
    }

    private AutoShootState autoState = AutoShootState.IDLE;
    private BallColor[] colorOrder = null;

    private int findSlotWithColor(BallColor targetColor) {
        BallColor[] slots = getRevolverColors();  // your existing function that reads 3 sensors
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
    ShootState shootState = ShootState.IDLE;

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
    private BallColor[] activePattern = null;

    // ------------------
    // Teleop variables
    // ------------------
    private boolean isSlowMode = false;
    private boolean ShootOn = false;
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

    private final double kP_shooter = 0.0007;
    private final double kI_shooter = 0.0002;
    private final double kD_shooter = 0.0001;
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

    private BallColor getColorFromSensor(RevColorSensorV3 sensor) {
        double[] rgb = getNormalizedRGB(sensor);
        double r = rgb[0], g = rgb[1], b = rgb[2];
        if (g > 0.44 && r < 0.235) return BallColor.GREEN;
        if (r > 0.24 && b > 0.33) return BallColor.PURPLE;
        return null;
    }

    private BallColor[] getRevolverColors() {
        return new BallColor[] {
                getColorFromSensor(colorSensors.ColS0),
                getColorFromSensor(colorSensors.ColS120),
                getColorFromSensor(colorSensors.ColS240)
        };
    }

    private int[] buildRevolverSteps() {
        if (activePattern == null) return new int[]{0,1,2};
        BallColor[] colorsOnWheel = getRevolverColors();
        int[] steps = new int[3];
        boolean[] usedSlots = new boolean[3];

        for (int i = 0; i < 3; i++) {
            BallColor needed = activePattern[i];
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
            robot.RevM.setPower(0.25);    // lower soft approach for zero overshoot
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
    private void rampStopShooter(double durationSec) {
        if (durationSec <= 0) {
            robot.ShotM.setPower(0.0);
            return;
        }
        double startPower = Math.abs(robot.ShotM.getPower());
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < durationSec) {
            double frac = 1.0 - (t.seconds()/durationSec);
            double p = startPower * Math.max(frac, 0.0);
            // keep the sign if you ever set negative; here shooter is positive
            robot.ShotM.setPower(p);
            // short yielding to allow other hardware to update
            sleep(10);
        }
        robot.ShotM.setPower(0.0);
    }

    private boolean ballLiftActive = false;
    private ElapsedTime ballLiftTimer = new ElapsedTime();
    private boolean ballLiftRetracting = false;

    @Override
    public void runOpMode() {
        // Initialization
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
        robot.RevM.setPower(0);
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
            double dt = pidTimer.seconds();
            pidTimer.reset();

            // ----------------------------
            // Drive
            // ----------------------------
            double yPower = -gamepad1.left_stick_y;
            double xPower = gamepad1.left_stick_x * 1.1;
            double zPower = -gamepad1.right_stick_x;

            if (gamepad1.left_bumper) isSlowMode = true;
            if (gamepad1.right_bumper) isSlowMode = false;

            double Slow = isSlowMode ? 0.32 : 1.0;

            robot.FLW.setPower(Range.clip(yPower + xPower + zPower, -1,1)*Slow);
            robot.FRW.setPower(Range.clip(yPower - xPower - zPower, -1,1)*Slow);
            robot.BLW.setPower(Range.clip(yPower - xPower + zPower, -1,1)*Slow);
            robot.BRW.setPower(Range.clip(yPower + xPower - zPower, -1,1)*Slow);

            // ----------------------------
            // Revolver buttons (non-blocking)
            // ----------------------------
            boolean rightTrigNow = gamepad2.right_trigger > 0.3;
            if (rightTrigNow && !prevRightTriggerPressed && rightTriggerTimer.seconds() > REV_TRIGGER_DEBOUNCE) {
                revIndex = (revIndex + 1) % revPositions.length;
                RevPos(revPositions[revIndex]);
                rightTriggerTimer.reset();
            }
            prevRightTriggerPressed = rightTrigNow;

            boolean leftTrigNow = gamepad2.left_trigger > 0.3;
            if (leftTrigNow && !prevLeftTriggerPressed && leftTriggerTimer.seconds() > REV_TRIGGER_DEBOUNCE) {
                revIndex = (revIndex - 1 + revPositions.length) % revPositions.length;
                RevPos(revPositions[revIndex]);
                leftTriggerTimer.reset();
            }
            prevLeftTriggerPressed = leftTrigNow;

            if (gamepad2.dpad_down) goToSlot(0);
            if (gamepad2.dpad_left) goToSlot(1);
            if (gamepad2.dpad_right) goToSlot(2);

            // ----------------------------
            // Ball pitch (non-blocking)
            // ----------------------------
            if (!ballPitchActive && gamepad2.dpad_up) {
                ballPitchActive = true;
                ballPitchTimer.reset();
                robot.BallLiftS.setPosition(0.7);
            }

            if (ballPitchActive && ballPitchTimer.seconds() > BALL_PITCH_DURATION) {
                robot.BallLiftS.setPosition(1.0);
                ballPitchActive = false;
            }



            // Start COLOR-BASED auto shoot (3 balls)
            if (gamepad2.a && !autoColorShoot && !autoSimpleShoot) {
                // initialize colorOrder from activePattern if available, otherwise sample current wheel
                if (activePattern != null) {
                    colorOrder = new BallColor[3];
                    System.arraycopy(activePattern, 0, colorOrder, 0, 3);
                } else {
                    colorOrder = getRevolverColors();
                }
                autoColorShoot = true;
                autoStep = 0;
                autoTimer.reset();
            }

            // Start SIMPLE 0→1→2 auto shoot
            if (gamepad2.b && !autoSimpleShoot && !autoColorShoot) {
                autoSimpleShoot = true;
                autoStep = 0;
                autoTimer.reset();
            }

            if (autoColorShoot) {
                if (autoStep >= 3) {
                    autoColorShoot = false;
                } else {
                    BallColor needed = null;
                    if (colorOrder != null && autoStep < colorOrder.length) needed = colorOrder[autoStep];

                    int targetSlot = -1;
                    if (needed != null) targetSlot = findSlotWithColor(needed);
                    if (targetSlot == -1) targetSlot = autoStep; // fallback

                    goToSlot(targetSlot);
                    double currentAngle = getMotorAngle(robot.RevM, RevTPR);

                    if (Math.abs(currentAngle - revPositions[targetSlot]) < 2.5) {
                        if (!ballLiftActive && !ballLiftRetracting) {
                            robot.BallLiftS.setPosition(0.7); // extend
                            ballLiftTimer.reset();
                            ballLiftActive = true;
                        }

                        // Wait 500ms for servo to push ball
                        if (ballLiftActive && ballLiftTimer.milliseconds() > 250) {
                            robot.BallLiftS.setPosition(1.0); // retract
                            ballLiftActive = false;
                            ballLiftRetracting = true;
                            ballLiftTimer.reset();
                        }

                        // Wait additional 250ms for servo to fully retract before next step
                        if (ballLiftRetracting && ballLiftTimer.milliseconds() > 250) {
                            ballLiftRetracting = false;
                            autoStep++;
                            autoTimer.reset();

                            if (autoStep >= 3) autoColorShoot = false;
                        }
                    }
                }
            }

// -----------------------------
// Auto Simple Shoot (0 → 1 → 2)
// -----------------------------
            if (autoSimpleShoot) {
                if (autoStep >= 3) {
                    autoSimpleShoot = false;
                } else {
                    int targetSlot = autoStep;
                    goToSlot(targetSlot);
                    double currentAngle = getMotorAngle(robot.RevM, RevTPR);

                    if (Math.abs(currentAngle - revPositions[targetSlot]) < 2.5) {
                        if (!ballLiftActive && !ballLiftRetracting) {
                            robot.BallLiftS.setPosition(0.7); // extend
                            ballLiftTimer.reset();
                            ballLiftActive = true;
                        }

                        // Wait 500ms for servo to push ball
                        if (ballLiftActive && ballLiftTimer.milliseconds() > 250) {
                            robot.BallLiftS.setPosition(1.0); // retract
                            ballLiftActive = false;
                            ballLiftRetracting = true;
                            ballLiftTimer.reset();
                        }

                        // Wait additional 250ms for servo to fully retract before next step
                        if (ballLiftRetracting && ballLiftTimer.milliseconds() > 250) {
                            ballLiftRetracting = false;
                            autoStep++;
                            autoTimer.reset();

                            if (autoStep >= 3) autoSimpleShoot = false;
                        }
                    }
                }
            }

            if (gamepad2.right_bumper) {
                robot.BallLiftS.setPosition(0.7);
            }

            if (gamepad2.left_bumper) {
                robot.BallLiftS.setPosition(1.0);
            }

            // ----------------------------
            // Intake toggle (non-blocking)
            // ----------------------------
            if (gamepad1.rightBumperWasPressed() && intakeToggleTimer.seconds() > TOGGLE_DELAY) {
                if (InTakeOn) {
                    robot.InM.setPower(-0.8);
                    robot.RevM.setPower(0.07);
                } else {
                    robot.InM.setPower(0);
                    robot.RevM.setPower(0);
                }
                InTakeOn = !InTakeOn;
                intakeToggleTimer.reset();
            }

            // ----------------------------
            // Shooter toggle
            // ----------------------------
            if (gamepad2.y) {
                ShootOn = !ShootOn;
                applySolution = ShootOn;
                if (!ShootOn) {
                    // Gentle stop for shooter motor instead of abruptly setting float/brake
                    rampStopShooter(0.20); // 200 ms gentle ramp to zero
                    robot.ShotM.setPower(0.0);
                    // set wheel behaviour to brake so robot doesn't roll when shooting is off
                    robot.FLW.setZeroPowerBehavior(BRAKE);
                    robot.FRW.setZeroPowerBehavior(BRAKE);
                    robot.BLW.setZeroPowerBehavior(BRAKE);
                    robot.BRW.setZeroPowerBehavior(BRAKE);
                } else {
                    // When turning on, allow shooter pid to run - make sure wheels coast while shooting
                    robot.FLW.setZeroPowerBehavior(FLOAT);
                    robot.FRW.setZeroPowerBehavior(FLOAT);
                    robot.BLW.setZeroPowerBehavior(FLOAT);
                    robot.BRW.setZeroPowerBehavior(FLOAT);
                }
            }

            // ----------------------------
            // Lift toggle (non-blocking)
            // ----------------------------
            if (gamepad1.dpad_up && liftToggleTimer.seconds() > TOGGLE_DELAY) {
                LiftUp = !LiftUp;
                if (LiftUp) {
                    robot.TransSBL.setPosition(0.2);
                    robot.TransSFL.setPosition(0.2);
                    robot.TransSBR.setPosition(0.8);
                    robot.TransSFR.setPosition(0.8);
                } else {
                    robot.TransSBL.setPosition(1.0);
                    robot.TransSFL.setPosition(1.0);
                    robot.TransSBR.setPosition(0.0);
                    robot.TransSFR.setPosition(0.0);
                }
                liftToggleTimer.reset();
            }

            // ----------------------------
            // Limelight + turret + shooter
            // ----------------------------
            LLResult result = limelight.getLatestResult();
            double robotX = lastX, robotY = lastY, robotZ = lastZ;
            boolean foundTag20 = false;
            double tx = 0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                if (tags != null) {
                    for (LLResultTypes.FiducialResult f : tags) {
                        if (f.getFiducialId() == 24) {
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

                    int seen = -1;
                    for (LLResultTypes.FiducialResult f : tags) {
                        int id = f.getFiducialId();
                        if (id == 21) { seen = 21; break; }
                        if (id == 22) { seen = 22; break; }
                        if (id == 23) { seen = 23; break; }
                    }

                    // pattern lock
                    if (detectedPattern != -1) {
                        if (detectedPattern == 21) activePattern = new BallColor[]{ BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE };
                        else if (detectedPattern == 22) activePattern = new BallColor[]{ BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE };
                        else if (detectedPattern == 23) activePattern = new BallColor[]{ BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN };
                    } else {
                        if (seen != -1) {
                            if (pendingPattern != seen) { pendingPattern = seen; patternTimer.reset(); }
                            else if (patternTimer.seconds() >= 1.0) detectedPattern = pendingPattern;
                        } else { pendingPattern = -1; patternTimer.reset(); }
                    }
                }
            }

            // Turret auto-aim: compute target ticks from center and Limelight tx
            int currentTicks = robot.ShotRotateM.getCurrentPosition();
            if (foundTag20) {
                // Convert tx degrees into ticks relative to center
                // Negative tx: target is right of center (Limelight convention)
                int desiredTicks = centerTicks + (int)(-tx * (SHOT_TPR / 360.0));
                desiredTicks = Range.clip(desiredTicks, minTicks, maxTicks);

                // Set motor to RUN_USING_ENCODER for active proportional control
                robot.ShotRotateM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

                // Compute signed proportional power
                int errorTicks = desiredTicks - currentTicks;
                double errorDeg = (errorTicks / SHOT_TPR) * 360.0;
                double steerPower = -errorDeg * kP_rot;

                // Clip power to max/min range
                steerPower = Range.clip(steerPower, -MAX_ROT_POWER, MAX_ROT_POWER);

                // Apply to motor
                robot.ShotRotateM.setPower(steerPower);
            } else {
                // No tag detected: optionally hold last position
                robot.ShotRotateM.setPower(0);
            }


// 1️⃣ Compute straight-line distance to tag
            double distance = Math.sqrt(robotX*robotX + robotZ*robotZ);

// 2️⃣ Clamp distance to allowed range
            double distIn = Math.max(MIN_DIST_IN, Math.min(MAX_DIST_IN, distance));

// -------------------------
// Shooter servo angle
// -------------------------
            double targetAngle = MIN_ANGLE_DEG + (MAX_ANGLE_DEG - MIN_ANGLE_DEG) *
                    (distIn - MIN_DIST_IN) / (MAX_DIST_IN - MIN_DIST_IN);

            double targetPos = angleToServo(targetAngle);
            robot.PitchS.setPosition(targetPos);

// Telemetry for debugging
            telemetry.addData("ShooterServo", "Angle: %.2f Pos: %.3f Distance: %.2f",
                    targetAngle, targetPos, distance);

// -------------------------
// Shooter motor power
// -------------------------
            if (applySolution) {
                // Use the same distance for motor power
                double tgtPower = MIN_POWER + (MAX_POWER - MIN_POWER) *
                        (distIn - MIN_DIST_IN) / (MAX_DIST_IN - MIN_DIST_IN);

                double newPower = updateShooterPID(robot.ShotM, tgtPower, dt);
                robot.ShotM.setPower(newPower);

                // Ensure drivetrain motors are in FLOAT to avoid resistance
                robot.FLW.setZeroPowerBehavior(FLOAT);
                robot.FRW.setZeroPowerBehavior(FLOAT);
                robot.BLW.setZeroPowerBehavior(FLOAT);
                robot.BRW.setZeroPowerBehavior(FLOAT);
            } else {
                if (robot.ShotM.getPower() != 0.0) {
                    robot.ShotM.setPower(0.0);
                }
            }

            if (gamepad2.x) {
                Gamepad2x = !Gamepad2x;
                if (Gamepad2x){
                    robot.ShotM.setPower(0.9);
                    robot.PitchS.setPosition(0.11);
                } else {
                    robot.ShotM.setPower(0.0);
                    robot.PitchS.setPosition(0.0);
                }
            }

            if (gamepad2.rightStickButtonWasPressed()){
                rightStickButton = !rightStickButton;
                if (rightStickButton){
                    robot.ShotM.setPower(0.75);
                    robot.PitchS.setPosition(0.5);
                } else {
                    robot.ShotM.setPower(0.0);
                    robot.PitchS.setPosition(0.0);
                }
            }

            if (gamepad2.leftStickButtonWasPressed()){
                leftStickButton = !leftStickButton;
                if (leftStickButton){
                    robot.ShotM.setPower(0.55);
                    robot.PitchS.setPosition(1.0);
                } else {
                    robot.ShotM.setPower(0.0);
                    robot.PitchS.setPosition(0.0);
                }
            }


            telemetry.addData("ShooterOn", ShootOn);
            telemetry.addData("IntakeOn", InTakeOn);
            telemetry.addData("LiftUp", LiftUp);
            telemetry.addData("RevSlot", revIndex);
            telemetry.addData("TX", tx);
            telemetry.addData("CurrentTicks", currentTicks);

            telemetry.update();

            telemetry.update();
        }
    }
}

