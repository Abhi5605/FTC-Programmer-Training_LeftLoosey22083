package org.firstinspires.ftc.teamcode.AutonOps;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOps.BlueSideTeleOpV1;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareMapping.HardwareMap1;
import org.firstinspires.ftc.teamcode.HardwareMapping.ColorSensor;

import java.util.List;

@Autonomous(name = "BlueSideAuto", group = "Autonomous")
@Configurable // Panels
public class BlueSideAuto extends OpMode {
    private final HardwareMap1 robot = new HardwareMap1();
    private final ColorSensor colorSensors = new ColorSensor();
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public Limelight3A limelight;

    public double[] getNormalizedRGB(RevColorSensorV3 sensor) {
        double r = sensor.red();
        double g = sensor.green();
        double b = sensor.blue();

        double total = r + g + b;
        if (total == 0) {
            return new double[]{0,0,0};
        }

        return new double[]{ r / total, g / total, b / total };
    }

    // ============================================================
    // SIMPLE SHOOTER CONSTANTS (your entire system)
    // ============================================================
    private final double MIN_DIST_IN = 25.0;
    private final double MAX_DIST_IN = 140.0;

    private final double MIN_POWER = 0.6;
    private final double MAX_POWER = 1.0;

    private final double MIN_ANGLE_DEG = 22.5;
    private final double MAX_ANGLE_DEG = 40.0;

    // Servo bottom=30° → 1.0, top=60° → 0.3 (your mapping)
    private final double SERVO_MIN_POS = 1.0;
    private final double SERVO_MAX_POS = 0.13;

    private boolean applySolution = false;

    //
    //Crazy MOTOR PID or nah?
    //
    // ============================================================
// === PID SHOOTER CONTROL VARIABLES ===
// ============================================================

    // PID constants — TUNE THESE BASED ON YOUR MOTOR
    private final double kP_shooter = 5.0;    // proportional gain
    private final double kI_shooter = 0.0;    // integral gain
    private final double kD_shooter = 0.0;    // derivative gain

    // PID internal state
    private double shooterIntegral = 0.0;
    private double lastShooterError = 0.0;

    /**
     * Get motor velocity in ticks per second
     */
//    private double getMotorVelocity(DcMotorEx motor) {
//        return motor.getVelocity(); // REV motors return ticks/sec
//    }Not needed, called in the shooter
    ElapsedTime pidTimer = new ElapsedTime();

    /**
     * Update shooter PID loop.
     * @param motor Motor to control (robot.ShotM)
     * @param targetPower Desired normalized power (0.0-1.0)
     * @param dt Loop time in seconds
     * @return New motor power to apply
     */
    private double updateShooterPID(DcMotorEx motor, double targetPower, double dt) {

        final double MAX_TICKS_PER_SEC = 53000.0; //53770.0; THIS WAS THE ORGINAL VALUE BUT it was too large
        double targetVelocity = targetPower * MAX_TICKS_PER_SEC;
        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;
        // Integral
        shooterIntegral += error * dt;
        // Derivative
        double derivative = (error - lastShooterError) / Math.max(dt, 1e-6);
        lastShooterError = error;
        // PID output = commanded ticks/sec (PID corrects around the target velocity)
        double commandedVelocity =
                targetVelocity +
                        kP_shooter * error +
                        kI_shooter * shooterIntegral +
                        kD_shooter * derivative;
        // Convert ticks/sec → motor power (0–1)
        double newPower = commandedVelocity / MAX_TICKS_PER_SEC;
        newPower = Range.clip(newPower, 0.0, 1.0);
        // Telemetry
        telemetry.addData("ShooterPID","TGT:%.0f CUR:%.0f PWR:%.3f", targetVelocity, currentVelocity, newPower);

        return newPower;
    }

    // ============================================================
    // DRIVE + OTHER STATES
    // ============================================================
    private boolean isSlowMode = false;
    private boolean rightTriggerReleased = true;
    private boolean leftTriggerReleased = true;
    private boolean LiftUp = false;
    private boolean ShootOn = true;
    private boolean InTakeOn = true;
    //FHHHHAAAAAAAAAAAAA The intake is different, use the same boolean though
//    private double sparkMiniPowerToServo(double power) {
//        return Range.clip(0.5 + (power * 0.5), 0.0, 1.0);
//    }

    private void BallPitchMove() {
        robot.BallLiftS.setPosition(0.5);//CHECK THIS
        //sleep(100);
        robot.BallLiftS.setPosition(0.0);
    }


    // REVOLVER POSITIONS
    // ============================================================
    private static final double RevTPR = 288;

    private boolean saw21 = false;
    private boolean saw22 = false;
    private boolean saw23 = false;

    private final int[] revPositions = {0, 120, 240};
    private int revIndex = 0;

    // TURRET
    // ============================================================
    final double SHOT_TPR = 537.7;
    final double MAX_ROT_POWER = 0.22;
    final double kP_rot = 0.025;
    final double deadband = 0.5;

    private int centerTicks;
    private int minTicks;
    private int maxTicks;

    enum BallColor { GREEN, PURPLE }
    private BlueSideTeleOpV1.BallColor[] activePattern = null;

    //DOES TS actually work?:
    // Revolver trigger timers
    private ElapsedTime rightTriggerTimer = new ElapsedTime();
    private ElapsedTime leftTriggerTimer = new ElapsedTime();
    private final double REV_TRIGGER_DEBOUNCE = 0.25; // 250ms between triggers

    // Ball pitch (shoot) timers
    private ElapsedTime ballPitchTimer = new ElapsedTime();
    private boolean ballPitchActive = false;
    private final double BALL_PITCH_DURATION = 0.15; // 150ms pulse

    // Intake toggle timer
    private ElapsedTime intakeToggleTimer = new ElapsedTime();
    private final double TOGGLE_DELAY = 0.35;

    // Lift toggle timer
    private ElapsedTime liftToggleTimer = new ElapsedTime();


    // ============================================================
    // Pattern-lock variables
    // ============================================================
    private int pendingPattern = -1;   // -1 = none, 1 = GPP(21), 2 = PGP(22), 3 = PPG(23)
    private int detectedPattern = -1;  // locked pattern once validated
    private ElapsedTime patternTimer = new ElapsedTime();


    // ============================================================
    // Utilities
    // ============================================================
    private double getMotorAngle(DcMotorEx m, double ticksPerRev) {
        double angle = (m.getCurrentPosition() % ticksPerRev) * (360.0 / ticksPerRev);
        return (angle < 0) ? angle + 360 : angle;
    }

    private void RevPos(int targetAngle) {
        double currentAngle = getMotorAngle(robot.RevM, RevTPR);
        double delta = targetAngle - currentAngle;

        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;

        int deltaTicks = (int)((delta / 360.0) * RevTPR);
        int newTarget = robot.RevM.getCurrentPosition() + deltaTicks;

        robot.RevM.setTargetPosition(newTarget);
        robot.RevM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        robot.RevM.setPower(0.5);

    }

    // ============================================================
    // SERVO MAPPING (simple)
    // ============================================================
    private double angleToServo(double angleDeg) {
        double n = (angleDeg - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double pos = SERVO_MIN_POS + n * (SERVO_MAX_POS - SERVO_MIN_POS);
        return Range.clip(pos, SERVO_MAX_POS, SERVO_MIN_POS);
    }

    private void detectPattern(LLResult result) {
        // keep this legacy method (),
        // but we'll use the pattern lock logic in the main loop instead.
        if (result == null || !result.isValid()) return;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return;

        for (LLResultTypes.FiducialResult f : tags) {
            int id = f.getFiducialId();

            if (id == 21) {   // GPP
                activePattern = new BlueSideTeleOpV1.BallColor[]{
                        BlueSideTeleOpV1.BallColor.GREEN,
                        BlueSideTeleOpV1.BallColor.PURPLE,
                        BlueSideTeleOpV1.BallColor.PURPLE
                };
            }

            if (id == 22) {   // PGP
                activePattern = new BlueSideTeleOpV1.BallColor[]{
                        BlueSideTeleOpV1.BallColor.PURPLE,
                        BlueSideTeleOpV1.BallColor.GREEN,
                        BlueSideTeleOpV1.BallColor.PURPLE
                };
            }

            if (id == 23) {   // PPG
                activePattern = new BlueSideTeleOpV1.BallColor[]{
                        BlueSideTeleOpV1.BallColor.PURPLE,
                        BlueSideTeleOpV1.BallColor.PURPLE,
                        BlueSideTeleOpV1.BallColor.GREEN
                };
            }
        }
    }
    private BlueSideTeleOpV1.BallColor getColorFromSensor(RevColorSensorV3 sensor) {
        if (sensor == null) return null; // handle missing sensor

        double[] rgb = getNormalizedRGB(sensor);
        double r = rgb[0];
        double g = rgb[1];
        double b = rgb[2];

        // ---- Tuned thresholds ----
        boolean isGreen = g > 0.44 && r < 0.235;
        boolean isPurple = r > 0.24 && b > 0.33;

        if (isGreen) return BlueSideTeleOpV1.BallColor.GREEN;
        if (isPurple) return BlueSideTeleOpV1.BallColor.PURPLE;

        return null; // Unknown / invalid color
    }

    private BlueSideTeleOpV1.BallColor[] getRevolverColors() {
        return new BlueSideTeleOpV1.BallColor[]{
                getColorFromSensor(colorSensors.ColS0),  // 0deg
                getColorFromSensor(colorSensors.ColS120),  // 120deg
                getColorFromSensor(colorSensors.ColS240)   // 240deg
        };
    }
    // Updated buildRevolverSteps
    private int[] buildRevolverSteps() {
        if (activePattern == null) return new int[]{0,1,2}; // fallback

        BlueSideTeleOpV1.BallColor[] colorsOnWheel = getRevolverColors();
        int[] steps = new int[3];
        boolean[] usedSlots = new boolean[3];

        for (int i = 0; i < 3; i++) {
            BlueSideTeleOpV1.BallColor needed = activePattern[i];
            int targetSlot = -1;

            // find matching color not already used
            for (int slot = 0; slot < 3; slot++) {
                if (!usedSlots[slot] && colorsOnWheel[slot] == needed) {
                    targetSlot = slot;
                    usedSlots[slot] = true;
                    break;
                }
            }

            // if pattern color not found, take any unused slot
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
    private void goToSlot(int slot) {
        if (slot < 0) return;
        RevPos(revPositions[slot]);
    }
    private int nextSlotSingleShot = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.828, 8.862),
                                    new Pose(58.043, 63.803),
                                    new Pose(15.508, 62.474)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15.508, 62.474), new Pose(62.252, 69.120))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62.252, 69.120),
                                    new Pose(66.462, 36.775),
                                    new Pose(21.268, 35.668)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.268, 35.668), new Pose(63.138, 24.591))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.138, 24.591),
                                    new Pose(42.978, 11.520),
                                    new Pose(8.640, 12.406)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(8.640, 12.406), new Pose(50.068, 12.406))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}
