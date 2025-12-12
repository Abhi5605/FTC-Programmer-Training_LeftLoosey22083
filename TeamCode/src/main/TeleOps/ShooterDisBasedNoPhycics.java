package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "ShooterDisBasedNoPhycics", group = "Testing")
public class ShooterDisBasedNoPhycics extends LinearOpMode {

    // ===== Hardware =====

    private Limelight3A limelight;
    private DcMotor shotMotor;        // ShotM
    private Servo PitchS;             // Pitching Servo
    private DcMotorEx ShotRotateM;    // Shooter Rotator Motor

    // ===== Constants =====
    private final double MIN_DIST_IN = 35.0;
    private final double MAX_DIST_IN = 135.0;

    private final double MIN_POWER = 0.65;
    private final double MAX_POWER = 1.0;

    private final double MAX_ANGLE_DEG = 22.5;
    private final double MIN_ANGLE_DEG = 40.0;

    private final double SERVO_MAX_POS = 1.0;
    private final double SERVO_MIN_POS = 0.0;

    final double SHOT_TPR = 537.7;
    final double MAX_ROT_POWER = 0.22;
    final double kP_rot = 0.025;
    final double deadband = 0.5;

    private boolean applySolution = false;

    private int centerTicks;
    private int minTicks;
    private int maxTicks;

    @Override
    public void runOpMode() {

        // ===== Hardware Mapping =====
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shotMotor = hardwareMap.get(DcMotor.class, "ShotM");
        PitchS = hardwareMap.get(Servo.class, "PitchS");
        ShotRotateM = hardwareMap.get(DcMotorEx.class, "ShotRotateM");

        // ===== Limelight Init =====
        limelight.setPollRateHz(75);
        limelight.pipelineSwitch(0);
        limelight.start();

        // ===== Shooter Motor Init =====
        shotMotor.setDirection(DcMotor.Direction.REVERSE);
        shotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shotMotor.setPower(0.0);

        // ===== Pitch Servo Init =====
        PitchS.setPosition(1.0);

        // ===== Rotator Init =====
        ShotRotateM.setDirection(DcMotor.Direction.REVERSE);

        centerTicks = ShotRotateM.getCurrentPosition();
        int ninetyDegTicks = (int) ((325.0 / 360.0) * SHOT_TPR); ///This is very changed
        minTicks = centerTicks - ninetyDegTicks;
        maxTicks = centerTicks + ninetyDegTicks;

        double lastX = 0, lastY = 0, lastZ = 0;

        telemetry.addLine("Shooter + Limelight Only — Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ===== LIMELIGHT TRACKING (UNCHANGED) =====
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

            // ===== AUTO CENTER ROTATOR (UNCHANGED) =====
            int currentTicks = ShotRotateM.getCurrentPosition();
            double steerPower = 0;

            if (foundTag20) {

                steerPower = -tx * kP_rot;
                steerPower = Range.clip(steerPower, -MAX_ROT_POWER, MAX_ROT_POWER);

                if ((currentTicks <= minTicks && steerPower < 0) ||
                        (currentTicks >= maxTicks && steerPower > 0)) {
                    steerPower = 0;
                }

                if (Math.abs(tx) < deadband) {
                    steerPower = 0;
                }
            }

            ShotRotateM.setPower(steerPower);

            // ===== DISTANCE, ANGLE, POWER (UNCHANGED) =====
            double horizDistM = Math.hypot(robotX, robotZ);
            double horizDistIn = horizDistM / 0.0254;

            double distClamped = Range.clip(horizDistIn, MIN_DIST_IN, MAX_DIST_IN);
            double normalized = (distClamped - MIN_DIST_IN) / (MAX_DIST_IN - MIN_DIST_IN);
            double targetPower = MIN_POWER + normalized * (MAX_POWER - MIN_POWER);

            double targetAngleDeg = MIN_ANGLE_DEG + normalized * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);

            double servoNorm = normalized;

            double servoPos = SERVO_MAX_POS + servoNorm * (SERVO_MAX_POS - SERVO_MIN_POS);

            servoPos = Range.clip(servoPos, SERVO_MAX_POS, SERVO_MIN_POS);

            // ===== APPLY / PREVIEW (UNCHANGED) =====
            if (gamepad1.a) applySolution = true;
            if (gamepad1.b) {
                applySolution = false;
                shotMotor.setPower(0);
            }

            if (applySolution) {
                PitchS.setPosition(servoPos);
                shotMotor.setPower(targetPower);
            }

            // ===== Telemetry =====
            telemetry.addData("Found Tag20", foundTag20);
            telemetry.addData("tx", tx);
            telemetry.addData("Distance (in)", horizDistIn);
            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Target Angle", targetAngleDeg);
            telemetry.addData("Servo Pos", servoPos);
            telemetry.addData("Rotator Ticks", currentTicks);
            telemetry.addData("applySolution", applySolution);
            telemetry.update();
        }
    }
}
