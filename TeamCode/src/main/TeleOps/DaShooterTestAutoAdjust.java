package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "DaShooterTestAutoAdjust", group = "Main")
public class DaShooterTestAutoAdjust extends LinearOpMode {

    // Hardware
    private Limelight3A limelight;
    private DcMotor shotMotor;      // config name: "ShooterMotor"
    private Servo ballPitchS;        // config name: "BallPitchS"
    private IMU imu;

    // --- Tuning & physical constants ---
    private final double INCH = 0.0254;
    private final double TAG_TO_GOAL_CENTER_IN = 9.25;
    private final double GOAL_PASS_MIN_ABOVE_GOAL_IN = 6;
    private final double GOAL_PASS_MAX_ABOVE_GOAL_IN = 15;
    private final double GOAL_BEHIND_OFFSET_IN = 8.0;
    private final double FLYWHEEL_DIAMETER_IN = 4.685;
    private final double FLYWHEEL_RADIUS_M = (FLYWHEEL_DIAMETER_IN * INCH) / 2.0;
    private final int MOTOR_FREE_RPM = 6000;
    private double spinToExitEfficiency = 0.95;
    private final double CONTACT_LENGTH_IN = 4.5;
    private final double CONTACT_LENGTH_M = CONTACT_LENGTH_IN * INCH;
    private final double SHOOTER_EXIT_HEIGHT_IN = -14.5;
    private final double SHOOTER_EXIT_HEIGHT_M = SHOOTER_EXIT_HEIGHT_IN * INCH;
    private double minAngleDeg = 30.0;
    private double maxAngleDeg = 60.0;
    private double allowedMinPower = -0.5;
    private double allowedMaxPower = -1.0;
    private final double SERVO_MIN_POS = 1.0;
    private final double SERVO_MAX_POS = 0.6;
    private boolean applySolution = false;

    // Simple struct for solver results
    static class ShotRange {
        boolean valid = false;
        double minAngleDeg = Double.POSITIVE_INFINITY;
        double maxAngleDeg = Double.NEGATIVE_INFINITY;
        double minV = Double.POSITIVE_INFINITY;
        double maxV = Double.NEGATIVE_INFINITY;
        double bestAngleDeg = -1;
        double bestV = Double.POSITIVE_INFINITY;
        double minPower = 0;
        double maxPower = 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // --- init hardware ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shotMotor = hardwareMap.get(DcMotor.class, "ShotM");
        ballPitchS = hardwareMap.get(Servo.class, "BallPitchS");
        imu = hardwareMap.get(IMU.class, "imu");

        // start limelight (pose pipeline)
        limelight.setPollRateHz(75);
        limelight.pipelineSwitch(0); // make sure this pipeline returns Pose3D
        limelight.start();

        shotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shotMotor.setPower(0.0);

        ballPitchS.setPosition(1.0);

        telemetry.addLine("ShooterPhysicsTeleOp initialized");
        telemetry.update();
        waitForStart();

        // fallback pose
        double lastRobotX = 0.0, lastRobotY = 0.0, lastRobotZ = 0.0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            boolean foundTag20;
            double robotX = 0.0, robotY = 0.0, robotZ = 0.0;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (f.getFiducialId() == 20) {
                            foundTag20 = true;
                            Pose3D p = f.getRobotPoseTargetSpace();
                            if (p != null) {
                                robotX = p.getPosition().x;
                                robotY = p.getPosition().y;
                                robotZ = p.getPosition().z;
                                lastRobotX = robotX;
                                lastRobotY = robotY;
                                lastRobotZ = robotZ;
                            }
                            break;
                        }
                    }
                }
            } else {
                robotX = lastRobotX;
                robotY = lastRobotY;
                robotZ = lastRobotZ;
            }

            // --- compute horizontal distance & vertical delta from Pose3D ---
            double horizDist = Math.hypot(robotX, robotZ) - GOAL_BEHIND_OFFSET_IN * INCH;
            if (horizDist < 0) horizDist = 0.0;

            double deltaH_goalCenter = (-TAG_TO_GOAL_CENTER_IN * INCH) - robotY;

            double midWindowMinAbs_m = (TAG_TO_GOAL_CENTER_IN + GOAL_PASS_MIN_ABOVE_GOAL_IN) * INCH;
            double midWindowMaxAbs_m = (TAG_TO_GOAL_CENTER_IN + GOAL_PASS_MAX_ABOVE_GOAL_IN) * INCH;
            double midHmin_rel_shooter = midWindowMinAbs_m - SHOOTER_EXIT_HEIGHT_M;
            double midHmax_rel_shooter = midWindowMaxAbs_m - SHOOTER_EXIT_HEIGHT_M;

            double midX = horizDist * 0.5;

            ShotRange sr = computeShotRangeWindow(horizDist, deltaH_goalCenter, midX, midHmin_rel_shooter, midHmax_rel_shooter, minAngleDeg, maxAngleDeg);
            if (sr.valid) {
                sr.minPower = exitVelocityToMotorPower(sr.minV);
                sr.maxPower = exitVelocityToMotorPower(sr.maxV);
            }

            // --- control toggles ---
            if (gamepad1.a) applySolution = true;
            if (gamepad1.b) {
                applySolution = false;
                shotMotor.setPower(0.0);
            }

            // --- apply solution if toggled ---
            if (applySolution && sr.valid) {
                double bestV = sr.bestV;
                double bestAngle = sr.bestAngleDeg;
                double estPower = exitVelocityToMotorPower(bestV);
                double clampedPower = Range.clip(estPower, allowedMinPower, allowedMaxPower);

                // set pitch servo & shooter motor
                setPitchServoFromAngle(bestAngle);
                shotMotor.setPower(clampedPower);

                double[] contact = contactAccelerationCheck(bestV, CONTACT_LENGTH_M);

                telemetry.addData("Applied", "YES");
                telemetry.addData("BestAngle (deg)", "%.2f", bestAngle);
                telemetry.addData("BestV (m/s)", "%.2f", bestV);
                telemetry.addData("MotorPower (est)", "%.2f", estPower);
                telemetry.addData("MotorPower (clamped)", "%.2f", clampedPower);
                telemetry.addData("Contact aReq (m/s^2)", "%.0f", contact[0]);
                telemetry.addData("Contact t (ms)", "%.1f", contact[1] * 1000);
            } else {
                // --- only show solver telemetry without moving hardware ---
                if (!sr.valid) {
                    telemetry.addData("Solver", "No valid solution in angle range");
                } else {
                    telemetry.addData("Solver Valid", "YES");
                    telemetry.addData("Angle Range (deg)", String.format("%.1f - %.1f", sr.minAngleDeg, sr.maxAngleDeg));
                    telemetry.addData("Velocity Range (m/s)", String.format("%.2f - %.2f", sr.minV, sr.maxV));
                    telemetry.addData("Motor Power Est (raw)", String.format("%.2f - %.2f", sr.minPower, sr.maxPower));
                    telemetry.addData("Best Angle (deg)", String.format("%.1f", sr.bestAngleDeg));
                    telemetry.addData("Best Velocity (m/s)", String.format("%.2f", sr.bestV));
                }
                telemetry.addData("Applied", "NO");
            }
            telemetry.update();
        }

    }
    // --- physics helpers ---
    private ShotRange computeShotRangeWindow(double horizDist, double deltaH,
                                             double midX, double midHmin, double midHmax,
                                             double minAngleDeg, double maxAngleDeg) {
        ShotRange r = new ShotRange();
        double g = 9.81;

        for (double angleDeg = minAngleDeg; angleDeg <= maxAngleDeg; angleDeg += 0.05) {
            double theta = Math.toRadians(angleDeg);
            double denomFinal = 2 * Math.pow(Math.cos(theta), 2) * (horizDist * Math.tan(theta) - deltaH);
            if (denomFinal <= 0) continue;

            double vFinal = Math.sqrt((g * horizDist * horizDist) / denomFinal);
            double yAtMid = midX * Math.tan(theta) - (g * midX * midX) / (2.0 * vFinal * vFinal * Math.pow(Math.cos(theta), 2));
            if (yAtMid < midHmin || yAtMid > midHmax) continue;

            r.valid = true;
            r.minAngleDeg = Math.min(r.minAngleDeg, angleDeg);
            r.maxAngleDeg = Math.max(r.maxAngleDeg, angleDeg);
            r.minV = Math.min(r.minV, vFinal);
            r.maxV = Math.max(r.maxV, vFinal);

            if (vFinal < r.bestV) {
                r.bestV = vFinal;
                r.bestAngleDeg = angleDeg;
            }
        }

        if (r.valid) {
            r.minPower = exitVelocityToMotorPower(r.minV);
            r.maxPower = exitVelocityToMotorPower(r.maxV);
        }

        return r;
    }

    private double exitVelocityToMotorPower(double vReq) {
        if (vReq <= 0.0) return 0.0;
        double omegaRps = vReq / (spinToExitEfficiency * FLYWHEEL_RADIUS_M);
        double rpmNeeded = omegaRps * 60.0 / (2.0 * Math.PI);
        return Range.clip(rpmNeeded / MOTOR_FREE_RPM, 0.0, 1.0);
    }

    private double[] contactAccelerationCheck(double vExit, double contactLengthM) {
        double[] out = new double[2];
        if (vExit <= 0 || contactLengthM <= 0) { out[0]=0; out[1]=0; return out; }
        out[0] = (vExit*vExit)/(2*contactLengthM);
        out[1] = (2*contactLengthM)/vExit;
        return out;
    }

    private void setPitchServoFromAngle(double angleDeg) {
        double normalized = (angleDeg - minAngleDeg)/(maxAngleDeg-minAngleDeg);
        double pos = Range.clip(SERVO_MIN_POS + normalized*(SERVO_MAX_POS - SERVO_MIN_POS), SERVO_MIN_POS, SERVO_MAX_POS);
        ballPitchS.setPosition(pos);
    }
}
