package org.firstinspires.ftc.teamcode.HardwareMapping;

import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
public class Limelight {

    private IMU imu;
    public Limelight3A limelight;

    // --- Limelight Initialization ---
    public void init() {
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        this.limelight.setPollRateHz(75);
        this.limelight.start();
    }

    // --- Switch pipeline ---
    public void switchPipeline(int pipelineNumber) {
        limelight.pipelineSwitch(pipelineNumber);
    }

    // --- Get LL Results ---
    public LLData LLResults() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            return new LLData(tx, ty, ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
            return null;
        }
    }

    // --- Get robot pose in meters ---
    public PoseData getRobotPose(double robotYaw) {
        limelight.updateRobotOrientation(robotYaw);

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;

                telemetry.addData("MT2 Location", "(" + x + ", " + y + ")");
                return new PoseData(x, y);
            }
        }

        telemetry.addData("MT2 Location", "No Valid Pose");
        return null;
    }

    // --- Detect initial tag for pattern ---
    public int detectInitialTag() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 1 || id == 2 || id == 3) {
                    telemetry.addData("Detected Initial Tag", id);
                    return id;
                }
            }
        }
        return -1;
    }

    // --- Select tracking tag ID based on input ---
    public int selectTrackingTag(String targetInput) {
        if ("Blue".equals(targetInput)) return 20;
        else if ("Red".equals(targetInput)) return 24;
        else return -1;
    }

    // --- Data classes ---
    public static class LLData {
        public double tx, ty, ta;
        public LLData(double tx, double ty, double ta) {
            this.tx = tx; this.ty = ty; this.ta = ta;
        }
    }

    public static class PoseData {
        public double PoseX, PoseY;
        public PoseData(double x, double y) {
            this.PoseX = x; this.PoseY = y;
        }
    }

    // =========================================================
    // --- LimelightAuto class for advanced features ---
    // =========================================================
    public static class LimelightAuto {

        private Limelight3A limelight;
        private IMU imu;
        private static final double METERS_TO_INCHES = 39.3701;

        public LimelightAuto(HardwareMap hardwareMap, IMU imu) {
            this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.setPollRateHz(100);
            this.limelight.start();
            this.imu = imu;
        }

        // --- Basic LL Results ---
        public LLData getLLResults() {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                return new LLData(result.getTx(), result.getTy(), result.getTa());
            }
            return null;
        }

        // --- Get robot field pose in inches ---
        public PoseData getRobotPoseInches() {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES); // if using DEGREES by default

            limelight.updateRobotOrientation(yaw);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D pose = result.getBotpose_MT2();
                if (pose != null) {
                    double xInches = pose.getPosition().x * METERS_TO_INCHES;
                    double yInches = pose.getPosition().y * METERS_TO_INCHES;
                    return new PoseData(xInches, yInches);
                }
            }
            return null;
        }

        // --- Detect initial tag ---
        public int detectInitialTag() {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId();
                    if (id == 1 || id == 2 || id == 3) return id;
                }
            }
            return -1;
        }

        // --- Select tracking tag ---
        public int selectTrackingTag(String targetInput) {
            if ("Blue".equals(targetInput)) return 20;
            else if ("Red".equals(targetInput)) return 24;
            return -1;
        }

        // --- Get horizontal offset to target ---
        public double getTxToTarget(int targetId) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetId) {
                        return fiducial.getTargetXDegrees();
                    }
                }
            }
            return 0;
        }

        // --- Get distance to tag in inches ---
        public double getDistanceToTarget(int targetId) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == targetId) {
                        Pose3D robotPose = fiducial.getRobotPoseFieldSpace();
                        if (robotPose != null) {
                            double dx = robotPose.getPosition().x * METERS_TO_INCHES;
                            double dy = robotPose.getPosition().y * METERS_TO_INCHES;
                            return Math.hypot(dx, dy);
                        }
                    }
                }
            }
            return -1;
        }

        // --- Simple projectile angle calculation ---
        public double calculateProjectileAngle(double distanceInches, double heightDiffInches, double velocityInchesPerSec) {
            double g = 386.09; // inches/sec^2
            double numerator = g * Math.pow(distanceInches, 2);
            double denominator = 2 * Math.pow(velocityInchesPerSec, 2) * (distanceInches * Math.tan(0) - heightDiffInches);
            if (denominator <= 0) return 45;
            return Math.toDegrees(Math.atan(numerator / denominator));
        }
    }
}