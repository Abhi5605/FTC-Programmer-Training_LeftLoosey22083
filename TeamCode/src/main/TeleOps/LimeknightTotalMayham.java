package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.HardwareMapping.Limelight;

@TeleOp(name = "LimeknightTotalMayham")
public class LimeknightTotalMayham extends LinearOpMode {

    private DcMotor aimMotor;
    private Limelight limelight;        // Limelight main class
    private Limelight.LimelightAuto limelightAuto;  // Auto helper class

    @Override
    public void runOpMode() {

        // --- Motor Setup ---
        aimMotor = hardwareMap.get(DcMotor.class, "aimMotor");
        aimMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aimMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- IMU Setup ---
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // --- Limelight Setup ---
        limelight = new Limelight();
        limelight.init();

        // --- PID/Aim Constants ---
        double kP = 0.02;       // Proportional constant for aiming
        double deadband = 1.0;  // Degrees error to ignore

        telemetry.addLine("Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. Get Limelight TX for aiming ---
            Limelight.LLData llResults = limelightAuto.getLLResults();
            double tx = (llResults != null) ? llResults.tx : 0;

            // Apply deadband
            if (Math.abs(tx) < deadband) {
                tx = 0;
            }

            // Proportional control
            double power = kP * tx;

            // Clamp motor power
            power = Math.max(Math.min(power, 0.4), -0.4);
            aimMotor.setPower(power);

            // --- 2. Get robot pose in inches for pathing ---
           // Limelight.LimelightAuto.PoseData pose = limelightAuto.getRobotPoseInches();
           // double poseX = (pose != null) ? pose.PoseX : 0;
           // double poseY = (pose != null) ? pose.PoseY : 0;

            // --- 3. Detect initial tag (optional) ---
            int initialTag = limelightAuto.detectInitialTag();

            // --- 4. Select tracking tag (example for user input) ---
            int trackingTag = limelightAuto.selectTrackingTag("Blue"); // or "Red"

            // --- 5. Get distance to target in inches ---
            double distanceToTarget = limelightAuto.getDistanceToTarget(trackingTag);

            // --- 6. Telemetry for debugging ---
            telemetry.addData("TX (Aim)", tx);
            telemetry.addData("Motor Power", power);
          //  telemetry.addData("Pose X (inches)", poseX);
          //  telemetry.addData("Pose Y (inches)", poseY);
            telemetry.addData("Initial Tag Detected", initialTag);
            telemetry.addData("Tracking Tag", trackingTag);
            telemetry.addData("Distance to Target (inches)", distanceToTarget);
            telemetry.update();
        }
    }
}
