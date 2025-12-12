package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareMapping.ColorSensor;

import java.util.ArrayList;

@TeleOp(name = "ColorSensorTest", group = "TeleOp")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor sensors = new ColorSensor();
    private DcMotorEx RevM;

    private static final double TPR = 288;                       // ticks per rev
    private final int[] chamberAngles = {0, 120, 240};           // 0°, 120°, 240°

    // Patterns
    private String[] targetPattern21 = new String[]{"Green", "Purple", "Purple"}; // ID21
    private String[] targetPattern22 = new String[]{"Purple", "Green", "Purple"}; // ID22
    private String[] targetPattern23 = new String[]{"Purple", "Purple", "Green"}; // ID23

    // Toggle
    private boolean sequenceActive = false;
    private boolean buttonReleased = true;

    // Placeholder for AprilTag ID (later replace with Limelight)
    private int detectedTagID = 21;

    @Override
    public void runOpMode() {

        sensors.init(hardwareMap);

        RevM = hardwareMap.get(DcMotorEx.class, "revolver");
        RevM.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RevM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Revolver System Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------
            // Toggle with a single button
            // -------------------------
            if (gamepad1.a && buttonReleased) {
                sequenceActive = !sequenceActive;
                buttonReleased = false;
            } else if (!gamepad1.a) {
                buttonReleased = true;
            }

            // -------------------------
            // Execute sequence if active
            // -------------------------
            if (sequenceActive) {

                // -------------------------
                // Detect AprilTag (placeholder)
                // -------------------------
                int tagID = detectedTagID; // replace with Limelight read later
                String[] selectedPattern;

                switch (tagID) {
                    case 21: selectedPattern = targetPattern21; break;
                    case 22: selectedPattern = targetPattern22; break;
                    case 23: selectedPattern = targetPattern23; break;
                    default: selectedPattern = targetPattern21; break; // fallback
                }

                // -------------------------
                // Read current colors
                // -------------------------
                String[] chamberColors = new String[]{
                        sensors.detectColorS0(),
                        sensors.detectColorS120(),
                        sensors.detectColorS240()
                };

                // -------------------------
                // Build rotation sequence
                // -------------------------
                ArrayList<Integer> angleSequence = new ArrayList<>();
                for (String needed : selectedPattern) {
                    int index = findChamberWithColor(chamberColors, needed);
                    if (index != -1) {
                        angleSequence.add(chamberAngles[index]);
                    }
                }

                // -------------------------
                // Execute sequence with placeholder for waits/methods
                // -------------------------
                for (int i = 0; i < angleSequence.size(); i++) {
                    int targetAngle = angleSequence.get(i);

                    telemetry.addData("Step", i + 1);
                    telemetry.addData("Target Angle", targetAngle);
                    telemetry.update();

                    RevPos(targetAngle);

                    // Example wait or method placeholder
                    sleep(250);
                    // customMethod1();
                    // customMethod2();
                }

                // Done with this run
                sequenceActive = false;
            }

            // -------------------------
            // Telemetry
            // -------------------------
            telemetry.addData("Sequence Active", sequenceActive);
            telemetry.addData("Current Angle", getMotorAngle(RevM, TPR));
            telemetry.addData("S0", sensors.detectColorS0());
            telemetry.addData("S120", sensors.detectColorS120());
            telemetry.addData("S240", sensors.detectColorS240());
            telemetry.addData("Tag ID (placeholder)", detectedTagID);
            telemetry.update();
        }
    }

    // -------------------------------------------------------------
    // Find chamber containing desired color
    // -------------------------------------------------------------
    private int findChamberWithColor(String[] chambers, String color) {
        for (int i = 0; i < chambers.length; i++) {
            if (chambers[i].equals(color)) return i;
        }
        return -1;
    }

    // -------------------------------------------------------------
    // Converts encoder ticks → degrees (0–360)
    // -------------------------------------------------------------
    private double getMotorAngle(DcMotorEx m, double ticksPerRev) {
        double angle = (m.getCurrentPosition() % ticksPerRev) * (360.0 / ticksPerRev);
        return (angle < 0) ? angle + 360 : angle;
    }

    // -------------------------------------------------------------
    // Rotate to an absolute angle using shortest direction
    // -------------------------------------------------------------
    private void RevPos(int targetAngle) {

        double currentAngle = getMotorAngle(RevM, TPR);
        double delta = targetAngle - currentAngle;

        // shortest direction
        if (delta > 180) delta -= 360;
        else if (delta < -180) delta += 360;

        int deltaTicks = (int) ((delta / 360.0) * TPR);
        int newTarget = RevM.getCurrentPosition() + deltaTicks;

        RevM.setTargetPosition(newTarget);
        RevM.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RevM.setPower(0.4);

        while (RevM.isBusy() && opModeIsActive()) {
            // optional: live debug
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", getMotorAngle(RevM, TPR));
            telemetry.update();
        }

        RevM.setPower(0);
        RevM.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
