package org.firstinspires.ftc.teamcode.TeleOps;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "IntakeMotor", group = "TeleOp")
public class IntakeMotor extends LinearOpMode {

    private DcMotor InM;

    @Override
    public void runOpMode() {
        // ===== Hardware Mapping =====
        InM = hardwareMap.get(DcMotor.class, "InM");
        InM.setZeroPowerBehavior(BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // ===== Read gamepad =====
            double xPower = gamepad1.left_stick_x;
            double yPower = -gamepad1.left_stick_y;

            // ===== Calculate and set motor power =====
            double intakePower = Range.clip(yPower + xPower, -1.0, 1.0);
            InM.setPower(intakePower);

            // ===== Telemetry =====
            telemetry.addData("Intake Power", intakePower);
            telemetry.update();
        }
    }
}
