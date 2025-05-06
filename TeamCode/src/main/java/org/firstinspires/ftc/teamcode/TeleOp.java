package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp - Mark 8", group="Teleop")

public class TeleOp extends LinearOpMode {

    // Pivot mode enum
    private enum PivotModes {UP, HOLD, DOWN}

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CRServo intakeServo = null;
        DcMotorEx pivot = null;
        DcMotor extension = null;  // Added extension motor

        // Servo power constants
        double INTAKE_IN_POWER = 1.0;
        double INTAKE_OUT_POWER = -1.0;
        double INTAKE_OFF_POWER = 0.0;
        double intakePower = INTAKE_OFF_POWER;

        // Pivot powers
        double PIVOT_UP_POWER = 0.35;
        double PIVOT_DOWN_POWER = -0.35;
        double PIVOT_HOLD_POWER = 0.001;
        PivotModes pivotMode = PivotModes.HOLD;

        int pivot_target_pos = 0;
        int pivot_home_pos = 0;

        // Extension power constants
        double EXTENSION_OUT_POWER = 0.40; // TODO Adjust slide power as needed
        double EXTENSION_IN_POWER = -0.40; // TODO Adjust slide power as needed

        // Initialize hardware
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        pivot = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        extension = hardwareMap.get(DcMotor.class, "extensionMotor");  // Initialize extension motor

        // Set motor directions and behaviors
        pivot.setDirection(DcMotor.Direction.FORWARD);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_home_pos = 0;

        // Set extension direction and mode (from EVERYBOT)
        extension.setDirection(DcMotor.Direction.REVERSE); // Forward should EXTEND
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            // Start in HOLD mode using encoder
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                // Intake controls
                boolean intakeInButton = gamepad1.a;
                boolean intakeOutButton = gamepad1.b;
                boolean intakeOffButton = gamepad1.x;

                // This conditional reduces ambiguity when multiple buttons are pressed.
                if (intakeInButton && intakeOutButton) {
                    intakeInButton = false;
                } else if (intakeOffButton && (intakeInButton || intakeOutButton)) {
                    intakeInButton = intakeOutButton = false;
                }

                // Pivot controls
                boolean pivotUpButton = gamepad1.right_bumper;
                boolean pivotDownButton = gamepad1.left_bumper;
                if (pivotUpButton && pivotDownButton) {
                    pivotUpButton = false;
                }

                // Extension controls (from EVERYBOT)
                boolean extensionOutButton = gamepad1.right_trigger > 0.1;
                boolean extensionInButton = gamepad1.left_trigger > 0.1;
                if (extensionOutButton && extensionInButton) {
                    extensionOutButton = false;
                }

                // INTAKE CODE
                if (intakeInButton) {
                    intakePower = INTAKE_IN_POWER;
                } else if (intakeOutButton) {
                    intakePower = INTAKE_OUT_POWER;
                } else if (intakeOffButton) {
                    intakePower = INTAKE_OFF_POWER;
                }

                // CRITICAL FIX: Actually set the servo power!
                intakeServo.setPower(intakePower);

                // Determine pivot mode
                if (pivotUpButton) {
                    pivotMode = PivotModes.UP;
                    pivot_target_pos += 5;
                } else if (pivotDownButton) {
                    pivotMode = PivotModes.DOWN;
                    pivot_target_pos -= 5;
                } else {
                    pivotMode = PivotModes.HOLD;
                }

                // Make sure that motor is in the correct control mode.
                // If there is a mismatch, we are transferring into that mode.
                // If we are transferring into HOLD mode, set the target hold position.
                if ((pivotMode == PivotModes.UP || pivotMode == PivotModes.DOWN) && (pivot.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)) {
                    pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                } else if ((pivotMode == PivotModes.HOLD) && (pivot.getMode() != DcMotor.RunMode.RUN_TO_POSITION)) {
                    pivot.setTargetPosition(pivot.getCurrentPosition());
                    pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                double pivotPower;
                if (pivotMode == PivotModes.UP) {
                    pivotPower = PIVOT_UP_POWER;
                } else if (pivotMode == PivotModes.DOWN) {
                    pivotPower = PIVOT_DOWN_POWER;
                } else {
                    pivotPower = PIVOT_HOLD_POWER;
                }

                // Set pivot power
                pivot.setTargetPosition(pivot_target_pos);
                pivot.setPower(pivotPower);

                // EXTENSION CODE (from EVERYBOT)
                double extensionPower;
                if (extensionOutButton) {
                    extensionPower = EXTENSION_OUT_POWER;
                } else if (extensionInButton) {
                    extensionPower = EXTENSION_IN_POWER;
                } else {
                    extensionPower = 0;
                }

                // Apply extension power
                extension.setPower(extensionPower);

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();

                String pivot_mode_str;
                if (pivotMode == PivotModes.UP) {
                    pivot_mode_str = "UP";
                } else if (pivotMode == PivotModes.DOWN) {
                    pivot_mode_str = "DOWN";
                } else {
                    pivot_mode_str = "HOLD";
                }

                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

                // ============ INTAKE TELEMETRY ============
                telemetry.addData("Intake Power", intakePower);

                // ============ PIVOT TELEMETRY ============
                telemetry.addData("Pivot Current/Target/power", "%d, %d, %4.2f", pivot.getCurrentPosition(), pivot.getTargetPosition(), pivot.getPower());
                telemetry.addData("Pivot MODE", "%s", pivot_mode_str);

                // ============ EXTENSION TELEMETRY ============
                // Show current extension state
                // Extension is using RUN_WITHOUT_ENCODER by default currently
                telemetry.addData("Extension Power", "%4.2f", extension.getPower());
                telemetry.addData("Extension Buttons", "Out: %b, In: %b", extensionOutButton, extensionInButton);

                if (extension.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    telemetry.addData("Extension Position", "%d", extension.getCurrentPosition());
                    telemetry.addData("Extension Velocity", "%.1f", ((DcMotorEx)extension).getVelocity());
                }

                // Debug extension state information
                // telemetry.addData("Extension Direction", "%s", extension.getDirection());
                // telemetry.addData("Extension Zero Power", "%s", extension.getZeroPowerBehavior());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}