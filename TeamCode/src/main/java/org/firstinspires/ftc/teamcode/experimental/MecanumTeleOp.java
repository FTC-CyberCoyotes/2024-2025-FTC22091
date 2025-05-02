package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * MecanumTeleOp - A TeleOp mode for controlling a mecanum drivetrain robot.
 *
 * This class demonstrates how to use the MecanumDriveTrain class for teleop control.
 */
@TeleOp(name="Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends LinearOpMode {
    // The drive train subsystem
    private MecanumDriveTrain drive;

    // Timer for limiting certain operations
    private ElapsedTime runtime = new ElapsedTime();

    // Drive mode state
    private boolean fieldCentric = true;
    private double speedMultiplier = 1.0;

    @Override
    public void runOpMode() {
        // Initialize the drive subsystem
        drive = new MecanumDriveTrain(hardwareMap);

        // Display status to driver
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Main loop - runs until the STOP button is pressed
        while (opModeIsActive()) {
            // Driver controls
            controlDrivetrain();

            // Toggle drive modes
            handleDriveModeControls();

            // Display status to driver
            updateTelemetry();
        }

        // Stop all motors when exiting
        drive.stop();
    }

    /**
     * Controls the drivetrain based on gamepad input.
     */
    private void controlDrivetrain() {
        // Get drive values from gamepad
        double drive = -gamepad1.left_stick_y; // Negative because y-axis is inverted
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // Apply speed multiplier
        drive *= speedMultiplier;
        strafe *= speedMultiplier;
        turn *= speedMultiplier * 0.8; // Reduce turn sensitivity

        // Apply deadzone to prevent tiny movements when joystick is released
        drive = applyDeadzone(drive, 0.05);
        strafe = applyDeadzone(strafe, 0.05);
        turn = applyDeadzone(turn, 0.05);

        // Drive using the joystick inputs
        drive.drive(drive, strafe, turn, fieldCentric);
    }

    /**
     * Handles button presses for changing drive modes.
     */
    private void handleDriveModeControls() {
        // Toggle field-centric mode with Y/Triangle button
        if (gamepad1.y && runtime.milliseconds() > 300) {
            fieldCentric = !fieldCentric;
            runtime.reset();

            // Give haptic feedback to driver
            gamepad1.rumble(0.5, 0.5, 200);
        }

        // Speed control
        if (gamepad1.right_trigger > 0.5) {
            // Boost mode (100% speed)
            speedMultiplier = 1.0;
        } else if (gamepad1.left_trigger > 0.5) {
            // Precision mode (30% speed)
            speedMultiplier = 0.3;
        } else {
            // Default speed (70%)
            speedMultiplier = 0.7;
        }
    }

    /**
     * Updates telemetry with current robot status.
     */
    private void updateTelemetry() {
        telemetry.addData("Drive Mode", fieldCentric ? "Field Centric" : "Robot Centric");
        telemetry.addData("Speed", String.format("%.0f%%", speedMultiplier * 100));

        // Display current position
        telemetry.addData("Position", drive.getPose().toString());

        // Display heading in degrees
        telemetry.addData("Heading", String.format("%.1f°", Math.toDegrees(drive.getHeading())));

        // Controls reminder
        telemetry.addLine("Y: Toggle Field Centric | RT: Boost | LT: Precision");

        telemetry.update();
    }

    /**
     * Applies a deadzone to a joystick value.
     *
     * @param value The raw joystick value
     * @param threshold The deadzone threshold
     * @return The processed value
     */
    private double applyDeadzone(double value, double threshold) {
        if (Math.abs(value) < threshold) {
            return 0;
        }

        // Scale values between threshold and 1 to be 0 to 1
        double sign = Math.signum(value);
        return sign * (Math.abs(value) - threshold) / (1.0 - threshold);
    }
}