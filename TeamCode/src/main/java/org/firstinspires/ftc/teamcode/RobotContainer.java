package org.firstinspires.ftc.teamcode;

// Import Constants
//import static org.firstinspires.ftc.teamcode.ArmSubsystem;
//import static org.firstinspires.ftc.teamcode.DriveSubsystem;
//import static org.firstinspires.ftc.teamcode.IntakeSubsystem;

import android.annotation.SuppressLint;

// Import Subsystems
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;

// @Disabled

@TeleOp(group = "Drive", name = "TeleOp")

public class RobotContainer extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive driveSub;
    private IntakeSubsystem intakeSub;
    private ArmSubsystem armSub;

    @SuppressLint("DefaultLocale")

    @Override

    public void runOpMode() throws InterruptedException {

    /* Subsystems */
    MecanumDrive driveSub = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    IntakeSubsystem intakeSub = new IntakeSubsystem();
    ArmSubsystem armSub = new ArmSubsystem();

    waitForStart();

//    armSub.resetArm(); // FIXME

    while (opModeIsActive()) {
        /*
         * DRIVER INPUT MAPPING
         * Map methods (actions) from the subsystems to gamepad inputs
         * See `ControllerMapping.md` for gamepad field names
         */

        // DRIVE CODE
        // Retrieve gamepad inputs
        double y = -gamepad1.left_stick_y; // Forward/backward (inverted for joystick)
        double x = gamepad1.left_stick_x;   // Strafe left/right
        double turn = gamepad1.right_stick_x; // Rotate

        // Apply deadband
        x = applyDeadband(x);
        y = applyDeadband(y);
        turn = applyDeadband(turn);

        // Scale the values
        double scaleFactor = 1.0; // You can adjust this for different speeds
        x *= scaleFactor;
        y *= scaleFactor;
        turn *= scaleFactor;

        // Drive the robot
        new Pose2d(y, x, turn);

        boolean intakeInButton = gamepad1.a;
        boolean intakeOutButton = gamepad1.b;
        boolean intakeOffButton = gamepad1.x;
        // This conditional reduces ambiguity when multiple buttons are pressed.
        if (intakeInButton && intakeOutButton) {
            intakeInButton = false;
        } else if (intakeOffButton && (intakeInButton || intakeOutButton)) {
            intakeInButton = intakeOutButton = false;
        }

        boolean extensionOutButton = gamepad1.left_trigger > 0.2;
        boolean extensionInButton = gamepad1.left_bumper;
        if (extensionOutButton && extensionInButton) {
            extensionOutButton = false;
        }

        boolean pivotUpButton = gamepad1.right_bumper;
        boolean pivotDownButton = gamepad1.right_trigger > 0.2;
        if (pivotUpButton && pivotDownButton) {
            pivotUpButton = false;
        }

        /*
        // INTAKE CODE
        if (intakeInButton) {
            intakePower = INTAKE_IN_POWER;
        } else if (intakeOutButton) {
            intakePower = INTAKE_OUT_POWER;
        } else if (intakeOffButton) {
            intakePower = 0.0;
        }
        */

        /*
        // EXTENSION CODE
        double extensionPower;
        if (extensionOutButton) {
            extensionPower = EXTENSION_OUT_POWER;
        } else if (extensionInButton) {
            extensionPower = EXTENSION_IN_POWER;
        } else {
            extensionPower = 0;
        }
        */

        /*
        // Determine pivot mode
        if (pivotUpButton) {
            pivotMode = TeleOpControlOpMode.PivotModes.UP;
            pivot_target_pos += 5;
        } else if (pivotDownButton) {
            pivotMode = TeleOpControlOpMode.PivotModes.DOWN;
            pivot_target_pos -= 5;
        } else {
            pivotMode = TeleOpControlOpMode.PivotModes.HOLD;
        }
        */

        // Make sure that motor is in the correct control mode.
        // If there is a mismatch, we are transferring into that mode.
        // If we are transferring into HOLD mode, set the target hold position.
//        if ((pivotMode == PivotModes.UP || pivotMode == PivotModes.DOWN) && (pivot.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)) {
//            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        } else if ((pivotMode == PivotModes.HOLD) && (pivot.getMode() != DcMotor.RunMode.RUN_TO_POSITION)) {
//            pivot.setTargetPosition(pivot.getCurrentPosition());
//            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

//        double pivotPower;
//        if (pivotMode == PivotModes.UP) {
//            pivotPower = PIVOT_UP_POWER;
//        } else if (pivotMode == PivotModes.DOWN) {
//            pivotPower = PIVOT_DOWN_POWER;
//        } else {
//            pivotPower = PIVOT_HOLD_POWER;
//        }

        /*
        intake.setPower(intakePower);
        extension.setPower(extensionPower);
        pivot.setTargetPosition(pivot_target_pos);
        pivot.setPower(1.0);

        String pivot_mode_str;
        if (pivotMode == TeleOpControlOpMode.PivotModes.UP) {
            pivot_mode_str = "UP";
        } else if (pivotMode == TeleOpControlOpMode.PivotModes.DOWN) {
            pivot_mode_str = "DOWN";
        } else {
            pivot_mode_str = "HOLD";
        }
        */

        // UPDATE TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//        telemetry.addData("Intake", "%%4.2f", intakePower);
//        telemetry.addData("Extension", "%4.2f", extension.getPower());
//        telemetry.addData("Pivot Current/Target/power", "%d, %d, %4.2f", pivot.getCurrentPosition(), pivot.getTargetPosition(),pivot.getPower());
//        telemetry.addData("Pivot MODE", "%s", pivot_mode_str);
        telemetry.update();


    } // end of while loop

    } // end of runOpMode() method

    /**
     * Applies a deadband to the input value.
     *
     * @param value The input value.
     * @return The value after applying the deadband.
     */
    private double applyDeadband(double value) {
        double deadband = 0.1; // Adjust as needed
        return Math.abs(value) > deadband ? value : 0.0;
    }

} // end of RobotContainer class
