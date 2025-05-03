package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Freight_Frenzy_TeleOp (Blocks to Java)")
public class Freight_Frenzy_TeleOp extends LinearOpMode {

  private DcMotor right;
  private DcMotor rightlift;
  private DcMotor leftlift;
  private DcMotor left;
  private DcMotor intake;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    right = hardwareMap.get(DcMotor.class, "right");
    rightlift = hardwareMap.get(DcMotor.class, "right lift");
    leftlift = hardwareMap.get(DcMotor.class, "left lift");
    left = hardwareMap.get(DcMotor.class, "left");
    intake = hardwareMap.get(DcMotor.class, "intake");

    // Put initialization blocks here.
    right.setDirection(DcMotorSimple.Direction.REVERSE);
    rightlift.setDirection(DcMotorSimple.Direction.REVERSE);
    rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        right.setPower(-gamepad1.right_stick_y);
        left.setPower(-gamepad1.left_stick_y);
        if (gamepad1.right_bumper) {
          intake.setPower(-1);
        } else if (gamepad1.left_bumper) {
          intake.setPower(1);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.dpad_up) {
          leftlift.setTargetPosition(140);
          rightlift.setTargetPosition(140);
          leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          leftlift.setPower(1);
          rightlift.setPower(1);
          leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad2.dpad_right) {
          leftlift.setTargetPosition(90);
          rightlift.setTargetPosition(90);
          leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          leftlift.setPower(1);
          rightlift.setPower(1);
          leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad2.dpad_left) {
          leftlift.setTargetPosition(60);
          rightlift.setTargetPosition(60);
          leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          leftlift.setPower(1);
          rightlift.setPower(1);
          leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
          rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad2.dpad_down) {
          leftlift.setPower(0);
          rightlift.setPower(0);
          leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
          rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        telemetry.update();
      }
    }
  }
}
