package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GyroExampleGood (Blocks to Java)")
public class GyroExampleGood extends LinearOpMode {

  private DcMotor left;
  private DcMotor right;
  private GyroSensor gyro;
  private DcMotor BackLeft;
  private DcMotor FrontLeft;
  private DcMotor BackRight;
  private DcMotor FrontRight;

  boolean onTarget;
  ElapsedTime _7BholdTimeVariable_7D;
  double steer;
  double error;
  double GYRO_ADJUSTMENT;
  double COUNTS_PER_INCH;
  double P_TURN_COEFF;
  int HEADING_THRESHOLD;
  double leftSpeed;
  double rightSpeed;
  double P_DRIVE_COEFF;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int COUNTS_PER_MOTOR_REV;
    int DRIVE_GEAR_REDUCTION;
    int WHEEL_DIAMETER_INCHES;
    double DRIVE_SPEED;
    double TURN_SPEED;

    left = hardwareMap.get(DcMotor.class, "left");
    right = hardwareMap.get(DcMotor.class, "right");
    gyro = hardwareMap.get(GyroSensor.class, "gyro");
    BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
    FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
    BackRight = hardwareMap.get(DcMotor.class, "BackRight");
    FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

    // **************  Initialization Section  **************************************************
    left.setPower(0);
    right.setPower(0);
    COUNTS_PER_MOTOR_REV = 1440;
    DRIVE_GEAR_REDUCTION = 1;
    WHEEL_DIAMETER_INCHES = 4;
    COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    GYRO_ADJUSTMENT = 1.0765;
    DRIVE_SPEED = 0.7;
    TURN_SPEED = 0.4;
    HEADING_THRESHOLD = 1;
    P_DRIVE_COEFF = 0.15;
    P_TURN_COEFF = 0.1;
    right.setDirection(DcMotorSimple.Direction.REVERSE);
    left.setDirection(DcMotorSimple.Direction.FORWARD);
    right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    gyro.calibrate();
    while (!isStopRequested() && gyro.isCalibrating()) {
      telemetry.addData("Gyro Calibrating", "Wait to Press Start");
      telemetry.update();
    }
    telemetry.addData("Gyro Calibrated", "Ready to Run");
    telemetry.addData("Robot Heading = ", ((ModernRoboticsI2cGyro) gyro).getIntegratedZValue());
    telemetry.update();
    waitForStart();
    // ************** Start Button Pressed *****************
    gyro.resetZAxisIntegrator();
    gyroDrive(DRIVE_SPEED, 12, 0);
    gyroDrive(DRIVE_SPEED, -12, 0);
    gyroTurn(TURN_SPEED, 90);
    gyroHold(TURN_SPEED, 90, 0.5);
    gyroTurn(TURN_SPEED, 180);
    gyroHold(TURN_SPEED, 180, 0.5);
    gyroTurn(TURN_SPEED, 270);
    gyroHold(TURN_SPEED, 270, 0.5);
    gyroTurn(TURN_SPEED, 0);
    gyroHold(TURN_SPEED, 0, 0.5);
  }

  /**
   * Describe this function...
   */
  private boolean onHeading(double p_speed, double p_angle, double PCoeff) {
    onTarget = false;
    error = getError(p_angle);
    telemetry.addData("speed", p_speed);
    telemetry.addData("angle", p_angle);
    telemetry.addData("pCoeff", PCoeff);
    telemetry.addData("error", error);
    telemetry.update();
    if (Math.abs(error) <= HEADING_THRESHOLD) {
      steer = 0;
      leftSpeed = 0;
      rightSpeed = 0;
      onTarget = true;
    } else {
      steer = getSteer(error, PCoeff);
      rightSpeed = steer * p_speed;
      leftSpeed = rightSpeed * -1;
    }
    BackLeft.setPower(leftSpeed);
    FrontLeft.setPower(leftSpeed);
    BackRight.setPower(rightSpeed);
    FrontRight.setPower(rightSpeed);
    telemetry.addData("Target", p_angle);
    telemetry.addData("Error", error);
    telemetry.addData("Steer", steer);
    telemetry.addData("LeftSpeed", leftSpeed);
    telemetry.addData("RightSpeed", rightSpeed);
    return onTarget;
  }

  /**
   * Describe this function...
   */
  private double getError(double targetAngle) {
    double robotError;

    robotError = targetAngle - ((ModernRoboticsI2cGyro) gyro).getIntegratedZValue() * GYRO_ADJUSTMENT;
    while (robotError > 180) {
      robotError = robotError - 360;
    }
    while (robotError <= -180) {
      robotError = robotError + 360;
    }
    return robotError;
  }

  /**
   * Describe this function...
   */
  private void gyroTurn(double p_speed, int p_angle) {
    _7BholdTimeVariable_7D = new ElapsedTime();
    while (opModeIsActive() && !onHeading(p_speed, p_angle, P_TURN_COEFF)) {
      telemetry.update();
    }
    BackLeft.setPower(0);
    FrontLeft.setPower(0);
    BackRight.setPower(0);
    FrontRight.setPower(0);
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading4
  private UNKNOWN_TYPE onHeading4(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading6
  private UNKNOWN_TYPE onHeading6(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading8
  private UNKNOWN_TYPE onHeading8(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading10
  private UNKNOWN_TYPE onHeading10(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading12
  private UNKNOWN_TYPE onHeading12(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading2
  private UNKNOWN_TYPE onHeading2(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  private void gyroHold(double p_speed, int p_angle, double p_holdTime) {
    _7BholdTimeVariable_7D = new ElapsedTime();
    while (opModeIsActive() && _7BholdTimeVariable_7D.time() < p_holdTime) {
      onTarget = onHeading(p_speed, p_angle, P_TURN_COEFF);
      telemetry.update();
    }
    BackLeft.setPower(0);
    FrontLeft.setPower(0);
    BackRight.setPower(0);
    FrontRight.setPower(0);
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading7
  private UNKNOWN_TYPE onHeading7(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading9
  private UNKNOWN_TYPE onHeading9(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading11
  private UNKNOWN_TYPE onHeading11(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading13
  private UNKNOWN_TYPE onHeading13(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading5
  private UNKNOWN_TYPE onHeading5(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  // TODO: Enter the correct return type for function named onHeading3
  private UNKNOWN_TYPE onHeading3(
      // TODO: Enter the type for argument named p_speed
      UNKNOWN_TYPE p_speed,
      // TODO: Enter the type for argument named p_angle
      UNKNOWN_TYPE p_angle,
      // TODO: Enter the type for argument named PCoeff
      UNKNOWN_TYPE PCoeff) {
  }

  /**
   * Describe this function...
   */
  private double getSteer(double p_error, double PCoeff) {
    steer = p_error * PCoeff;
    if (steer > 1) {
      steer = 1;
    } else if (steer < -1) {
      steer = -1;
    }
    return steer;
  }

  /**
   * Describe this function...
   */
  private void gyroDrive(double p_speed, int p_distance, int p_angle) {
    double MoveCounts;
    double newLeftBackTarget;
    double newRightBackTarget;
    double newLeftFrontTarget;
    double newRightFrontTarget;
    double max;

    if (opModeIsActive()) {
      MoveCounts = p_distance * COUNTS_PER_INCH;
      newLeftBackTarget = BackLeft.getCurrentPosition() + MoveCounts;
      newRightBackTarget = BackRight.getCurrentPosition() + MoveCounts;
      newLeftFrontTarget = FrontLeft.getCurrentPosition() + MoveCounts;
      newRightFrontTarget = FrontRight.getCurrentPosition() + MoveCounts;
      BackLeft.setTargetPosition((int) newLeftBackTarget);
      BackRight.setTargetPosition((int) newRightBackTarget);
      FrontLeft.setTargetPosition((int) newLeftFrontTarget);
      FrontRight.setTargetPosition((int) newRightFrontTarget);
      BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      FrontLeft.setPower(p_speed);
      FrontRight.setPower(p_speed);
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      BackLeft.setPower(p_speed);
      BackRight.setPower(p_speed);
      while (opModeIsActive() && BackLeft.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy()) {
        error = getError(p_angle);
        steer = getSteer(error, P_DRIVE_COEFF);
        if (p_distance < 0) {
          steer = steer * -1;
        }
        leftSpeed = p_speed - steer;
        rightSpeed = p_speed + steer;
        if (Math.abs(leftSpeed) > Math.abs(rightSpeed)) {
          max = leftSpeed;
        } else {
          max = rightSpeed;
        }
        if (max > 1) {
          leftSpeed = leftSpeed / max;
          rightSpeed = rightSpeed / max;
        }
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        FrontLeft.setPower(leftSpeed);
        FrontRight.setPower(rightSpeed);
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        BackLeft.setPower(leftSpeed);
        BackRight.setPower(rightSpeed);
        telemetry.addData("Back Left", BackLeft.getCurrentPosition());
        telemetry.addData("Back Right", BackRight.getCurrentPosition());
        telemetry.addData("Front Right", FrontRight.getCurrentPosition());
        telemetry.addData("Front Left", FrontLeft.getCurrentPosition());
        telemetry.addData("BackLeftTarget", newLeftBackTarget);
        telemetry.addData("BackRightTarget", newRightBackTarget);
        telemetry.addData("RightFrontTarget", newRightFrontTarget);
        telemetry.addData("LeftFrontTarget", newLeftFrontTarget);
        telemetry.addData("Target", p_angle);
        telemetry.addData("Steer", steer);
        telemetry.addData("LeftSpeed", leftSpeed);
        telemetry.addData("RightSpeed", rightSpeed);
        telemetry.update();
      }
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      FrontLeft.setPower(0);
      FrontRight.setPower(0);
      // The Y axis of a joystick ranges from -1 in its topmost position
      // to +1 in its bottommost position. We negate this value so that
      // the topmost position corresponds to maximum forward power.
      BackLeft.setPower(0);
      BackRight.setPower(0);
      BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }
}
