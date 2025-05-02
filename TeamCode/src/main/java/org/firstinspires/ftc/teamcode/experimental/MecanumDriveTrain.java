package org.firstinspires.ftc.teamcode.experimental;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * MecanumDriveTrain - A class for driving a mecanum drivetrain using Road Runner.
 *
 * This class provides methods for controlling a mecanum drive both manually and with
 * Road Runner trajectories. The class handles:
 * - Initializing motors and IMU
 * - Direct control for teleop
 * - Road Runner actions for autonomous
 */
public class MecanumDriveTrain {
    // Hardware
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private IMU imu;

    // Motor names - change these to match your robot's configuration
    private static final String LEFT_FRONT_NAME = "leftFront";
    private static final String LEFT_REAR_NAME = "leftRear";
    private static final String RIGHT_FRONT_NAME = "rightFront";
    private static final String RIGHT_REAR_NAME = "rightRear";
    private static final String IMU_NAME = "imu";

    // Drive constants
    private static final double TICKS_PER_REV = 537.7; // GoBILDA 5202 Motor with 19.2:1 Gearbox
    private static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm Mecanum wheels
    private static final double TRACK_WIDTH = 14.5; // Distance between left and right wheels in inches
    private static final double WHEEL_BASE = 10.5; // Distance between front and back wheels in inches

    // Feedforward parameters (tune these for your robot)
    public static double kS = 0.14; // Static friction compensation
    public static double kV = 0.14; // Velocity feed forward
    public static double kA = 0.04; // Acceleration feed forward

    // Feedback parameters (tune these for your robot)
    public static double TRANSLATIONAL_PID_GAIN = 8.0;
    public static double HEADING_PID_GAIN = 8.0;

    // Road Runner integration
    private MecanumDrive roadRunnerDrive;

    // Position tracking
    private Pose2d currentPose = new Pose2d(0, 0, 0);

    /**
     * Constructor for the MecanumDriveTrain class.
     *
     * @param hardwareMap The robot hardware map
     */
    public MecanumDriveTrain(HardwareMap hardwareMap) {
        // Initialize with default pose at origin
        initHardware(hardwareMap);

        // Road Runner drive
        roadRunnerDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    /**
     * Constructor with custom starting pose for autonomous.
     *
     * @param hardwareMap The robot hardware map
     * @param startPose The starting Pose2d for the robot
     */
    public MecanumDriveTrain(HardwareMap hardwareMap, Pose2d startPose) {
        initHardware(hardwareMap);
        this.currentPose = startPose;

        // Road Runner drive
        roadRunnerDrive = new MecanumDrive(hardwareMap, startPose);
    }

    /**
     * Initialize hardware components.
     *
     * @param hardwareMap The robot hardware map
     */
    private void initHardware(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_NAME);

        // Set motor directions - adjust these based on your robot design
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        try {
            imu = hardwareMap.get(IMU.class, IMU_NAME);
            IMU.Parameters parameters = new IMU.Parameters(
                    AxesOrder.ZYX,
                    AxesReference.INTRINSIC,
                    AngleUnit.RADIANS,
                    0);
            imu.initialize(parameters);
        } catch (Exception e) {
            // If we can't get the IMU, print an error message
            System.out.println("IMU initialization failed: " + e.getMessage());
        }

        // Set bulk caching mode to improve performance
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Drive method for teleop control.
     *
     * @param drive Forward/backward power (-1.0 to 1.0)
     * @param strafe Left/right power (-1.0 to 1.0)
     * @param turn Rotation power (-1.0 to 1.0)
     * @param fieldCentric Whether to use field-centric control
     */
    public void drive(double drive, double strafe, double turn, boolean fieldCentric) {
        // Field-centric driving calculations
        if (fieldCentric) {
            double heading = getHeading();
            double rotatedX = strafe * Math.cos(-heading) - drive * Math.sin(-heading);
            double rotatedY = strafe * Math.sin(-heading) + drive * Math.cos(-heading);
            drive = rotatedY;
            strafe = rotatedX;
        }

        // Mecanum drive calculation
        double leftFrontPower = drive + strafe + turn;
        double leftRearPower = drive - strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double rightRearPower = drive + strafe - turn;

        // Normalize powers if any is > 1.0
        double max = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightRearPower))
        );
        if (max > 1.0) {
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     * Get the current heading from the IMU.
     *
     * @return Current heading in radians
     */
    public double getHeading() {
        try {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        } catch (Exception e) {
            return 0.0;
        }
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

    /**
     * Reset motor encoders to zero.
     */
    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get the current pose from Road Runner.
     *
     * @return Current pose
     */
    public Pose2d getPose() {
        return roadRunnerDrive.getPose();
    }

    /**
     * Set the current pose. Useful for resetting position at the start of autonomous.
     *
     * @param pose The new pose
     */
    public void setPose(Pose2d pose) {
        roadRunnerDrive.setPose(pose);
        this.currentPose = pose;
    }

    /**
     * Creates a Road Runner TrajectoryActionBuilder for autonomous path creation.
     *
     * @param startPose The starting pose for this trajectory
     * @return A TrajectoryActionBuilder
     */
    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        return roadRunnerDrive.actionBuilder(startPose);
    }

    /**
     * Follow a Road Runner trajectory.
     *
     * @param trajectory The trajectory to follow
     * @return An Action that will follow the trajectory
     */
    public Action followTrajectory(Trajectory trajectory) {
        return roadRunnerDrive.followTrajectory(trajectory);
    }

    /**
     * Turn to face a specific angle.
     *
     * @param angle The angle to turn to in radians
     * @return An Action that will turn to the specified angle
     */
    public Action turn(double angle) {
        return roadRunnerDrive.turn(angle);
    }

    /**
     * Move to a specific point on the field.
     *
     * @param endPosition The position to move to
     * @return An Action that will move to the specified point
     */
    public Action moveToPosition(Vector2d endPosition) {
        Pose2d currentPose = getPose();
        return actionBuilder(currentPose)
                .strafeTo(endPosition)
                .build();
    }

    /**
     * Creates a custom drive action that drives with specified powers for a duration.
     *
     * @param drive Forward/backward power
     * @param strafe Left/right power
     * @param turn Rotation power
     * @param durationMs The duration to drive in milliseconds
     * @return An Action that will drive with the specified powers for the duration
     */
    public Action driveForTime(double drive, double strafe, double turn, long durationMs) {
        return new Action() {
            private ElapsedTime timer = new ElapsedTime();
            private boolean isStarted = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isStarted) {
                    timer.reset();
                    isStarted = true;
                }

                if (timer.milliseconds() < durationMs) {
                    MecanumDriveTrain.this.drive(drive, strafe, turn, false);
                    return true;
                } else {
                    stop();
                    return false;
                }
            }
        };
    }

    /**
     * Inner class for creating a custom Action that follows a trajectory and waits for completion.
     */
    public class FollowTrajectoryAction implements Action {
        private final Trajectory trajectory;
        private boolean initialized = false;

        public FollowTrajectoryAction(Trajectory trajectory) {
            this.trajectory = trajectory;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                // Start following the trajectory
                roadRunnerDrive.followTrajectory(trajectory);
                initialized = true;
            }

            // Check if we're done following the trajectory
            if (roadRunnerDrive.isBusy()) {
                return true; // Still running
            } else {
                stop();
                return false; // Done
            }
        }
    }

    /**
     * Example of how to create a custom Action for any drivetrain behavior.
     * This one implements a simple point turn to a specific angle.
     */
    public class TurnToAngleAction implements Action {
        private final double targetAngle;
        private final double tolerance = Math.toRadians(1.0); // 1 degree tolerance
        private final double maxTurnPower = 0.5;

        public TurnToAngleAction(double targetAngleRadians) {
            this.targetAngle = targetAngleRadians;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double currentAngle = getHeading();
            double error = calculateAngleError(currentAngle, targetAngle);

            packet.put("Target Angle", Math.toDegrees(targetAngle));
            packet.put("Current Angle", Math.toDegrees(currentAngle));
            packet.put("Angle Error", Math.toDegrees(error));

            // Check if we're within tolerance
            if (Math.abs(error) < tolerance) {
                stop();
                return false; // We're done
            }

            // Simple P controller for turning
            double turnPower = error * HEADING_PID_GAIN;

            // Limit the turn power
            turnPower = Math.max(-maxTurnPower, Math.min(turnPower, maxTurnPower));

            // Drive with just the turn component
            drive(0, 0, turnPower, false);

            return true; // Keep running
        }

        private double calculateAngleError(double current, double target) {
            double error = target - current;
            // Normalize the error to be between -π and π
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            return error;
        }
    }
}