package org.firstinspires.ftc.teamcode.experimental;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/***
 * FTC subsystem for handling game pieces in and out the robot. Includes:
 * A motor to extend/intake a slide mechanism
 * A servo in continuous mode attached to the slide mechanism to intake/deploy game pieces
 ***/

public class IntakeSubsystem {

    // Declare motors & servos
    private Servo endEffectorMotor = null;
    private DcMotorEx slideMotor = null;

    // Declare constants
    // TODO: Test endEffector power
    private double EFFECTOR_IN_POWER = 1.0;
    private double EFFECTOR_OUT_POWER = -1.0;
    private double EFFECTOR_OFF_POWER = 0.0;

    private double SLIDE_IN_POWER = 0.25;
    private double SLIDE_OUT_POWER = -0.25;
}
