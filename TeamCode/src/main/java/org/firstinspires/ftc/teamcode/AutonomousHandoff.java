package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This Autonomous OpMode performs the following actions:
 *  1. Moves backwards (approximately half a foot).
 *  2. Hands off a game piece from the intake grabber to the outtake claw (deposit mechanism).
 *  3. Raises the lift all the way up.
 *  4. Flips the deposit mechanism to reset position.
 *  5. Opens the deposit claw to deposit the piece.
 *  6. Closes the intake claw at the end.
 */
@Autonomous(name = "Autonomous Handoff", group = "Competition")
public class AutonomousHandoff extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    // Lift motor
    private DcMotor liftMotor;
    // Mechanism servos
    private Servo clawRotate;    // Intake mechanism orientation servo
    private Servo clawGrab;      // Intake grabber (holds game piece)
    private Servo clawRoll;      // (Optional, not used in this sequence)
    private Servo depositRotate; // Depositor mechanism orientation servo
    private Servo depositGrab;   // Depositor claw (for outtake)
    // (Optional: intake extender servos if needed)
    // private Servo intakeExtend1, intakeExtend2;

    // Preset positions (adjust to your robot's calibration)
    final double CLAW_ORIENTATION_UP   = 0.90;  // Default intake claw up
    final double CLAW_ORIENTATION_DOWN = 0.0;   // Intake claw down (if needed)
    final double CLAW_GRAB_CLOSED      = 0.0;   // Intake claw closed (holding piece)
    final double CLAW_GRAB_OPEN        = 0.3;   // Intake claw open (release piece)
    
    final double DEPOSIT_ORIENTATION_ALIGNED = 1.0;  // Depositor aligned for handoff
    final double DEPOSIT_ORIENTATION_RESET   = 0.4;  // Depositor reset (flipped back)
    final double DEPOSIT_GRAB_CLOSED         = 0.0;  // Depositor claw closed (capture piece)
    final double DEPOSIT_GRAB_OPEN           = 0.2;  // Depositor claw open (release piece)
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors
        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");
        liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");

        // Set motor directions (adjust according to your robot configuration)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos
        clawRotate    = hardwareMap.get(Servo.class, "claw_rotate");
        clawGrab      = hardwareMap.get(Servo.class, "claw_grab");
        clawRoll      = hardwareMap.get(Servo.class, "claw_roll");
        depositRotate = hardwareMap.get(Servo.class, "deposit_rotate");
        depositGrab   = hardwareMap.get(Servo.class, "deposit_grab");
        // If using intake extender, initialize here:
        // intakeExtend1 = hardwareMap.get(Servo.class, "intake_extend1");
        // intakeExtend2 = hardwareMap.get(Servo.class, "intake_extend2");

        // Configure lift motor for a basic timed lift (could switch to RUN_TO_POSITION if you have targets)
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial servo positions:
        // Intake claw starts closed holding a piece.
        clawRotate.setPosition(CLAW_ORIENTATION_UP);
        clawGrab.setPosition(CLAW_GRAB_OPEN);
        // Deposit mechanism now starts aligned for handoff and deposit claw is closed to grab the piece.
        depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // --- STEP 1: Move Robot Backwards ---
        telemetry.addData("Step", "Moving Forward");
        telemetry.update();
        // Negative power for backwards
        setDrivePower(-0.5, -0.5);
        sleep(500); // Adjust this duration until the robot has moved roughly half a foot.
        setDrivePower(0, 0);
        sleep(250);
    }
    /**
     * Helper method to set power to both sides of the drive train.
     *
     * @param leftPower  power for left side motors
     * @param rightPower power for right side motors
     */
    private void setDrivePower(double leftPower, double rightPower) {
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }
} 