package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

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
public class AutonomousHandoff extends AutonomousBase {
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

    // Additional servo positions needed for this routine
    final double CLAW_ORIENTATION_UP = 0.90;
    final double CLAW_ORIENTATION_DOWN = 0.0;
    final double CLAW_GRAB_CLOSED = 0.3;
    final double CLAW_GRAB_OPEN = 0.0;
    final double CLAW_ROLL_NEUTRAL = 0.35;

    final double DEPOSIT_ORIENTATION_ALIGNED = 0.98;
    final double DEPOSIT_ORIENTATION_WALL = 0.25;
    final double DEPOSIT_ORIENTATION_RESET   = 0.4;
    final double DEPOSIT_GRAB_CLOSED         = 0.30;  // Depositor claw closed (capture piece)
    final double DEPOSIT_GRAB_OPEN           = 0.03;  // Depositor claw open (release piece)

    // Movement constants
    private static final double DRIVE_POWER = 0.3;
    private static final double STRAFE_POWER = 0.4;
    private static final int LIFT_HEIGHT = 1200;
    
    @Override
    public void runOpMode() {
        try {
            telemetry.addData("Status", "Initializing...");
            telemetry.update();
            
            // Initialize robot hardware
            initializeHardware();

            

            // ------------------------------
            // Initialize Drive and Lift Motors (Gamepad1 & gamepad1)
            // ------------------------------
            frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
            frontRight = hardwareMap.get(DcMotor.class, "front_right");
            backLeft   = hardwareMap.get(DcMotor.class, "back_left");
            backRight  = hardwareMap.get(DcMotor.class, "back_right");
            liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");


            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            liftMotor.setDirection(DcMotor.Direction.REVERSE);

            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // ------------------------------
            // Initialize Mechanism Servos (gamepad1)
            // ------------------------------
            clawRotate    = hardwareMap.get(Servo.class, "claw_rotate");
            clawGrab      = hardwareMap.get(Servo.class, "claw_grab");
            clawRoll      = hardwareMap.get(Servo.class, "claw_roll");
            depositRotate = hardwareMap.get(Servo.class, "deposit_rotate");
            depositGrab   = hardwareMap.get(Servo.class, "deposit_grab");
            clawRoll.setDirection(Servo.Direction.FORWARD);

            // Initialize starting positions
            initializeServoPositions();
            
            telemetry.addData("Status", "Initialization Complete");
            telemetry.addData("Touch Sensor", liftSensor.isPressed() ? "Pressed" : "Not Pressed");
            telemetry.update();
            
            waitForStart();
            
            if (opModeIsActive()) {
                // 1. Strafe left to align with bar
                telemetry.addData("Status", "Strafing Left");
                telemetry.update();
                strafeLeft(2000); // Time in milliseconds
                
                // 2. Move backwards to approach bar
                telemetry.addData("Status", "Moving Backward");
                telemetry.update();
                moveBackward(1250); // Time in milliseconds
                
                // 3. Perform handoff sequence
                telemetry.addData("Status", "Performing Handoff");
                telemetry.update();
                performHandoff();
                
                telemetry.addData("Status", "Autonomous Complete");
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Exception: " + e.getMessage());
            telemetry.update();
        } finally {
            stopRobot();
        }
    }
    
    private void initializeServoPositions() {
        clawGrab.setPosition(CLAW_GRAB_CLOSED);
        clawRotate.setPosition(CLAW_ORIENTATION_UP);
        clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
    }
    
    private void strafeLeft(long milliseconds) {
        frontLeft.setPower(STRAFE_POWER);
        backLeft.setPower(-STRAFE_POWER);
        frontRight.setPower(-STRAFE_POWER);
        backRight.setPower(STRAFE_POWER);
        sleep(milliseconds);
        stopRobot();
    }
    
    private void moveBackward(long milliseconds) {
        frontLeft.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);
        sleep(milliseconds);
        stopRobot();
    }
    
    private void performHandoff() {
        // Set claw pos
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        depositRotate.setPosition(DEPOSIT_ORIENTATION_WALL);
        sleep(100);
        liftMotor.setPower(1.0);
        sleep(600);
        liftMotor.setPower(1);
        sleep(500);
        liftMotor.setPower(0);
        resetLift();
    }
    
    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
} 