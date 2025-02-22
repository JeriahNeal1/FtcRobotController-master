package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousBase extends LinearOpMode {
    // Hardware declarations
    protected DcMotor frontLeft, frontRight, backLeft, backRight, liftMotor;
    protected Servo clawRotate, clawGrab, clawRoll, depositRotate, depositGrab;
    protected TouchSensor liftSensor;
    
    // Constants from your Main.java
    protected final double DEPOSIT_ORIENTATION_WALL = 0.25;
    protected final double DEPOSIT_GRAB_CLOSED = 0.13;
    protected final double DEPOSIT_GRAB_OPEN = 0.0;
    
    // Movement constants
    private final double TICKS_PER_REV = 537.7;  // For GoBilda 5202 series
    private final double WHEEL_DIAMETER_MM = 96.0;
    private final double TICKS_PER_MM = TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);

    double leftPower, rightPower = 1.0;
    double strafePower = 1;
    
    protected void initializeHardware() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        
        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        
        // Initialize servos
        clawRotate = hardwareMap.get(Servo.class, "claw_rotate");
        clawGrab = hardwareMap.get(Servo.class, "claw_grab");
        clawRoll = hardwareMap.get(Servo.class, "claw_roll");
        depositRotate = hardwareMap.get(Servo.class, "deposit_rotate");
        depositGrab = hardwareMap.get(Servo.class, "deposit_grab");
        
        // Initialize lift sensor
        liftSensor = hardwareMap.get(TouchSensor.class, "lift_touch_sensor");
        
        // Configure motors for position control
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set zero power behavior
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    protected void resetLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(-0.5);
        
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        while (!liftSensor.isPressed() && timeout.seconds() < 5.0 && opModeIsActive()) {
            telemetry.addData("Status", "Resetting lift...");
            telemetry.update();
        }
        
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    protected void moveLift(int targetPosition, double power) {
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        
        while (liftMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
            telemetry.update();
        }
        
        liftMotor.setPower(0);
    }
    
    protected void depositPiece() {
        // Raise lift
        moveLift(1200, 0.8);
        
        // Position depositor for wall
        depositRotate.setPosition(DEPOSIT_ORIENTATION_WALL);
        sleep(500);
        
        // Open depositor
        depositGrab.setPosition(DEPOSIT_GRAB_OPEN);
        sleep(300);
        
        // Reset lift
        resetLift();
    }

        protected void moveRobotForward(int millimeters) {

        }

        protected void moveRobotBack(int millimeters) {
            
        }

        protected void moveRobotRight(int millimeters) {
            strafePower = -1;
    
            frontLeft.setPower(leftPower + strafePower);
            backLeft.setPower(leftPower - strafePower);
            frontRight.setPower(rightPower - strafePower);
            backRight.setPower(rightPower + strafePower);
        }

        protected void moveRobotLeft(int millimeters) {
            strafePower = 1;
    
            frontLeft.setPower(leftPower + strafePower);
            backLeft.setPower(leftPower - strafePower);
            frontRight.setPower(rightPower - strafePower);
            backRight.setPower(rightPower + strafePower);
        }

    // More movement functions in next message...
} 