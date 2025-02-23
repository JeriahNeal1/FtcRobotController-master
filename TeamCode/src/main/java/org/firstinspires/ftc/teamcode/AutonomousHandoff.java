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
    final double CLAW_GRAB_CLOSED = 0.0;
    final double CLAW_GRAB_OPEN = 0.3;
    final double CLAW_ROLL_NEUTRAL = 0.35;

    final double DEPOSIT_ORIENTATION_ALIGNED = 0.98;
    final double DEPOSIT_ORIENTATION_WALL = 0.25;
    final double DEPOSIT_ORIENTATION_RESET   = 0.4;
    final double DEPOSIT_GRAB_CLOSED         = 0.0;  // Depositor claw closed (capture piece)
    final double DEPOSIT_GRAB_OPEN           = 0.2;  // Depositor claw open (release piece)

    // Movement constants
    private static final double TARGET_DISTANCE_MM = -152.4; // Negative for backward movement
    private static final double POSITION_TOLERANCE_MM = 5.0;
    private static final double MOVEMENT_POWER = 0.3;
    private static final int LIFT_HEIGHT = 14; // Encoder ticks for lift height
    
    // Odometry computer
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        try {
            // Initialize robot hardware using parent class method
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
            
            // Initialize and configure odometry
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            odo.setOffsets(-84.0, -168.0);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                   GoBildaPinpointDriver.EncoderDirection.FORWARD);
            
            // Reset odometry and IMU
            odo.resetPosAndIMU();
            
            // Ensure servos are in starting positions
            initializeServoPositions();
            
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Odometry Status", odo.getDeviceStatus());
            telemetry.update();
            
            waitForStart();
            
            if (opModeIsActive()) {
                // 1. Move backwards with precise odometry control
                moveBackwardWithOdometry(TARGET_DISTANCE_MM);
                
                // 2. Handoff sequence
                performHandoff();
                
                telemetry.addData("Status", "Autonomous Complete");
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("Error", "Exception: " + e.getMessage());
            telemetry.update();
        } finally {
            // Ensure motors are stopped
            stopRobot();
        }
    }
    
    private void initializeServoPositions() {
        // Set initial positions
        clawGrab.setPosition(CLAW_GRAB_OPEN);
        clawRotate.setPosition(CLAW_ORIENTATION_DOWN);
        clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
    }

    private void performHandoff() {
//        Set claw pos
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        depositRotate.setPosition(DEPOSIT_ORIENTATION_WALL);
        liftMotor.setPower(1.0);
        Sleeper.sleep(600);
        resetLift();
    }

    private void moveBackwardWithOdometry(double targetDistanceMM) {
        ElapsedTime timeout = new ElapsedTime();
        timeout.reset();
        
        // Get initial position
        odo.update();
        Pose2D initialPose = odo.getPosition();
        double startX = initialPose.getX(DistanceUnit.MM);
        
        while (opModeIsActive() && timeout.seconds() < 3.0) {
            odo.update();
            Pose2D currentPose = odo.getPosition();
            double currentX = currentPose.getX(DistanceUnit.MM);
            double distanceMoved = currentX - startX;
            
            // Calculate error and adjust power
            double error = targetDistanceMM - distanceMoved;
            double power = -Math.min(MOVEMENT_POWER, 
                                   Math.abs(error / 50.0)) * Math.signum(error);
            
            // Check if we've reached target
            if (Math.abs(error) < POSITION_TOLERANCE_MM) {
                stopRobot();
                break;
            }
            
            // Apply power to motors
            setDrivePowers(power);
            
            // Update telemetry
            telemetry.addData("Target Distance", targetDistanceMM);
            telemetry.addData("Current Distance", distanceMoved);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.update();
        }
        
        stopRobot();
    }
    
    private void setDrivePowers(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    
    private void stopRobot() {
        setDrivePowers(0);
    }
} 