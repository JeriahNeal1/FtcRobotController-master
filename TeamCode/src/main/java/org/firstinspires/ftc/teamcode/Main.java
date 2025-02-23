package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.List;
import java.util.Objects;

// Import the HuskyLensManager wrapper
// import org.firstinspires.ftc.teamcode.HuskyLensManager;

/**
 * Complex Arm Control OpMode with Improved Driver-Friendly Controls
 *
 * MAIN CONTROLS (gamepad1):
 *   Claw (main) functions on ABX:
 *     A: Toggle Claw Orientation (Down for pickup, Up for handoff)
 *     B: Toggle Claw Grab (Closed/Open)
 *     X: Toggle Claw Roll (Pickup angle/Neutral)
 *     Y (hold): Reset all claw settings to neutral (requires hold for 0.5 sec)
 *
 *   Deposit functions on the DPad:
 *     DPad Up: Toggle deposit claw orientation (aligned/not)
 *     DPad Down: Toggle deposit claw grab (closed/open)
 *     DPad Left/Right: Optional fine adjustments
 *
 *   Intake Extender (scissor mechanism) on Right Trigger:
 *     • rt < 10% → extender = 10%
 *     • 10% ≤ rt < 50% → smoothly scales to 50%
 *     • rt ≥ 50% → smoothly scales to 100%
 *
 *   Lift Control:
 *     Left Trigger controls lift-up (proportional).
 *     Y Button controls lift-down (fixed speed).
 *
 * DRIVE (Gamepad1):
 *   Tank drive with left joystick (left motors) and right joystick (right motors),
 *   using cubic scaling for smoother low-speed operation.
 */
@TeleOp(name = "Complex Arm Control", group = "Competition")
public class Main extends OpMode {

    // ------------------------------
    // Hardware Declarations (Drive, Lift & Mechanisms)
    // ------------------------------
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor liftMotor;

    // Mechanism Servos (gamepad1)
    Servo clawRotate;      // Claw orientation servo (rotate to face ground/up)
    Servo clawGrab;        // Claw grabber servo (open/close)
    Servo clawRoll;        // Claw roll servo (adjust pickup angle)
    Servo depositRotate;   // Depositor claw orientation servo
    Servo depositGrab;     // Depositor claw grabber servo
    Servo intakeExtend1;   // Intake extender servo 1 (scissor mechanism)
    Servo intakeExtend2;   // Intake extender servo 2 (mirrors servo 1)

    // ------------------------------
    // HuskyLens Manager (Wrapper for the GoBilda HuskyLens)
    // ------------------------------
    // private HuskyLensManager huskyLensManager;

    // ------------------------------
    // Toggle Declarations for gamepad1 Buttons (ABX & D-Pad)
    // ------------------------------
    Toggle mechA = new Toggle();         // Claw Orientation toggle (A)
    Toggle mechB = new Toggle();         // Claw Roll toggle (B
    Toggle mechX = new Toggle();         // Claw Grab toggle (X)
    // (Y is now processed with a hold timer for reset.)

    Toggle dpadUpToggle    = new Toggle(); // Depositor Claw Orientation (DPad Up)
    Toggle dpadDownToggle  = new Toggle(); // Depositor Claw Grab (DPad Down)
    Toggle dpadLeftToggle  = new Toggle(); // Optional fine adjustment (DPad Left)
    Toggle dpadRightToggle = new Toggle(); // Optional fine adjustment (DPad Right)

    // ------------------------------
    // Preset Positions for Servos
    // ------------------------------
    // Main Claw presets:
    final double CLAW_ORIENTATION_DOWN = 0.0;   // Claw faces down for pickup.
    final double CLAW_ORIENTATION_UP   = 0.90;   // Claw faces up for handoff.
    final double CLAW_GRAB_CLOSED      = 0.3;     // Claw grabber closed.
    final double CLAW_GRAB_OPEN        = 0.0;     // Claw grabber open.
    final double CLAW_ROLL_ANGLE       = 0;       // Claw roll for pickup angle.
    final double CLAW_ROLL_NEUTRAL     = 0.35;    // Neutral claw roll.
    final double CLAW_ROLL_FLIP        = 1;     // Flip for specimen transfer

    // Intake Extender presets (scissor mechanism):
    final double INTAKE_EXTENDER_HALF = 0.25;   // 50% extension.
    final double INTAKE_EXTENDER_RANGE = 0.36;

    // Depositor presets:
    final double DEPOSIT_ORIENTATION_ALIGNED = 0.98;
    final double DEPOSIT_ORIENTATION_WALL = 0.25;
    final double DEPOSIT_ORIENTATION_RESET   = 0.4;
    final double DEPOSIT_GRAB_CLOSED         = 0.30;
    final double DEPOSIT_GRAB_OPEN           = 0.03;

    // ------------------------------
    // Y Button Reset Hold Timer
    // ------------------------------
    private double yHoldStartTime = 0;
    private final double Y_RESET_HOLD_THRESHOLD = 0.5; // seconds

    // Example voltage threshold (adjust as needed)
    final double SAFE_VOLTAGE_THRESHOLD = 11.0;

    // Additional member variables...
    private double lastLeftPower = 0.0;
    private double lastRightPower = 0.0;
    private double lastLoopTime = 0.0;
    final private double minIntakePosition = 0.05;
    final private double intakeRestPos = 0.22;

    private double lastIntakeCommandedPosition = INTAKE_EXTENDER_HALF;  // starting from half-extended
    private double intakeExtenderPosition = INTAKE_EXTENDER_RANGE;
    private double intakeWatchdogStartTime = 0.0;
    private final double INTAKE_MOVEMENT_TIMEOUT = 2.0; // seconds allowed for movement

    // Ramping constants (change according to your testing)
    private final double RAMP_RATE = 1.0;        // maximum motor power change per second
    private final double SERVO_RAMP_RATE = 0.5;    // maximum servo position change per second
    private final double SERVO_TOLERANCE = 0.01;   // tolerance for servo target accuracy
    // Claw Roll Logic
    public boolean claw_rolling = false;
    public double claw_roll_degrees = 0;

    public String deposit_position = "aligned";

    // Lift-related constants
    final int DEPOSIT_LIFT_HEIGHT = 1200; // Target encoder ticks for deposit height

    // For detecting a rising edge on gamepad2.a (to trigger the auto pickup routine only once per press)
    private boolean prevGamepad2A = false;
    // For detecting a rising edge on gamepad2.y (to trigger the auto flip routine only once per press)
    private boolean prevGamepad2Y = false;

    public boolean opModeIsActive() {
        return true;
    }

    public static class Toggle {
        public boolean toggled = false;
        public boolean previous = false;

        /**
         * Update the toggle state; on a rising edge, flip the state.
         */
        public void update(boolean current) {
            if (current && !previous) {
                toggled = !toggled;
            }
            previous = current;
        }

        @Override
        public String toString() {
            return String.valueOf(toggled);
        }
    }

    /**
     * autoIntakeAndDepositPiece()
     *
     * This function performs a high-level automated routine to:
     *   1) Confirm the AI camera sees a piece
     *   2) Extend + rotate claw for pickup
     *   3) Grab the piece
     *   4) Retract + hand off to depositor
     *   5) Raise + drop the piece in the bucket
     *
     * NOTE: This is a blocking routine for demonstration.
     * In a real TeleOp, consider using a state machine or
     * non-blocking approach for smoother driver control.
     */
    /**
     * Helper function to move the intake servo system to a specific position.
     *
     * @param position A value between 0.0 and 1.0 representing the desired servo position.
     */
    private void moveIntakeTo(double position) {
        double target = Math.max(0.0, Math.min(1.0, position));
        intakeExtend1.setPosition(target);
        intakeExtend2.setPosition(target);
    }

    // Odometry Computer: GoBilda Pinpoint device configured in the robot configuration as "odo"
    GoBildaPinpointDriver odo;

    FtcDashboard dashboard;

    @Override
    public void init() {
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
        intakeExtend1 = hardwareMap.get(Servo.class, "intake_extend1");
        intakeExtend2 = hardwareMap.get(Servo.class, "intake_extend2");

        intakeExtend1.setDirection(Servo.Direction.REVERSE);
        intakeExtend2.setDirection(Servo.Direction.FORWARD);
        clawRoll.setDirection(Servo.Direction.FORWARD);

        // ------------------------------
        // Initialize the GoBilda® Pinpoint Odometry Computer
        // ------------------------------
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.resetPosAndIMU();

        // ------------------------------
        // Initialize the HuskyLens via Wrapper
        // ------------------------------
        // "husky_lens" must match the device name from your robot configuration.
        // ------------------------------
        // Initialize Dashboard
        // ------------------------------
        dashboard = FtcDashboard.getInstance();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        double deltaTime = currentTime - lastLoopTime;
        lastLoopTime = currentTime;

        // ------------------------------
        // Update Odometry
        // ------------------------------
        odo.update();
        Pose2D pose = odo.getPosition();
        telemetry.addData("Pose", "{X: %.2f mm, Y: %.2f mm, H: %.2f°}",
                pose.getX(DistanceUnit.MM), pose.getY(DistanceUnit.MM), pose.getHeading(AngleUnit.DEGREES));

        // // ------------------------------
        // // Update HuskyLens via the Wrapper
        // // ------------------------------
        // String data = huskyLensManager.getData();
        // telemetry.addData("HuskyLens Data", data);

        // TelemetryPacket packet = new TelemetryPacket();
        // packet.put("HuskyLens Data", data);
        // dashboard.sendTelemetryPacket(packet);

        // ------------------------------
        // Safety Mode (monitor battery)
        // ------------------------------
        VoltageSensor batterySensor = hardwareMap.voltageSensor.iterator().next();
        double batteryVoltage = batterySensor.getVoltage();
        boolean safetyMode = batteryVoltage < SAFE_VOLTAGE_THRESHOLD;
        double safetyScaleFactor = safetyMode ? 0.6 : 1.0;

        // ------------------------------
        // Drive Controls with Tank Drive and Strafing
        // ------------------------------
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        double strafePower = 0;

        if (gamepad1.left_bumper) {
            strafePower = 1;
        } else if (gamepad1.right_bumper) {
            strafePower = -1;
        }

        frontLeft.setPower((leftPower + strafePower) * safetyScaleFactor);
        backLeft.setPower((leftPower - strafePower) * safetyScaleFactor);
        frontRight.setPower((rightPower - strafePower) * safetyScaleFactor);
        backRight.setPower((rightPower + strafePower) * safetyScaleFactor);

        // ------------------------------
        // Update Toggle States for Mechanisms (gamepad1)
        // ------------------------------
        mechA.update(gamepad1.a);
        mechB.update(gamepad1.b);
        mechX.update(gamepad1.x);

        dpadUpToggle.update(gamepad1.dpad_up);
        dpadLeftToggle.update(gamepad1.dpad_left);
        dpadRightToggle.update(gamepad1.dpad_right);

        // ------------------------------
        // Process Y Button Reset (Requires a Hold)
        // ------------------------------
        boolean yReset = false;
        if (gamepad1.y) {
            if (yHoldStartTime == 0) {
                yHoldStartTime = getRuntime();
            } else if (getRuntime() - yHoldStartTime >= Y_RESET_HOLD_THRESHOLD) {
                yReset = true;
            }
        } else {
            yHoldStartTime = 0;
        }

        // ------------------------------
        // Main Claw Mechanism Actions
        // ------------------------------
        if (yReset) {
            clawRotate.setPosition(CLAW_ORIENTATION_UP);
            clawGrab.setPosition(CLAW_GRAB_OPEN);
            clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
            mechA.toggled = false;
            ;
            mechB.toggled = false;
            mechX.toggled = false;
        } else {
            if (mechA.toggled) {
                clawRotate.setPosition(CLAW_ORIENTATION_DOWN);
            } else {
                clawRotate.setPosition(CLAW_ORIENTATION_UP);
            }

            if (mechX.toggled) {
                clawGrab.setPosition(CLAW_GRAB_OPEN);
            } else {
                clawGrab.setPosition(CLAW_GRAB_CLOSED);
            }

            double joystickX = gamepad2.right_stick_x;
            final double DEADZONE = 0.05;

            if (Math.abs(joystickX) > DEADZONE) {
                claw_rolling = true;
            } else if (gamepad1.b) {
                claw_rolling = false;
            }

            if (!claw_rolling) {
                if (mechB.toggled) {
                    clawRoll.setPosition(CLAW_ROLL_ANGLE);
                } else {
                    clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
                }
            }
        }
        // ------------------------------
        // Intake Extender Control (Right Trigger on Gamepad1)
        // ------------------------------
        double rawRt = gamepad1.right_trigger;
        double triggerDeadzone = 0.05;
        double filteredRT = Math.abs(rawRt) < triggerDeadzone ? 0 : rawRt;

        double targetExtenderPosition = scaleExtenderInput(filteredRT);
        intakeExtenderPosition = rampServoValue(targetExtenderPosition, lastIntakeCommandedPosition, deltaTime);
        lastIntakeCommandedPosition = intakeExtenderPosition;

        intakeExtend1.setPosition(intakeExtenderPosition);
        intakeExtend2.setPosition(intakeExtenderPosition);

        // ------------------------------
        // Depositor Mechanism Controls (Now on gamepad2)
        // ------------------------------
        dpadDownToggle.update(gamepad2.dpad_down);

        if (gamepad2.dpad_up) {
            deposit_position = "reset";
        } else if (gamepad2.dpad_left) {
            deposit_position = "wall";
        } else if (gamepad2.dpad_right) {
            deposit_position = "aligned";
        }

        if (Objects.equals(deposit_position, "aligned")) {
            depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
        } else if (Objects.equals(deposit_position, "wall")) {
            depositRotate.setPosition(DEPOSIT_ORIENTATION_WALL);
        } else if (Objects.equals(deposit_position, "reset")) {
            depositRotate.setPosition(DEPOSIT_ORIENTATION_RESET);
        }

        if (dpadDownToggle.toggled) {
            depositGrab.setPosition(DEPOSIT_GRAB_OPEN);
        } else {
            depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        }

//        // Manual claw roll control using gamepad2 left stick X
//        final double ROLL_ADJUSTMENT_SPEED = 0.002;
//        double leftStickX = gamepad2.left_stick_x;
//        if (Math.abs(leftStickX) > 0.05) {
//            double currentPos = clawRoll.getPosition();
//            clawRoll.setPosition(Math.min(1, Math.max(0, currentPos + (leftStickX * ROLL_ADJUSTMENT_SPEED))));
//            claw_rolling = true;
//        }

        // ------------------------------
        // Lift Control (Now on gamepad2 - Right stick Y for up/down)
        // ------------------------------
        double rawLiftSpeed = 0;
        double liftStickY = -gamepad2.right_stick_y;

        if (Math.abs(liftStickY) > 0.05) {
            rawLiftSpeed = liftStickY;
        }

        if (Math.abs(rawLiftSpeed) > 0.05) {
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setPower(rawLiftSpeed);
        } else {
            liftMotor.setPower(0);
        }

        // ------------------------------
        // Auto-Grab (A on gamepad2)
        // ------------------------------
        if (gamepad2.a && !prevGamepad2A) {
            autoIntakeAndDepositPiece();
        }
        prevGamepad2A = gamepad2.a;

        if (gamepad2.y && !prevGamepad2Y) {
            autoIntakeAndDepositSpecimen();
        }
        prevGamepad2Y = gamepad2.y;
        // ------------------------------
        // Telemetry for Driver Feedback
        // ------------------------------
        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Safety Mode", safetyMode);
        telemetry.addData("Left Power", "%.2f", leftPower);
        telemetry.addData("Right Power", "%.2f", rightPower);
        telemetry.addData("Raw RT", "%.2f", rawRt);
        telemetry.addData("Target Extender Pos", "%.2f", targetExtenderPosition);
        telemetry.addData("Commanded Extender Pos", "%.2f", intakeExtenderPosition);
        telemetry.addData("Delta Time", "%.2f", deltaTime);
        telemetry.addData("Claw Orientation (A)", mechA.toggled);
        telemetry.addData("Claw Grab (B)", mechB.toggled);
        telemetry.addData("Claw Roll (X)", mechX.toggled);
        telemetry.addData("Y Reset Held", yReset);
        telemetry.addData("Deposit Orientation (DPad Up)", dpadUpToggle.toggled);
        telemetry.addData("Deposit Grab (DPad Down)", dpadDownToggle.toggled);
        telemetry.update();
    }

    /**
     * Cubic scaling for drive input for refined control at low speeds.
     */
    private double scaleDriveInput(double input) {
        return input * input * input;
    }

    /**
     * Scale the right trigger input into the allowed intake extender range.
     */
    private double scaleExtenderInput(double rt) {
        double min = Math.max(minIntakePosition, 0.1);  // Use the larger of minIntakePosition or 0.1
        double max = INTAKE_EXTENDER_RANGE;
        double mid = (min + max) / 2.0;

        if (rt < 0.1) {
            return min;
        } else if (rt < 0.5) {
            double normalized = (rt - 0.1) / 0.4;
            double scaled = normalized * normalized;
            return min + scaled * (mid - min);
        } else {
            double normalized = (rt - 0.5) / 0.5;
            double scaled = normalized * normalized;
            return mid + scaled * (max - mid);
        }
    }

    // Helper function for servo ramping
    private double rampServoValue(double target, double current, double deltaTime) {
        double maxDelta = 0.5 * deltaTime;
        double delta = target - current;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return Math.max(minIntakePosition, current + delta);
    }

    /**
     * Performs the auto routine for picking up and depositing a game piece.
     * It uses encoder-controlled lift movement and resets the lift using the TouchSensor.
     */
    public void autoIntakeAndDepositPiece() {
        telemetry.addData("Auto-Grab", "Target detected! Starting routine.");
        telemetry.update();

        // 0) Set Motor powers to zero to ensure robot does not drift from original position
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // 1) Extend & Orient Intake for Pickup
        moveIntakeTo(0.2);
        depositGrab.setPosition(DEPOSIT_GRAB_OPEN);
        clawRotate.setPosition(CLAW_ORIENTATION_DOWN);
        clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
        Sleeper.sleep(150);

        depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
        moveIntakeTo(0);
        Sleeper.sleep(525);

        // 4) Handoff to the Depositor
        depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
        Sleeper.sleep(225);
        clawGrab.setPosition(CLAW_GRAB_OPEN);
        moveIntakeTo(0.2);
        Sleeper.sleep(200);
//        Make sure to set toggles properly so that servos retain position after sequence finishes
        dpadDownToggle.toggled = false;
        mechA.toggled = true;
        mechB.toggled = false;
        mechX.toggled = true;
        clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
    }
public void autoIntakeAndDepositSpecimen() {
    telemetry.addData("Auto-Bar", "Target detected! Starting routine.");
    telemetry.update();

    // 0) Set Motor powers to zero to ensure robot does not drift from original position
    frontLeft.setPower(0);
    backLeft.setPower(0);
    frontRight.setPower(0);
    backRight.setPower(0);

    // 1) Extend & Orient Intake for Pickup
    moveIntakeTo(0.28);
    depositGrab.setPosition(DEPOSIT_GRAB_OPEN);
    clawRotate.setPosition(CLAW_ORIENTATION_DOWN);
    clawRoll.setPosition(CLAW_ROLL_FLIP);
    Sleeper.sleep(275);

    depositRotate.setPosition(DEPOSIT_ORIENTATION_ALIGNED);
    moveIntakeTo(0);
    Sleeper.sleep(525);

    // 4) Handoff to the Depositor
    depositGrab.setPosition(DEPOSIT_GRAB_CLOSED);
    Sleeper.sleep(200);
    clawGrab.setPosition(CLAW_GRAB_OPEN);

//        Make sure to set toggles properly so that servos retain position after sequence finishes
    moveIntakeTo(0.25);
    dpadDownToggle.toggled = false;
    mechA.toggled = true;
    mechB.toggled = false;
    mechX.toggled = true;
Sleeper.sleep(50);

    // Offsetting claw rotation time to allow outtake arm to escape
    clawRoll.setPosition(CLAW_ROLL_NEUTRAL);
}
}