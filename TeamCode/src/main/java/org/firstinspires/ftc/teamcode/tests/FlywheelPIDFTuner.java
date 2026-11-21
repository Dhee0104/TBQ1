package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;

/**
 * =====================================================================
 * ITERATIVE FLYWHEEL RPM PIDF TUNER (Using FTC SDK 10.3+ Native Edge Detection)
 * =====================================================================
 * CONTROLS EXPLANATION:
 *  - D-Pad Left / Right : Cycle through parameters to edit
 *                         (Target RPM -> Kp -> Ki -> Kd -> Kf)
 *  - D-Pad Up / Down    : Add (+) or Subtract (-) active magnitude
 *  - Right Bumper       : Increase magnitude step-size (multiply by 10)
 *  - Left Bumper        : Decrease magnitude step-size (divide by 10)
 * =====================================================================
 */
@TeleOp(name = "Iterative Flywheel RPM PIDF Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends OpMode {

    private DcMotorEx motorLeft, motorRight;
    private PIDFController pidfController;

    // Motor Encoder Constant (Update to match your specific motor's CPR)
    // GoBILDA 5202 series (19.2:1) = 537.6 | REV HD Hex = 537.7
    private static final double TICKS_PER_REV = 537.6;

    // Tuning Parameters (RPM Mode)
    private double targetRPM = 2500.0;
    private double kp = 0.001;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0002;

    // Selection & Magnitude management
    private int selectedParamIndex = 0;
    private final String[] paramNames = {"Target RPM", "Kp", "Ki", "Kd", "Kf"};

    // Step sizes spanning powers of 10
    private final double[] magnitudes = {0.00001, 0.0001, 0.001, 0.01, 0.1, 1.0, 10.0, 100.0, 1000.0};
    private int magnitudeIndex = 3; // Starts at 0.01

    @Override
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightFlywheel");

        // Set opposing directions so both wheels spin in the same direction
        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidfController = new PIDFController(kp, ki, kd, kf);

        telemetry.addData("Status", "Initialized (SDK 10.3+ Native Edge Detection). Press Play.");
        telemetry.update();
    }

    @Override
    public void start() {
        pidfController.reset();
    }

    @Override
    public void loop() {
        // 1. Process inputs using built-in FTC SDK 10.3+ edge-detection methods directly
        handleInputs();

        // 2. Feed updated tuning gains into the PIDF controller live
        pidfController.setPIDF(kp, ki, kd, kf);

        // 3. Compute real-time feedback loop in RPM
        double avgTicksPerSec = (motorLeft.getVelocity() + motorRight.getVelocity()) / 2.0;
        double currentRPM = (avgTicksPerSec * 60.0) / TICKS_PER_REV;
        double power = pidfController.calculate(currentRPM, targetRPM);

        motorLeft.setPower(power);
        motorRight.setPower(power);

        // 4. Update live Telemetry Dashboard
        telemetry.addLine("=== ITERATIVE FLYWHEEL RPM TUNER ===");
        telemetry.addData("Active Magnitude", magnitudes[magnitudeIndex]);
        telemetry.addLine("----------------------------------------");

        for (int i = 0; i < paramNames.length; i++) {
            String prefix = (i == selectedParamIndex) ? " --> [ACTIVE] " : "      ";
            telemetry.addData(prefix + paramNames[i], getParamValue(i));
        }

        telemetry.addLine("----------------------------------------");
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM Error", targetRPM - currentRPM);
        telemetry.addData("Motor Power Output", power);

        telemetry.addLine("----------------------------------------");
        telemetry.addLine("CONTROLS: DPad L/R = Change Param | DPad U/D = Adjust Value");
        telemetry.addLine("          Bumpers (L/R) = Change Step Magnitude");
    }

    @Override
    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    /**
     * Handles inputs leveraging the native SDK 10.3+ edge detection methods
     * directly on gamepad1.
     */
    private void handleInputs() {
        double currentMag = magnitudes[magnitudeIndex];

        if (gamepad1.dpadUpWasPressed()) {
            adjustSelectedParam(currentMag);
        }
        if (gamepad1.dpadDownWasPressed()) {
            adjustSelectedParam(-currentMag);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            selectedParamIndex--;
            if (selectedParamIndex < 0) selectedParamIndex = paramNames.length - 1;
        }
        if (gamepad1.dpadRightWasPressed()) {
            selectedParamIndex++;
            if (selectedParamIndex >= paramNames.length) selectedParamIndex = 0;
        }
        if (gamepad1.leftBumperWasPressed()) {
            magnitudeIndex = Math.max(0, magnitudeIndex - 1);
        }
        if (gamepad1.rightBumperWasPressed()) {
            magnitudeIndex = Math.min(magnitudes.length - 1, magnitudeIndex + 1);
        }
    }

    private void adjustSelectedParam(double delta) {
        if (selectedParamIndex == 0) {
            targetRPM += delta * 1000.0;
            targetRPM = Math.max(0, targetRPM);
        } else {
            switch (selectedParamIndex) {
                case 1: kp = Math.max(0, kp + delta); break;
                case 2: ki = Math.max(0, ki + delta); break;
                case 3: kd = Math.max(0, kd + delta); break;
                case 4: kf = Math.max(0, kf + delta); break;
            }
        }
    }

    private double getParamValue(int index) {
        switch (index) {
            case 0: return targetRPM;
            case 1: return kp;
            case 2: return ki;
            case 3: return kd;
            case 4: return kf;
            default: return 0.0;
        }
    }
}