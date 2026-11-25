package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;

/**
 - =====================================================================
 - SINGLE-MOTOR FLYWHEEL RPM PIDF TUNER (SDK 10.3+)
 - =====================================================================
 - CONTROLS EXPLANATION:
 -  - Triggers (L / R)   : Make flywheel move slower / faster (adjusts Target RPM)
 -  - D-Pad Left / Right : Cycle through parameters to edit (Kp -> Ki -> Kd -> Kf)
 -  - D-Pad Up / Down    : Increase or decrease the active magnitude step-size
 -  - Bumpers (L / R)    : Subtract (-) or Add (+) selected magnitude to active gain
 -  - Button A           : Starts up / enables the PIDF flywheel control loop
 -  - Button B           : Turns off the motor
 - =====================================================================
 */
@TeleOp(name = "Single Motor Flywheel Tuner", group = "Tuning")
public class FlywheelPIDFTuner extends OpMode {

    private DcMotorEx motor;
    private PIDFController pidfController;

    private static final double TICKS_PER_REV = 28; // GoBILDA 1:1

    // Tuning Parameters
    private double targetRPM = 2500.0;
    private double kp = 0.000;
    private double ki = 0.0;
    private double kd = 0.0;
    private double kf = 0.0002;

    // Motor state toggle
    private boolean motorEnabled = false;

    // Parameter selection (Kp, Ki, Kd, Kf)
    private int selectedParamIndex = 0;
    private final String[] paramNames = {"Kp", "Ki", "Kd", "Kf"};

    // Magnitude step sizes
    private final double[] magnitudes = {0.00001, 0.0001, 0.001, 0.01, 0.1, 1.0};
    private int magnitudeIndex = 2; // Starts at 0.001

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidfController = new PIDFController(kp, ki, kd, kf);

        telemetry.addData("Status", "Initialized. Press A to Start, B to Stop.");
        telemetry.update();
    }

    @Override
    public void start() {
        pidfController.reset();
    }

    @Override
    public void loop() {
        // 1. Handle Gamepad Inputs using SDK 10.3+ native methods
        handleInputs();

        // 2. Update controller gains live
        pidfController.setPIDF(kp, ki, kd, kf);

        // 3. Compute current RPM
        double ticksPerSec = motor.getVelocity();
        double currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV;

        // 4. Drive motor if enabled, otherwise cut power
        double power = 0.0;
        if (motorEnabled) {
            power = pidfController.calculate(currentRPM, targetRPM);
            motor.setPower(power);
        } else {
            motor.setPower(0);
            pidfController.reset();
        }

        // 5. Telemetry Dashboard with High Precision Formatting
        telemetry.addLine("=== SINGLE MOTOR FLYWHEEL TUNER ===");
        telemetry.addData("Motor Status", motorEnabled ? "RUNNING" : "STOPPED (Press A)");
        telemetry.addData("Target RPM", "%.1f", targetRPM);
        telemetry.addData("Current RPM", "%.1f", currentRPM);
        telemetry.addData("RPM Error", "%.1f", targetRPM - currentRPM);
        telemetry.addData("Power Output", "%.4f", power);
        telemetry.addLine("----------------------------------------");
        telemetry.addData("Active Step Magnitude", "%.6f", magnitudes[magnitudeIndex]);

        for (int i = 0; i < paramNames.length; i++) {
            String prefix = (i == selectedParamIndex) ? " --> [ACTIVE] " : "      ";
            telemetry.addData(prefix + paramNames[i], String.format("%.6f", getGainValue(i)));
        }

        telemetry.addLine("----------------------------------------");
        telemetry.addLine("CONTROLS: Triggers = Adjust RPM | A = Start | B = Stop");
        telemetry.addLine("          DPad L/R = Select Gain | DPad U/D = Change Magnitude");
        telemetry.addLine("          Bumpers = Subtract (-) / Add (+) Magnitude");
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    private void handleInputs() {
        if (gamepad1.aWasPressed()) {
            motorEnabled = true;
        }
        if (gamepad1.bWasPressed()) {
            motorEnabled = false;
        }

        if (gamepad1.rightTriggerWasPressed()) {
            targetRPM += 50;
        }
        if (gamepad1.leftTriggerWasPressed()) {
            targetRPM -= 50;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            selectedParamIndex--;
            if (selectedParamIndex < 0) selectedParamIndex = paramNames.length - 1;
        }
        if (gamepad1.dpadRightWasPressed()) {
            selectedParamIndex++;
            if (selectedParamIndex >= paramNames.length) selectedParamIndex = 0;
        }

        if (gamepad1.dpadUpWasPressed()) {
            magnitudeIndex = Math.min(magnitudes.length - 1, magnitudeIndex + 1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            magnitudeIndex = Math.max(0, magnitudeIndex - 1);
        }

        double currentMag = magnitudes[magnitudeIndex];
        if (gamepad1.rightBumperWasPressed()) {
            adjustActiveGain(currentMag);
        }
        if (gamepad1.leftBumperWasPressed()) {
            adjustActiveGain(-currentMag);
        }
    }

    private void adjustActiveGain(double delta) {
        switch (selectedParamIndex) {
            case 0: kp = Math.max(0, kp + delta); break;
            case 1: ki = Math.max(0, ki + delta); break;
            case 2: kd = Math.max(0, kd + delta); break;
            case 3: kf = Math.max(0, kf + delta); break;
        }
    }

    private double getGainValue(int index) {
        switch (index) {
            case 0: return kp;
            case 1: return ki;
            case 2: return kd;
            case 3: return kf;
            default: return 0.0;
        }
    }
}