package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * =====================================================================
 * BASIC SERVO TEST OPMODE
 * =====================================================================
 * CONTROLS EXPLANATION:
 *  - D-Pad Up   : Increase servo position by +0.05 (Max: 1.0)
 *  - D-Pad Down : Decrease servo position by -0.05 (Min: 0.0)
 *  - A Button   : Jump position instantly to 0.0 (Closed / Home)
 *  - Y Button   : Jump position instantly to 1.0 (Open / Max)
 * =====================================================================
 */
@TeleOp(name = "Basic Servo Test", group = "Tuning")
public class servotest extends OpMode {

    private Servo testServo;

    // Starting position (usually 0.0 or 0.5)
    private double servoPosition = 0.5;

    // Step size for fine-tuning position per D-pad click
    private static final double STEP_SIZE = 0.05;

    @Override
    public void init() {
        // Retrieve servo from hardware map (change "testServo" to match your Config name)
        testServo = hardwareMap.get(Servo.class, "testServo");

        // Set initial position
        testServo.setPosition(servoPosition);

        telemetry.addData("Status", "Initialized Servo Test. Press Play.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Handle inputs using SDK 10.3+ edge detection
        if (gamepad1.dpadUpWasPressed()) {
            servoPosition = Math.min(1.0, servoPosition + STEP_SIZE);
        }
        if (gamepad1.dpadDownWasPressed()) {
            servoPosition = Math.max(0.0, servoPosition - STEP_SIZE);
        }

        // Quick preset buttons for testing common extremes
        if (gamepad1.aWasPressed()) {
            servoPosition = 0.0;
        }
        if (gamepad1.yWasPressed()) {
            servoPosition = 1.0;
        }

        // 2. Apply position to the physical servo
        testServo.setPosition(servoPosition);

        // 3. Telemetry Dashboard
        telemetry.addLine("=== BASIC SERVO TESTER ===");
        telemetry.addData("Current Position", "%.2f", servoPosition);
        telemetry.addLine("----------------------------------------");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  DPad Up/Down : Adjust position by ±0.05");
        telemetry.addLine("  A Button     : Set to 0.0 (Home/Closed)");
        telemetry.addLine("  Y Button     : Set to 1.0 (Max/Open)");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Optional safety behavior on stop
    }
}