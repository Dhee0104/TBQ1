package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.turret; // Adjust package name if needed

@TeleOp(name = "Turret Standalone  2", group = "Tuning")
public class turretTest extends OpMode {

    private turret turret;

    // Virtual goal coordinates on the field (inches), starting relative to robot at (0,0)
    private double testGoalX = -50;
    private double testGoalY = 0;
    private final double stepSize = 2.0; // Inches per D-pad press

    @Override
    public void init() {
        // 1. Initialize Turret Servos
        Servo servo1 = hardwareMap.get(Servo.class, "turretServo1");
        Servo servo2 = hardwareMap.get(Servo.class, "turretServo2");
        turret = new turret(servo1, servo2);
        turret.init();

        telemetry.addData("Status", "Turret Standalone Test Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Adjust Virtual Goal X and Y using D-pad inputs (with basic debounce control via pressed checks if using SDK 10.3+, or standard triggers)
        if (gamepad1.dpadUpWasPressed()) {
            testGoalY += stepSize;
        }
        if (gamepad1.dpadDownWasPressed()) {
            testGoalY -= stepSize;
        }
        if (gamepad1.dpadRightWasPressed()) {
            testGoalX += stepSize;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            testGoalX -= stepSize;
        }

        // 2. Assume Robot is fixed at (0,0) facing 0 radians (straight ahead)
        double robotX = 0.0;
        double robotY = 0.0;
        double robotHeading = 0.0;

        // 3. Calculate dX and dY relative to robot position (0,0)
        double dX = testGoalX - robotX;
        double dY = testGoalY - robotY;

        // 4. Calculate and command turret angle
        double targetAngle = turret.calculateTargetAngle(dX, dY, robotHeading);
        turret.setAngle(targetAngle);

        // 5. Telemetry Dashboard
        telemetry.addLine("=== TURRET STANDALONE TESTER ===");
        telemetry.addData("Robot Fixed Position", "0.0, 0.0 in (Heading: 0.0 deg)");
        telemetry.addData("Virtual Goal (X, Y)", "%.1f, %.1f in", testGoalX, testGoalY);
        telemetry.addData("Calculated Target Angle", "%.1f deg", targetAngle);
        telemetry.addLine("----------------------------------------");
        telemetry.addLine("CONTROLS: D-Pad Up/Down   = Adjust Goal Y (+/-)");
        telemetry.addLine("          D-Pad Left/Right = Adjust Goal X (+/-)");
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.end();
    }
}