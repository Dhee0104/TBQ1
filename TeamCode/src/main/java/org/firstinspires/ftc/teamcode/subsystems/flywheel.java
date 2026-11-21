package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;

public class flywheel {
    private final DcMotorEx motorLeft;
    private final DcMotorEx motorRight;

    private final PIDFController pidfController;
    private final InterpolatingTreeMap distanceToVelocityTable;

    // TODO: Update this constant to match your specific motor's encoder CPR (Counts Per Revolution)
    // Example: GoBILDA 19.2:1 = 537.6 | REV HD Hex = 537.7 | REV Core Hex = 288.0
    private static final double TICKS_PER_REV = 537.6;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double powerOutput = 0.0;

    public flywheel(DcMotorEx motorLeft, DcMotorEx motorRight, double kp, double ki, double kd, double kf) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;

        // Motors are on opposite sides, reverse one so they spin together
        this.motorLeft.setDirection(DcMotor.Direction.FORWARD);
        this.motorRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize FTCLib PIDF Controller
        this.pidfController = new PIDFController(kp, ki, kd, kf);

        // Initialize our standalone Interpolating TreeMap (Distance -> Target RPM)
        this.distanceToVelocityTable = new InterpolatingTreeMap();
        setupLookupTable();
    }

    /**
     * Populate calibration points: Key = Distance (inches), Value = Target RPM
     */
    private void setupLookupTable() {
        distanceToVelocityTable.put(24.0, 1500.0); // At 24 inches -> target 1500 RPM
        distanceToVelocityTable.put(48.0, 2500.0); // At 48 inches -> target 2500 RPM
        distanceToVelocityTable.put(72.0, 3500.0); // At 72 inches -> target 3500 RPM
    }

    /**
     * Call this in the OpMode's init() method.
     */
    public void init() {
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidfController.reset();
    }

    /**
     * Call this repeatedly in the OpMode's loop() method.
     *
     * @param distance The current distance to the target
     */
    public void loop(double distance) {
        // 1. Look up target RPM from our lookup table based on distance
        targetRPM = distanceToVelocityTable.getInterpolated(distance);

        // 2. Read native motor velocity (ticks/sec), average both motors, and convert to RPM
        double avgTicksPerSec = (motorLeft.getVelocity() + motorRight.getVelocity()) / 2.0;
        currentRPM = (avgTicksPerSec * 60.0) / TICKS_PER_REV;

        // 3. Calculate PIDF output (FTCLib calculate takes (measured, target))
        powerOutput = pidfController.calculate(currentRPM, targetRPM);

        // 4. Apply power to motors
        motorLeft.setPower(powerOutput);
        motorRight.setPower(powerOutput);
    }

    /**
     * Call this in the OpMode's stop() or end() method.
     */
    public void end() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    // Getters for telemetry debugging
    public double getTargetRPM() { return targetRPM; }
    public double getCurrentRPM() { return currentRPM; }
    public double getPowerOutput() { return powerOutput; }
}