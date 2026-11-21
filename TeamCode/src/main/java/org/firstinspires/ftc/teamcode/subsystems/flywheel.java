package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.InterpolatingTreeMap;

public class flywheel {
    private final DcMotorEx motor;

    private final PIDFController pidfController;
    private final InterpolatingTreeMap distanceToVelocityTable;

    private static final double TICKS_PER_REV = 537.6; // GoBILDA 19.2:1 (Update if using a different motor)

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double powerOutput = 0.0;

    public flywheel(DcMotorEx motor, double kp, double ki, double kd, double kf) {
        this.motor = motor;
        this.motor.setDirection(DcMotor.Direction.FORWARD); // Adjust if your motor spins backward

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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        // 2. Read native motor velocity (ticks/sec) and convert to RPM
        double ticksPerSec = motor.getVelocity();
        currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV;

        // 3. Calculate PIDF output
        powerOutput = pidfController.calculate(currentRPM, targetRPM);

        // 4. Apply power to the motor
        motor.setPower(powerOutput);
    }

    /**
     * Call this in the OpMode's stop() or end() method.
     */
    public void end() {
        motor.setPower(0);
    }

    // Getters for telemetry debugging
    public double getTargetRPM() { return targetRPM; }
    public double getCurrentRPM() { return currentRPM; }
    public double getPowerOutput() { return powerOutput; }
}