package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class gate {
    private final Servo servo;

    // Default servo positions (values range from 0.0 to 1.0)
    // Adjust these numbers based on how your physical gate linkage is mounted.
    private double closedPosition = 0.0;
    private double openPosition = 1.0;

    public gate(Servo servo) {
        this.servo = servo;
    }

    /**
     * Call this in the OpMode's init() method.
     */
    public void init() {
        // Automatically start with the gate closed on initialization
        servo.setPosition(closedPosition);
    }

    /**
     * Call this repeatedly in the OpMode's loop() method.
     *
     * @param isOpen True to command the gate open, false to close it.
     */
    public void loop(boolean isOpen) {
        if (isOpen) {
            servo.setPosition(openPosition);
        } else {
            servo.setPosition(closedPosition);
        }
    }

    /**
     * Manually open the gate.
     */
    public void open() {
        servo.setPosition(openPosition);
    }

    /**
     * Manually close the gate.
     */
    public void close() {
        servo.setPosition(closedPosition);
    }

    /**
     * Call this in the OpMode's stop() or end() method.
     */
    public void end() {
        // Optional: Return gate to closed position when the OpMode ends
        servo.setPosition(closedPosition);
    }

    /**
     * Optional: Calibrate custom servo positions.
     */
    public void setPositions(double closed, double open) {
        this.closedPosition = closed;
        this.openPosition = open;
    }
}