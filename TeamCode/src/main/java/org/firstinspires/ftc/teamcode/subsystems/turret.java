package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class turret {
    private final Servo servo1;
    private final Servo servo2;

    private double currentTargetAngleDeg = 180.0;

    // 355-degree servo configuration parameters
    private static final double MAX_SERVO_RANGE_DEG = 355.0;
    private static final double HALF_RANGE_DEG = MAX_SERVO_RANGE_DEG / 2.0; // 177.5 degrees
    private static final double CENTER_ANGLE_DEG = 180.0; // 180° corresponds to servo 0.5

    public turret(Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
    }

    public void init() {
        setAngle(CENTER_ANGLE_DEG); // Initializes to servo position 0.5 (facing back)
    }

    /**
     * Calculates the raw angle to the goal relative to the robot's heading.
     */
    public double calculateTargetAngle(double dX, double dY, double robotHeading) {
        double angleToGoalField = Math.atan2(dY, dX);
        double relativeAngleRad = angleToGoalField - robotHeading;
        return Math.toDegrees(relativeAngleRad);
    }

    /**
     * Sets the turret angle using 180° as the physical center.
     * @param angleDeg Desired angle in degrees where 180° maps to servo 0.5
     */
    public void setAngle(double angleDeg) {
        this.currentTargetAngleDeg = angleDeg;

        // 1. Calculate offset from the 180° center position
        double offsetDeg = angleDeg - CENTER_ANGLE_DEG;

        // 2. Clamp the offset to prevent hitting mechanical hard stops (±177.5°)
        offsetDeg = Math.max(-HALF_RANGE_DEG, Math.min(HALF_RANGE_DEG, offsetDeg));

        // 3. Map the clamped offset to a servo position where 0.5 is center (0 offset)
        // -177.5 deg offset -> 0.0 position
        //      0.0 deg offset -> 0.5 position
        // +177.5 deg offset -> 1.0 position
        double servoPosition = 0.5 - (offsetDeg / MAX_SERVO_RANGE_DEG);

        // Safety clamp for bounds [0.0, 1.0]
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

        servo1.setPosition(servoPosition);
        servo2.setPosition( servoPosition); // Mirror servo if mounted symmetrically
    }

    public void end() {
        setAngle(CENTER_ANGLE_DEG);
    }

    public double getCurrentTargetAngle() {
        return currentTargetAngleDeg;
    }
}