package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class drivetrain {
    private final DcMotorEx leftFront, rightFront, leftBack, rightBack;
    private final GoBildaPinpointDriver pinpoint;

    public drivetrain(HardwareMap hardwareMap) {
        // 1. Initialize Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set motor directions (invert left or right depending on your chassis build)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // 2. Initialize Pinpoint Odometry Computer (configured as "pinpoint" in Driver Station)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    /**
     * Call in OpMode init()
     */
    public void init() {
        // Brake mode ensures precise positioning when joysticks are released
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure Pinpoint pods and mounting offsets (values in mm)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(0.0, -4.0, DistanceUnit.CM); // Adjust X and Y offsets relative to robot center

        // Recalibrate IMU and reset position to origin (0, 0)
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
    }

    /**
     * Call repeatedly in OpMode loop()
     *
     * @param drive        Forward/Backward joystick input
     * @param strafe       Left/Right sideways joystick input
     * @param turn         Rotation joystick input
     * @param fieldCentric True for field-centric driving using Pinpoint heading
     */
    public void loop(double drive, double strafe, double turn, boolean fieldCentric) {
        // CRITICAL: Must call update() every loop to refresh sensor data
        pinpoint.update();

        double robotHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double rotX = strafe;
        double rotY = drive;

        if (fieldCentric) {
            // Rotate movement vector by robot heading for field-centric control
            rotX = strafe * Math.cos(-robotHeading) - drive * Math.sin(-robotHeading);
            rotY = strafe * Math.sin(-robotHeading) + drive * Math.cos(-robotHeading);
        }

        // Mecanum kinematics mixing
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
        double frontLeftPower  = (rotY + rotX + turn) / denominator;
        double backLeftPower   = (rotY - rotX + turn) / denominator;
        double frontRightPower = (rotY - rotX - turn) / denominator;
        double backRightPower  = (rotY + rotX - turn) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public void end() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    // --- Position and Velocity Getters for Turret Shoot-on-the-Move ---

    public double getX(DistanceUnit unit) {
        return pinpoint.getPosition().getX(unit);
    }

    public double getY(DistanceUnit unit) {
        return pinpoint.getPosition().getY(unit);
    }

    public double getHeading(AngleUnit unit) {
        return pinpoint.getHeading(unit);
    }

    public double getVelocityX(DistanceUnit unit) {
        return pinpoint.getVelX(unit); // Inches per second
    }

    public double getVelocityY(DistanceUnit unit) {
        return pinpoint.getVelY(unit); // Inches per second
    }
}