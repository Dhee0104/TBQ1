package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class intake {
    private final DcMotorEx motor;

    private double intakePower = 0.8;
    private double outtakePower = -0.8;

    public intake(DcMotorEx motor) {
        this.motor = motor;
        this.motor.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Call this in the OpMode's init() method.
     */
    public void init() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);
    }

    /**
     * Call this repeatedly in the OpMode's loop() method.
     *
     * @param isIntaking  True when the button to pull items in is pressed
     * @param isReversing True when the button to push items out is pressed
     */
    public void loop(boolean isIntaking, boolean isReversing) {
        if (isIntaking) {
            motor.setPower(intakePower);
        } else if (isReversing) {
            motor.setPower(outtakePower);
        } else {
            motor.setPower(0.0);
        }
    }

    /**
     * Call this in the OpMode's stop() or end() method.
     */
    public void end() {
        motor.setPower(0.0);
    }

    /**
     * Optional: Adjust default speeds dynamically.
     */
    public void setSpeeds(double intake, double outtake) {
        this.intakePower = intake;
        this.outtakePower = outtake;
    }
}
