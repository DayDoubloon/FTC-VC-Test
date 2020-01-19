package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AccelerationTest extends LinearOpMode {

    public DcMotor motor = null;

    @Override
    public void runOpMode() {

        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        while (opModeIsActive()) {

        }

    }

    public void accelDecel(int targetPosition, double minSpeed, double maxSpeed) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(targetPosition);

        double accelerationDistance = motor.getTargetPosition() / 4;
        double decelerationDistance = motor.getTargetPosition() - accelerationDistance;

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motor.getCurrentPosition() < motor.getTargetPosition()) {

            if (motor.getCurrentPosition() < accelerationDistance) {

                double speedFactor = motor.getCurrentPosition() / accelerationDistance;
                motor.setPower(speedFactor * maxSpeed);

            } else if (decelerationDistance > motor.getCurrentPosition() && motor.getCurrentPosition() > accelerationDistance) {

                motor.setPower(maxSpeed);

            } else if (motor.getCurrentPosition() > decelerationDistance) {

                double speedFactor = (motor.getCurrentPosition() - decelerationDistance) / accelerationDistance;
                motor.setPower(speedFactor * maxSpeed);

            }

        }

        motor.setPower(0);

    }

}
