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

    public void accelDecel(int targetPosition, double maxSpeed) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(targetPosition);

        double accelerationDistance = motor.getTargetPosition() / 4;
        double decelerationDistance = motor.getTargetPosition() - accelerationDistance;
        double power;

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motor.getCurrentPosition() < motor.getTargetPosition()) {

            if (motor.getCurrentPosition() < accelerationDistance) {
                double speedFactor = motor.getCurrentPosition() / accelerationDistance;
                power = speedFactor * maxSpeed;

                if (power < .1) {
                    power = .1;
                }

                motor.setPower(power);

            } else if (decelerationDistance > motor.getCurrentPosition() && motor.getCurrentPosition() > accelerationDistance) {
                power = maxSpeed;

                if (power < .1) {
                    power = .1;
                }

                motor.setPower(power);

            } else if (motor.getCurrentPosition() > decelerationDistance) {
                double speedFactor = (motor.getCurrentPosition() - decelerationDistance) / accelerationDistance;
                power = speedFactor * maxSpeed;

                if (power < .1) {
                    power = .1;
                }

                motor.setPower(power);

            }

        }

        motor.setPower(0);

    }

    public void continuousAccelDecel() {

    }

}
