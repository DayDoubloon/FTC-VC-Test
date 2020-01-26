package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class HWRobotTest {

    //Declare Public Hardware Devices
    public DcMotor FLDrive = null;
    public DcMotor FRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BRDrive = null;

    //Private hw members
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Declare variables
    double wheelDiameter = 4;
    double wheelCircumference = wheelDiameter * Math.PI;
    int ticksPerRotation = 560;

    //Constructor
    public HWRobotTest() {

    }

    /* Initializations */
    /*--------------------------------------------------------------------------------------------*/

    //Initialize motors and servos, set modes
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Define and Initialize Motors
        FLDrive = hwMap.get(DcMotor.class, "FLMotor");
        FRDrive = hwMap.get(DcMotor.class, "FRMotor");
        BLDrive = hwMap.get(DcMotor.class, "BLMotor");
        BRDrive = hwMap.get(DcMotor.class, "BRMotor");

        //Reverse Motors
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        //Set powers to 0
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);

        //set to run using encoders
        setDriveMode("withEncode");

        //set zero power behavior to brake
        setDriveMode("brake");

    }


    public void setDriveMode(String mode) {

        //check what was imputed
        switch(mode) {

            case "reset":
                FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

            case "position":
                FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            case "withEncode":
                FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            case "withoutEncode":
                FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;

            case "brake":
                FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;

        }

    }



    /* Utility */
    /*--------------------------------------------------------------------------------------------*/

    //convert inches to ticks for linear movement
    public int inchTick(double inches) {
        return((int) ((inches / wheelCircumference) * ticksPerRotation));
    }


    //convert degrees to ticks for rotation
    public int degreeTick(double degrees) {
        return((int) (degrees * 8));
    }


    //check if drive motors are busy
    public boolean driveMotorsBusy() {
        return(FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy());
    }



    /* Autonomous */
    /*--------------------------------------------------------------------------------------------*/

    public void encoderDrive(String mode, double inchesOrDegrees, double maxSpeed, boolean opModeActive) {

        setDriveMode("reset");
        int ticks;

        switch(mode) {
            case "forward":
                ticks = inchTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(ticks);
                break;

            case "sideways":
                ticks = inchTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(-ticks);
                BLDrive.setTargetPosition(-ticks);
                BRDrive.setTargetPosition(ticks);
                break;

            case "rotation":
                ticks = degreeTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(-ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(-ticks);
                break;

            default:
                ticks = 0;
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(ticks);

        }

        double accelerationDistance = Math.abs(ticks / 2);
        double minSpeed = .1;
        double power = minSpeed;

        setDriveMode("position");

        while (opModeActive && driveMotorsBusy()) {

            if (Math.abs(FLDrive.getCurrentPosition()) < accelerationDistance) {
                power += .01;
                power = Range.clip(power, minSpeed, maxSpeed);

                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);

            } else if (Math.abs(FLDrive.getCurrentPosition()) > accelerationDistance) {
                power -= .01;
                power = Range.clip(power, minSpeed, maxSpeed);

                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);

            }

        }

        setDriveMode("withEncode");

        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);

    }


    /* TeleOp */
    /*--------------------------------------------------------------------------------------------*/
    public void mecanumDrive(double G1RX, double G1RY, double G1LX, double G1LT, double powerScale) {

        double powerFactor;

        if (G1LT > 0) {
            powerFactor = powerScale;
        } else {
            powerFactor = 1;
        }

        //set powers
        FLDrive.setPower(Range.clip(powerFactor * (G1RX + G1RY + G1LX), -1, 1));
        FRDrive.setPower(Range.clip(powerFactor * (G1RX - G1RY - G1LX), -1, 1));
        BLDrive.setPower(Range.clip(powerFactor * (G1RX - G1RY + G1LX), -1, 1));
        BRDrive.setPower(Range.clip(powerFactor * (G1RX + G1RY - G1LX), -1, 1));

    }


}

