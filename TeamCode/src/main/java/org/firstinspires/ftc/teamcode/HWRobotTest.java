package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class HWRobotTest {

    /*
     * Setup
     */
    /*--------------------------------------------------------------------------------------------*/

    // Declare drive motors
    public DcMotor FLDrive = null;
    public DcMotor FRDrive = null;
    public DcMotor BLDrive = null;
    public DcMotor BRDrive = null;

    // Declare collector motors
    public DcMotor BackCollectL = null;
    public DcMotor BackCollectR = null;
    public DcMotor FrontCollectL = null;
    public DcMotor FrontCollectR = null;

    // Private hw members
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Declare variables
    double wheelDiameter = 4;
    double wheelCircumference = wheelDiameter * Math.PI;
    int ticksPerRotation = 560;

    // Constructor
    public HWRobotTest() {

    }


    /*
     * Initializations
     */
    /*--------------------------------------------------------------------------------------------*/

    /**
     * Initializes motors and servos
     * assigns to names in config, sets modes, and sets directions
     * @param ahwMap allows function to use hardwareMap, input hardwareMap into function when using
     */
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and Initialize Drive Motors
        FLDrive = hwMap.get(DcMotor.class, "FLMotor");
        FRDrive = hwMap.get(DcMotor.class, "FRMotor");
        BLDrive = hwMap.get(DcMotor.class, "BLMotor");
        BRDrive = hwMap.get(DcMotor.class, "BRMotor");

        // Define and Initialize collector Motors
        BackCollectL = hwMap.get(DcMotor.class, "BackCollectL");
        BackCollectR = hwMap.get(DcMotor.class, "BackCollectR");
        FrontCollectL = hwMap.get(DcMotor.class, "FrontCollectL");
        FrontCollectR = hwMap.get(DcMotor.class, "FrontCollectR");

        // Reverse Drive Motors
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reverse Collector Motors
        BackCollectL.setDirection(DcMotor.Direction.REVERSE);
        BackCollectR.setDirection(DcMotor.Direction.FORWARD);
        FrontCollectL.setDirection(DcMotor.Direction.REVERSE);
        FrontCollectR.setDirection(DcMotor.Direction.FORWARD);

        // Set powers to 0
        FLDrive.setPower(0);
        FRDrive.setPower(0);
        BLDrive.setPower(0);
        BRDrive.setPower(0);

        // set to run using encoders
        setDriveMode("withEncode");

        // set zero power behavior to brake
        setDriveMode("brake");

    }


    /**
     * Sets different drive modes for drive motors depending on what is imputed
     * @param mode "reset" - STOP_AND_RESET_ENCODER, "position" - RUN_TO_POSITION,
     *             "withEncode" - RUN_USING_ENCODER, "withoutEncode" - RUN_WITHOUT_ENCODER,
     *             "brake" - ZeroPowerBehavior.BRAKE
     */
    public void setDriveMode(String mode) {

        // check what was imputed
        switch(mode) {

            // stop and reset encoders
            case "reset":
                FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;

            // set motors to run to position
            case "position":
                FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;

            // set motors to run using encoders
            case "withEncode":
                FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;

            // set motors to run without encoders
            case "withoutEncode":
                FLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                FRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BLDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                BRDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;

            // set zero power behavior to brake
            case "brake":
                FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;

        }

    }



    /*
     * Utility
     */
    /*--------------------------------------------------------------------------------------------*/

    /**
     * Converts inches into number of encoder ticks
     * @param inches number of inches to be converted
     * @return number of encoder ticks calculated from inches
     */
    public int inchTick(double inches) {

        // return wheel ticks per revolution multiplied by number of revolutions (inches / circumference)
        return((int) ((inches / wheelCircumference) * ticksPerRotation));

    }


    /**
     * Converts degrees into number of encoder ticks
     * @param degrees number of degrees to be converted
     * @return number of encoder ticks calculated from inches
     */
    public int degreeTick(double degrees) {

        // return wheel ticks by multiplying degrees by 8
        return((int) (degrees * 8));

    }


    /**
     * Checks to see if any drive motors are busy
     * @return returns true if any are busy
     */
    public boolean driveMotorsBusy() {

        // return in any drive motors are busy
        return(FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy());

    }


    /**
     * Set all motors of a certain group to a certain power
     * @param motorType the type of motors to set power for
     *                  "drive" - drive motors
     *                  "collector" - collector motors
     * @param power power for motors to run at
     */
    public void setEqualPower(String motorType, double power) {

        // check what was imputed
        switch(motorType) {

            // set power of drive motors to power
            case "drive":
                FLDrive.setPower(power);
                FRDrive.setPower(power);
                BLDrive.setPower(power);
                BRDrive.setPower(power);
                break;

            // set power of collect motors to power
            case "collect":
                BackCollectL.setPower(power);
                BackCollectR.setPower(power);
                FrontCollectL.setPower(power / 2);
                FrontCollectR.setPower(power / 2);
                break;

        }

    }


    /**
     * Returns quotient as an int without the remainder
     * @param dividend number to be divided
     * @param divisor number dividing by
     * @return quotient of dividend / divisor without the remainder
     */
    public int returnIntQuotient(double dividend, double divisor) {
        return (int) ((dividend / divisor) - ((dividend % divisor) / divisor));
    }



    /*
     * Autonomous
     */
    /*--------------------------------------------------------------------------------------------*/

    /**
     * Drive using encoders, can do forward/back, side-to-side and rotation but only one at a time
     * @param mode "forward" - drives forward or backwards, input negative inchesOrDegrees for backwards
     *             "sideways" - drives left or right, input negative inchesOrDegrees for left
     *             "rotation" - rotates left or right, input negative inchesOrDegrees for left
     * @param inchesOrDegrees inches or degrees to be traveled, make this negative not maxSpeed
     * @param maxSpeed maximum speed robot can go to, this will not always be reached
     * @param opModeActive input opModeIsActive() when using to check if the op mode is active
     */
    public void encoderDrive(String mode, double inchesOrDegrees, double maxSpeed, boolean opModeActive) {

        // stop and reset encoders using setDriveMode()
        setDriveMode("reset");

        // set up int ticks to hold number of ticks
        int ticks;

        // switch how the tick counts are assigned based on what mode is imputed
        switch(mode) {

            // if "forward" is imputed,
            // convert inchesOrDegrees with inchTick() and assign ticks in the forward pattern
            case "forward":
                ticks = inchTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(ticks);
                break;

            // if "sideways" is imputed,
            // convert inchesOrDegrees with inchTick() and assign ticks in the sideways pattern
            case "sideways":
                ticks = inchTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(-ticks);
                BLDrive.setTargetPosition(-ticks);
                BRDrive.setTargetPosition(ticks);
                break;

            //if "rotation" is imputed,
            // convert inchesOrDegrees with degreeTick() and assign ticks in the rotation pattern
            case "rotation":
                ticks = degreeTick(inchesOrDegrees);
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(-ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(-ticks);
                break;

            // if incorrect mode is imputed,
            // default to no movement
            default:
                ticks = 0;
                FLDrive.setTargetPosition(ticks);
                FRDrive.setTargetPosition(ticks);
                BLDrive.setTargetPosition(ticks);
                BRDrive.setTargetPosition(ticks);

        }

        // set the distance to accelerate to be always positive and half of the total distance
        double accelerationDistance = Math.abs(ticks / 2);

        // set the minimum speed to .1
        double minSpeed = .1;

        // set the amout power will increase by every 45 ticks
        double powerIncrease = .05;

        // set the initial value of power to the value of minSpeed
        double power = minSpeed;

        // set the drive motors to run to position
        setDriveMode("position");

        // run while the opMode is active and the motors are busy
        while (opModeActive && driveMotorsBusy()) {

            // if the absolute value of the current position is less than the acceleration distance
            // (if the robot has not reached the halfway point yet)
            if (Math.abs(FLDrive.getCurrentPosition()) < accelerationDistance) {

                // increase power by .05 for every 45 ticks the robot travels (~1 inch)
                power += powerIncrease * returnIntQuotient(FLDrive.getCurrentPosition(), 45);

            // else if the motor is past the halfway point
            } else if (Math.abs(FLDrive.getCurrentPosition()) > accelerationDistance) {

                // decrease power by .05 for every 45 ticks the robot travels (~1 inch)
                power -= powerIncrease * returnIntQuotient(FLDrive.getCurrentPosition(), 45);

            }

            // make sure that power is between minSpeed and maxSpeed
            power = Range.clip(power, minSpeed, maxSpeed);

            // set the power of the motors to power
            setEqualPower("drive", power);

        }

        // turn off run to position by setting the drive motors to run with encoders
        setDriveMode("withEncode");

        // set power of drive motors to zero
        setEqualPower("drive", 0);

    }



    /*
     * TeleOp
     */
    /*--------------------------------------------------------------------------------------------*/

    /**
     * Mecanum driving for driver control
     * @param G1RX gamepad right stick x, controls side-to-side movement
     * @param G1RY gamepad right stick y, controls forward/backward movement
     * @param G1LX gamepad left stick x, controls left/right rotation
     * @param G1LT gamepad left trigger, if pressed the speed will be reduced by powerScale
     * @param powerScale % to multiply powers by to reduce speed of the robot
     */
    public void mecanumDrive(double G1RX, double G1RY, double G1LX, double G1LT, double powerScale) {

        // set up powerFactor variable
        double powerFactor;

        // if trigger is pressed, set powerFactor to powerScale
        // else set powerFactor to 1
        if (G1LT > 0) {
            powerFactor = powerScale;
        } else {
            powerFactor = 1;
        }

        // set powers
        FLDrive.setPower(Range.clip(powerFactor * (G1RX + G1RY + G1LX), -1, 1));
        FRDrive.setPower(Range.clip(powerFactor * (G1RX - G1RY - G1LX), -1, 1));
        BLDrive.setPower(Range.clip(powerFactor * (G1RX - G1RY + G1LX), -1, 1));
        BRDrive.setPower(Range.clip(powerFactor * (G1RX + G1RY - G1LX), -1, 1));

    }


}