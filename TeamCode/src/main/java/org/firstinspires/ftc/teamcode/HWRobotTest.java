package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HWRobotTest {

    //Declare Public Hardware Devices
    public static DcMotor FLDrive = null;
    public static DcMotor FRDrive = null;
    public static DcMotor BLDrive = null;
    public static DcMotor BRDrive = null;

    //Private hw members
    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    //Constructor
    public HWRobotTest() {

    }

    //Initialize motors and servos
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
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void mecanumDrive(double G1RX, double G1RY, double G1LX, double G1LT, double powerScale) {

        double powerFactor;

        if (G1LT > 0) {
            powerFactor = powerScale;
        } else {
            powerFactor = 1;
        }

        //set powers
        FLDrive.setPower(powerFactor * (G1RX + G1RY + G1LX));
        FRDrive.setPower(powerFactor * (G1RX - G1RY - G1LX));
        BLDrive.setPower(powerFactor * (G1RX - G1RY + G1LX));
        BRDrive.setPower(powerFactor * (G1RX + G1RY - G1LX));

    }




}
