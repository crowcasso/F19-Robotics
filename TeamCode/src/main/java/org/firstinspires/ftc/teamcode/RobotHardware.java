package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    //DcMotors
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;

    //Sensors
    public DigitalChannel touchSensor = null;
    public ColorSensor colorSensor = null;

    private final double WHEEL_DIAMETER = 10.0;  // in cm
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    private final double ENCODER_TICKS_40 = 1120;
    private final double WHEEL_TRACKS = 36.5; // in cm


    public RobotHardware(HardwareMap hardwareMap) {

        // map the motors to the hardware
        leftMotor = hardwareMap.get(DcMotor.class, "left_drive");
        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");

        // reverse one of the motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        resetEncoders();

        // setup the sensors
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public int convertCmToTicks(double cm) {
        double wheelRotations = cm / WHEEL_CIRCUMFERENCE;
        return (int) (wheelRotations * ENCODER_TICKS_40);
    }

    public int convertDegreesToTicks(double degrees) {
        double turnCircumference = Math.PI * WHEEL_TRACKS;
        double wheelDistance = degrees / 360.0 * turnCircumference;
        double rotations = wheelDistance / WHEEL_CIRCUMFERENCE;

        return (int) (rotations * ENCODER_TICKS_40);
    }

}
