package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Counting Lines", group="Autonomous")
public class AutonomousTwo extends LinearOpMode {

    private RobotHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        // write code here
        while (opModeIsActive()) {

            boolean touch = robot.touchSensor.getState();
            telemetry.addData("touchSensor", touch);

            int red = robot.colorSensor.red();
            int green = robot.colorSensor.green();
            int blue = robot.colorSensor.blue();
            int alpha = robot.colorSensor.alpha();
            telemetry.addData("colorSensor", "r:" + red + ", g:" + green +
                    ", b:" + blue + ", a:" + alpha);


            telemetry.update();

        }

    }


    private void drive(double distance, double speed) {

        int distanceInTicks = robot.convertCmToTicks(Math.abs(distance));

        speed = Math.abs(speed);
        double direction = Math.signum(distance);

        robot.resetEncoders();

        robot.leftMotor.setPower(speed * direction);
        robot.rightMotor.setPower(speed * direction);

        while (Math.abs(robot.leftMotor.getCurrentPosition()) < distanceInTicks && opModeIsActive()) {
            // do nothing
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }

    private void turn(double degree, double speed) {

        int distanceInTicks = robot.convertDegreesToTicks(Math.abs(degree));

        speed = Math.abs(speed);
        double direction = Math.signum(degree);

        robot.resetEncoders();

        robot.leftMotor.setPower(speed * direction);
        robot.rightMotor.setPower(speed * -direction);

        while (Math.abs(robot.leftMotor.getCurrentPosition()) < distanceInTicks && opModeIsActive()) {
            // do nothing
        }

        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }
}
