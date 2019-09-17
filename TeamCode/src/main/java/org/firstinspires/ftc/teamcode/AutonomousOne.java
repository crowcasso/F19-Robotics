package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousOne", group="Autonomous")
public class AutonomousOne extends LinearOpMode {

    private RobotHardware robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);

        waitForStart();

        drive(182.88, 0.2);
        drive(-60.96, 0.2);
        turn(-90, 0.2);
        drive(182.88, 0.2);
        turn(-120, 0.2);
        drive(140.716, 0.2);
        turn(-60, 0.2);
        drive(121.92, 0.2);
        turn(270, 0.2);

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
