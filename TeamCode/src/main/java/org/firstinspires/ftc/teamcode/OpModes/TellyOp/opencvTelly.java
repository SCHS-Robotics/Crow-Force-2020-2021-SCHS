package org.firstinspires.ftc.teamcode.OpModes.TellyOp;

import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.Robots.opencvRobot;

@TeleOp (name = "OpencvCamera")
public class opencvTelly extends BaseTeleop {
    @Override
    protected Robot buildRobot() {
        return null;
    }
    /*opencvRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new opencvRobot(this);
        return robot;
    }
    public void update()throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("status: ", robot.camera.check());
            telemetry.update();
        }
    }*/
}
