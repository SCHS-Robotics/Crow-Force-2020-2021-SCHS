package org.firstinspires.ftc.teamcode.Subsystems.OneTimeUse;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;

import org.firstinspires.ftc.teamcode.Robots.TestMec;

public class TestMecOp extends SubSystem {
    public TestMecOp(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {
        if(robot.gamepad1.y == true) {
            ((TestMec) robot).mDrive.reverseFront();
        }
        if(robot.gamepad1.a == true) {
            ((TestMec) robot).mDrive.reverseBack();
        }
        if(robot.gamepad1.x == true) {
            ((TestMec) robot).mDrive.reverseLeft();
        }
        if(robot.gamepad1.b == true) {
            ((TestMec) robot).mDrive.reverseRight();
        }

    }

    @Override
    public void stop() {

    }
}
