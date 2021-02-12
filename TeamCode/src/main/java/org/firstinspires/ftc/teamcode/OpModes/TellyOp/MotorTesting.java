package org.firstinspires.ftc.teamcode.OpModes.TellyOp;



import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;

import org.firstinspires.ftc.teamcode.Robots.MotorTest;

public class MotorTesting extends BaseTeleop {
    @Override
    protected Robot buildRobot() {
        return new MotorTest(this);
    }
}
