package org.firstinspires.ftc.teamcode.OpModes.TellyOp;


import com.SCHSRobotics.HAL9001.system.config.StandAlone;
import com.SCHSRobotics.HAL9001.system.robot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MainRobot;

@StandAlone
@TeleOp(name = "Main Teleop")
public class MainTeleop extends BaseTeleop {
    MainRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new MainRobot(this);
        return robot;
    }



    @Override
    protected void onInit() {
        /*robot.mDrive.setStabilityPID(new PIDController(0,0,0));*/
    }
}
