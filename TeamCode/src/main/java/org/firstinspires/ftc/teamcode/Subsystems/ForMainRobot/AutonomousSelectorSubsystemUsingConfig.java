package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;


import com.SCHSRobotics.HAL9001.system.config.AutonomousConfig;
import com.SCHSRobotics.HAL9001.system.config.ConfigData;
import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;

public class AutonomousSelectorSubsystemUsingConfig extends SubSystem {

    public String autonomous;
    public String color;
    public String startPos;
    public AutonomousSelectorSubsystemUsingConfig(Robot r) {
        super(r);
        usesConfig = true;
    }

    @Override
    public void init()  {

    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        ConfigData params = robot.pullNonGamepad(this);
        autonomous = params.getData("Autonomous", String.class);
        color = params.getData("Color", String.class);
        startPos = params.getData("StartPosition", String.class);
    }

    @Override
    public void handle()  {

    }

    @Override
    public void stop()  {

    }
    @AutonomousConfig
    public static ConfigParam[] autoConfig() {
        return new ConfigParam[] {
                new ConfigParam("Autonomous", new String[] {
                        "ParkOnBridge",
                        "Forward23in",
                        "MoveFoundationPark",
                        "MaxPoints",
                        "PID",
                        "OpenCV",
                        "Turn90"},
                        "ParkOnBridge"),
                new ConfigParam("Color", new String[] {
                        "Blue",
                        "Red"},
                        "Blue"),
                new ConfigParam("StartPosition", new String[] {
                        "Resource",
                        "Construction"},
                        "Resource")
        };
    }
}
