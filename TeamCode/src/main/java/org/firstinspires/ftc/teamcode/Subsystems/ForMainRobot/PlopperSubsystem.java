package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;



import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.qualcomm.robotcore.hardware.Servo;

public class PlopperSubsystem extends SubSystem {

    CustomizableGamepad inputs;
    public Servo armServo;
    public Servo clawServo;
    static final String ARMBUTTON = "ArmButton";
    static final String CLAWBUTTON = "ClawButton";
    Toggle armToggle;
    Toggle clawToggle;
    public PlopperSubsystem(Robot r, String armServo, String clawServo) {
        super(r);
        this.armServo = robot.hardwareMap.servo.get(armServo);
        this.clawServo = robot.hardwareMap.servo.get(clawServo);
        armToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
        clawToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
        usesConfig = true;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        inputs = robot.pullControls(this);
    }

    @Override
    public void handle() {
        armToggle.updateToggle(inputs.getInput("armToggleButton"));
        clawToggle.updateToggle(inputs.getInput("armToggleButton"));
        if(armToggle.getCurrentState()){
            armServo.setPosition(1);
        }
        else {
            armServo.setPosition(-1);
        }
        if(clawToggle.getCurrentState()){
            clawServo.setPosition(1);
        }
        else {
            clawServo.setPosition(-1);
        }
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(ARMBUTTON, Button.BooleanInputs.b, 2),
                new ConfigParam(CLAWBUTTON, Button.BooleanInputs.x, 2)
        };
    }
}
