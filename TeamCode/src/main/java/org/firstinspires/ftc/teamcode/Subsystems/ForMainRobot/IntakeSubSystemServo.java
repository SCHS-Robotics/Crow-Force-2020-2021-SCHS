package org.firstinspires.ftc.teamcode.Subsystems.ForMainRobot;


import com.SCHSRobotics.HAL9001.system.config.ConfigParam;
import com.SCHSRobotics.HAL9001.system.config.TeleopConfig;
import com.SCHSRobotics.HAL9001.system.robot.Robot;
import com.SCHSRobotics.HAL9001.system.robot.SubSystem;
import com.SCHSRobotics.HAL9001.util.control.Button;
import com.SCHSRobotics.HAL9001.util.control.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.control.Toggle;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubSystemServo extends SubSystem {
    private CustomizableGamepad inputs;
    Servo IntakeServo;
    private final double DOWN = -120;
    private final double UP = 180;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);


    static final String INTAKEBUTTON = "IntakeButton";

    public IntakeSubSystemServo(Robot r, String servo) {
        super(r);
        IntakeServo = robot.hardwareMap.servo.get(servo);
        usesConfig = true;
    }



    @Override
    public void init()  {
        IntakeServo.setPosition(UP);
    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        inputs = robot.pullControls(this);
    }
    @Override
    public void handle ()  {
        toggle.updateToggle(inputs.getInput(INTAKEBUTTON));
        if (toggle.getCurrentState()) {
            IntakeServo.setPosition(DOWN);
        }
        else {
            IntakeServo.setPosition(UP);
        }

    }

    @Override
    public void stop ()  {

    }

    public void intakeDown () {
        IntakeServo.setPosition(DOWN);
    }


    public void intakeUp() {
            IntakeServo.setPosition(UP);
        }



    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(INTAKEBUTTON, Button.BooleanInputs.x,2)
        };
    }
}

