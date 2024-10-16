package frc.robot.subsystems.linebreak;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.utils.statemachine.StateMachine;

public class LineBreak extends StateMachine<LineBreak.State> {
    AnalogInput lowerIntakeSensor = new AnalogInput(3);
    AnalogInput magazine3Sensor = new AnalogInput(2);
    AnalogInput magazine2Sensor = new AnalogInput(1);
    AnalogInput magazine1Sensor = new AnalogInput(0);

    public LineBreak() {
        super("LineBreak", State.UNDETERMINED, State.class);
    }

    @Override
    protected void determineSelf() {
        setState(State.UNDETERMINED);
    }

    @Override
    protected void update() {
        boolean lowerIntake = lowerIntakeSensor.getVoltage() > 1.0;
        boolean magazine3 = magazine3Sensor.getVoltage() > 1.0;
        boolean magazine2 = magazine2Sensor.getVoltage() > 1.0;
        boolean magazine1 = magazine1Sensor.getVoltage() > 1.0;

        // Overall state
        if(lowerIntake && magazine3 && magazine2 && magazine1) {
            setState(State.LOADED);
        }
        else if(!lowerIntake && !magazine3 && !magazine2 && !magazine1) {
            setState(State.EMPTY);
        } else {
            setState(State.HAS_GAME_PIECE);
        }

        // Flags for individual sensors
        if(lowerIntake) {
            setFlag(State.GAME_PIECE_LOWER_INTAKE);
        } else {
            clearFlag(State.GAME_PIECE_LOWER_INTAKE);
        }

        if(magazine3) {
            setFlag(State.GAME_PIECE_MAGAZINE_3);
        } else {
            clearFlag(State.GAME_PIECE_MAGAZINE_3);
        }

        if(magazine2) {
            setFlag(State.GAME_PIECE_MAGAZINE_2);
        } else {
            clearFlag(State.GAME_PIECE_MAGAZINE_2);
        }

        if(magazine1) {
            setFlag(State.GAME_PIECE_MAGAZINE_1);
        } else {
            clearFlag(State.GAME_PIECE_MAGAZINE_1);
        }
    }

    public enum State {
        UNDETERMINED,
        HAS_GAME_PIECE,
        EMPTY,
        LOADED,

        // flags
        GAME_PIECE_LOWER_INTAKE,
        GAME_PIECE_MAGAZINE_3,
        GAME_PIECE_MAGAZINE_2,
        GAME_PIECE_MAGAZINE_1,
    }
}
