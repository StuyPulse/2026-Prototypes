package com.stuypulse.robot.subsystems;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static final Intake instance;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum IntakeState {
        INTAKE(() -> Settings.Intake.INTAKE.get()), //TODO: add to settings
        OUTTAKE(() -> Settings.Intake.OUTTAKE.get()),
        STOW(() -> Settings.Intake.STOW.get());

        private Supplier<Double> dutyCycle;

        private IntakeState(Supplier<Double> dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public Supplier<Double> getDutyCycle() {
            return dutyCycle;
        }
    }

    public enum IntakePosition {
        UP(() -> Settings.Intake.UP.get()),
        DOWN(() -> Settings.Intake.DOWN.get()),
        STOP(() -> Settings.Intake.STOP.get());

        private Supplier<Double> speed; //TODO: update variable name when using PID controller and etc

        private IntakePosition(Supplier<Double> speed) {
            this.speed = speed;
        }

        public Supplier<Double> getSpeed() {
            return speed;
        }
    }

    IntakePosition position;
    IntakeState state;

    protected Intake() {
        position = IntakePosition.STOP;
        state = IntakeState.STOW;
    }

    public IntakePosition getIntakePosition() {
        return this.position;
    }

    public IntakeState getIntakeState() {
        return this.state;
    }

    public void setIntakePosition(IntakePosition position) {
        this.position = position;
    }

    public void setIntakeState(IntakeState state) {
        this.state = state;
    }

}
