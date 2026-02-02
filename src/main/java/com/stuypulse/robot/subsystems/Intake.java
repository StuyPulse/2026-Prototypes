package com.stuypulse.robot.subsystems;

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
        INTAKE(Settings.Intake.INTAKE), //TODO: add to settings
        OUTTAKE(Settings.Intake.OUTTAKE),
        STOW(Settings.Intake.STOW);

        private double dutyCycle;

        private IntakeState(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return dutyCycle;
        }
    }

    public enum IntakePosition {
        UP(Settings.Intake.UP),
        DOWN(Settings.Intake.DOWN),
        STOP(Settings.Intake.STOP);

        private double speed; //TODO: update variable name when using PID controller and etc

        private IntakePosition(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
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
