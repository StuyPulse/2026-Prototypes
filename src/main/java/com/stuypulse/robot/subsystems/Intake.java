package com.stuypulse.robot.subsystems;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    private static final Intake instance;

    static {
        if(Robot.isReal()) {
            instance = new IntakeImpl();
        }
        else {
            instance = new IntakeSim();
        }
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
        DOWN(() -> Settings.Intake.DOWN.get());

        private Supplier<Double> targetAngle; //TODO: update variable name when using PID controller and etc

        private IntakePosition(Supplier<Double> targetAngle) {
            this.targetAngle = targetAngle;
        }

        public Supplier<Double> getTargetAngle() {
            return targetAngle;
        }
    }

    IntakePosition position;
    IntakeState state;

    protected Intake() {
        position = IntakePosition.UP;
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

    //public abstract Supplier<Rotation2d> getTargetRotations();

}
