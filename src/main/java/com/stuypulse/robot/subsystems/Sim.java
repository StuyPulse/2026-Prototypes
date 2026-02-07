package com.stuypulse.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sim extends SubsystemBase {
    public static final Sim instance;
    public static int randomAngle = 0;

    static {
        instance = new SimImpl();
    }

    public static Sim getInstance() {
        return instance;
    }

    public enum State { //UPDATE ANGLE TO BE THE RANDOM ANGLE
        TOTARGET(() -> randomAngle, 1), //TODO: make it be random and have this value change via command. SPEED SHOULD BE MUTLIPLIED BY VOLTAGE!!
        STOP(() -> 0, 0); //TODO: speed values are good as they are
        
        Supplier<Integer> angle;
        int speed;

        private State(Supplier<Integer> angle, int speed) {
            this.angle = angle;
            this.speed = speed;
        }

        public Supplier<Integer> getAngle() {
            return angle;
        }

        public int getSpeed() {
            return speed;
        }
        //TODO: if command stuff doesn't work out, make a set speed and etc...
    }

    private State state;

    public Sim() {
        state = State.STOP;
    }

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setRandomAngle(int angle) {
        this.randomAngle = angle;
    }

    @Override
    public void periodic() {
        //TODO: add stuff here? -> not necessary ig?
    }
    
}
