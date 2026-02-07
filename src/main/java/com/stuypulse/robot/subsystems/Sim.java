package com.stuypulse.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sim extends SubsystemBase {
    public static final Sim instance;

    static {
        instance = new SimImpl();
    }

    public static Sim getInstance() {
        return instance;
    }

    public enum State { //UPDATE ANGLE TO BE THE RANDOM ANGLE
        TOTARGET(1, 1), //TODO: make it be random and have this value change via command. SPEED SHOULD BE MUTLIPLIED BY VOLTAGE!!
        STOP(0, 0); //TODO: speed values are good as they are

        int angle;
        int speed;

        private State(int angle, int speed) {
            this.angle = angle;
            this.speed = speed;
        }

        public int getAngle() {
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

    @Override
    public void periodic() {
        //TODO: add stuff here? -> not necessary ig?
    }
    
}
