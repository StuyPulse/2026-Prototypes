package com.stuypulse.robot.subsystems;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


public class IntakeSim extends Intake{
    // private final TalonFX ACTUAL_POSITION_MOTOR;
    Mechanism2d armCanvas = new Mechanism2d(4, 4);
    MechanismRoot2d armRoot = armCanvas.getRoot("Arm Simulation", 2, 2);
    MechanismLigament2d armActual = armRoot.append(new MechanismLigament2d("Intake Positional Arm (Actual)", 2, 90));

    //DCMotorSim POSITIONAL_MOTOR_DCMOTOR;
    //TalonFXSimState POSITIONAL_MOTOR_TALON;
    SingleJointedArmSim intake_arm_sim;

    PIDController controller;
    ArmFeedforward FFcontroller;



    public IntakeSim() {
        //POSITIONAL_MOTOR_DCMOTOR = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, Constants.Intake.Intake_Extension_GEAR_RATIO), DCMotor.getKrakenX60(1));
        
        intake_arm_sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            Constants.Intake.Intake_Extension_GEAR_RATIO, 
            0.001,
            5,
            Math.toRadians(Constants.Intake.INTAKE_POSITIONAL_MIN_ANGLE),
            Math.toRadians(Constants.Intake.INTAKE_POSITIONAL_MAX_ANGLE),
            false,
            Math.toRadians(IntakePosition.UP.getTargetAngle().get())
        ); 

        
    }

    //not really neccesary (and if anything I should probs override it from Intake)
    public Supplier<Double> getCurrentAngle() {
        return () -> Math.toDegrees(intake_arm_sim.getAngleRads());
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("arm Mechanism", armCanvas);
        armActual.setColor(new Color8Bit(Color.kBlue));
        armActual.setLength(2);
        armActual.setAngle(Math.toDegrees(intake_arm_sim.getAngleRads()));
    
        //not the best practice but it let's users control the PID on the inside. Probs better way to do it
        controller = new PIDController( 
            Gains.Intake.kP.getAsDouble(),
            Gains.Intake.kI.getAsDouble(), 
            Gains.Intake.kD.getAsDouble()
        );

        FFcontroller = new ArmFeedforward(
            Gains.Intake.kS.getAsDouble(),
            Gains.Intake.kG.getAsDouble(),
            Gains.Intake.kV.getAsDouble(),
            Gains.Intake.kA.getAsDouble(),
            0.020
        );

        double pidOutput = controller.calculate(intake_arm_sim.getAngleRads(), Math.toRadians(getIntakePosition().getTargetAngle().get())); 
        double ffOutput = FFcontroller.calculate(controller.getSetpoint(), 1); //TODO: add velocity later
        double voltage = SLMath.clamp(pidOutput + ffOutput, -12.0, 12.0);
        
        intake_arm_sim.setInputVoltage(voltage);
        //intake_arm_sim.setState(Math.toRadians(getIntakePosition().getTargetAngle().get()), 0.1);
        intake_arm_sim.update(0.0020);

        SmartDashboard.putString("INTAKE SIM / POSITIONAL/ STATE", Intake.getInstance().getIntakePosition().toString());

        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ PID OUTPUT", pidOutput);
        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ FF OUTPUT", ffOutput);

        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ TARGET DEGREES", getIntakePosition().getTargetAngle().get()); //should be either 0 or 1.54...
        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ TARGET RADIANS", Math.toRadians(getIntakePosition().getTargetAngle().get())); 

        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ CURRENT ANGLE (DEG) ", getCurrentAngle().get());

        SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ VOLTAGE ", voltage);

        // SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ TOP ANGLE (RAD)", Math.toRadians(IntakePosition.UP.getTargetAngle().get()));
        //SmartDashboard.putNumber("INTAKE SIM / POSITIONAL/ PID SETPOINT", controller.getSetpoint());
    }
}
