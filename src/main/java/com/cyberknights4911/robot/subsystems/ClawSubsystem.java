package com.cyberknights4911.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.cyberknights4911.robot.constants.Constants;

import edu.wpi.first.wpilibj2.command.Subsystem;
import libraries.cheesylib.drivers.TalonFXFactory;

/**
 * Subsystem for controlling the Claw.
 */
public final class ClawSubsystem implements Subsystem {

    private TalonFX mFXMotor;



    public void Claw(){
        mFXMotor = TalonFXFactory.createDefaultTalon(Constants.CLAW_MOTOR_ID, Constants.CANIVORE_NAME);

        mFXMotor.setInverted(false);


    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void collect() {
        mFXMotor.set(ControlMode.PercentOutput, 0.3);
    }

    public void back() {
        mFXMotor.set(ControlMode.PercentOutput, -0.3);
    }

    public void stop() {
        mFXMotor.set(ControlMode.PercentOutput, 0.0);
    }







}

