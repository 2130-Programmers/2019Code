package org.usfirst.frc2130.AlphaHydrae.subsystems;

import org.usfirst.frc2130.AlphaHydrae.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class elevatorPIDSubsystem extends PIDSubsystem {

    private DoubleSolenoid climbingFeetSolenoid;

    // Initialize your subsystem here
    public elevatorPIDSubsystem() {
        // TODO: These gains need to be updated. Start with P gain.
        super("elevatorPIDSubsystem", 0.01, 0.0, 0.0);
        setAbsoluteTolerance(100);
        getPIDController().setContinuous(false);
        getPIDController().setName("elevatorPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
        
        // Make sure to disable the brake on the rear leg
        climbingFeetSolenoid = new DoubleSolenoid(0, 5, 6);
        // Make sure the system is off to start with
        disable();
    }

    @Override
    public void initDefaultCommand() {
        // Unused. No default command should be set for this subsystem.
    }

    @Override
    protected double returnPIDInput() {

        return elevatorHeight();

    }

    @Override
    protected void usePIDOutput(double output) {

        moveMotors(-output);
     
    }

    public double elevatorHeight() {
        return Math.abs(Robot.elevatorSubsystem.elevatorEncoderValue());
    }

    public void startClimb(double setpoint) {

        setSetpoint(setpoint);
    }

    public void moveMotors (double speed) {
        Robot.elevatorSubsystem.moveElevator(true, speed);
    }

    public void stopClimb() {
        // Turn everything off
        Robot.elevatorSubsystem.setBrakeState(false);
        disable();
        moveMotors(0.0);
    }

    /* This tells us the output of the Talon, and whether its in the forward or backward direction */
    public double motorOutput() {
    	return Robot.elevatorSubsystem.motorOutput();
    }

    public void setMaxMinOutput(double max, double min) {
        Robot.elevatorSubsystem.setMaxMinOutput(max, min);
    }

    public boolean setpointCheck(int setpoint) {

        if (Robot.elevatorSubsystem.getProx("Mid")) {
            return true;
        }

        if (elevatorHeight() >= setpoint) {
            return true;
        } else {
            return false;
        }
    }

    /* The following function will run our elevator to several desired setpoints, activating a pnuematic
     * break when it reaches its destination */
    public void setSetpointWithBrake(int setpoint) {    
    	/* This code functions as a sort of dampener. When the elevator is below a certain point it slows down and
    	 * gently parks at the bottom prox */
    	if (elevatorHeight() < 4000) {
            setMaxMinOutput(0.1, -0.7);
    	}
    	if (elevatorHeight() >= 4000) {
            setMaxMinOutput(0.5, -0.7);
    	}
    	
    	if(motorOutput() < 0 && Robot.elevatorSubsystem.getProx("Max") == true) {
    		disable();
    		moveMotors(0);
    	}
    	if(motorOutput() > 0 && Robot.elevatorSubsystem.getProx("Low") == true) {
    		disable();
    		moveMotors(0);
    	}
    	
    	/* Currently this gives our loop a tolerance of 10 native encoder units. Once we are within this range,
    	 * the brake will engage and the loop will end, holding us position until a new setpoint is called*/
    	if(elevatorHeight() > setpoint - 500 && elevatorHeight() < setpoint + 500 && elevatorHeight() > 1000) {
    		disable();
    		Robot.elevatorSubsystem.setBrakeState(false);
    		stopAllMotors();
    	}
    	else {
            setSetpoint(setpoint);
    		enable();
    		Robot.elevatorSubsystem.setBrakeState(true);
    	}
    	
    	if(elevatorHeight() < setpoint && elevatorHeight() <= 1000 && motorOutput() < 0) {
    		disable();
    		stopAllMotors();
        }
    }

    public void stopAllMotors() {
        Robot.elevatorSubsystem.stopAllMotors();
    }

    public void engageClimbingFeet() {
         climbingFeetSolenoid.set(Value.kForward);
    }
}
