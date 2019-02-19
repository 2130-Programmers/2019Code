package org.usfirst.frc2130.AlphaHydrae.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2130.AlphaHydrae.Robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class elevatorPIDSubsystem extends PIDSubsystem {

    private DoubleSolenoid climbingFeetSolenoid;
    private DigitalInput lowProx;
    private DigitalInput midProx;
    private DigitalInput maxProx;
    private WPI_TalonSRX elevatorMotorMaster;
    private WPI_TalonSRX elevatorMotorSlave;
    private WPI_TalonSRX rearFoot;
    private Solenoid elevatorBrakeSolenoid;
    private String desiredProx;

    // Initialize your subsystem here
    public elevatorPIDSubsystem() {
        // TODO: These gains need to be updated. Start with P gain.
        super("elevatorPIDSubsystem", 0.001, 0.0, 0.0);
        setAbsoluteTolerance(100);
        getPIDController().setContinuous(false);
        getPIDController().setName("elevatorPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
        
        // Make sure to disable the brake on the rear leg
        climbingFeetSolenoid = new DoubleSolenoid(0, 5, 6);
        // Make sure the system is off to start with
        disable();

        lowProx = new DigitalInput(0);
        addChild("lowProx",lowProx);
        
        
        midProx = new DigitalInput(1);
        addChild("midProx",midProx);
        
        
        maxProx = new DigitalInput(2);
        addChild("maxProx",maxProx);
        
        
        elevatorMotorMaster = new WPI_TalonSRX(8);
        
        elevatorMotorSlave = new WPI_TalonSRX(9);

        rearFoot = new WPI_TalonSRX(6);

        elevatorBrakeSolenoid = new Solenoid(4);

        elevatorMotorSlave.follow(elevatorMotorMaster);

        desiredProx = "Low";
    }

    @Override
    public void initDefaultCommand() {
        // Unused. No default command should be set for this subsystem.
    }

    @Override
    protected double returnPIDInput() {
        return elevatorEncoderValue();
    }

    @Override
    protected void usePIDOutput(double output) {
        elevatorMotorMaster.pidWrite(-output);
    }

    public void stopClimb() {
        // Turn everything off
        setBrakeState(false);
        disable();
        elevatorMotorMaster.set(0);
    }

    public boolean setpointCheck(int setpoint) {

        if (getProx("Mid")) {
            return true;
        }

        if (elevatorEncoderValue() >= setpoint) {
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
    	if (elevatorEncoderValue() < 4000) {
            setMaxMinOutput(0.1, -0.7);
    	}
    	if (elevatorEncoderValue() >= 4000) {
            setMaxMinOutput(0.5, -0.7);
    	}
    	
    	if(motorOutput() < 0 && getProx("Max") == true) {
    		disable();
    		elevatorMotorMaster.set(0);
    	}
    	if(motorOutput() > 0 && getProx("Low") == true) {
    		disable();
    		elevatorMotorMaster.set(0);
    	}
    	
    	/* Currently this gives our loop a tolerance of 10 native encoder units. Once we are within this range,
    	 * the brake will engage and the loop will end, holding us position until a new setpoint is called*/
    	if(elevatorEncoderValue() > setpoint - 500 && elevatorEncoderValue() < setpoint + 500 && elevatorEncoderValue() > 1000) {
    		disable();
    		setBrakeState(false);
    		stopAllMotors();
    	}
    	else {
            setSetpoint(setpoint);
    		enable();
    		setBrakeState(true);
    	}
    	
    	//if(elevatorHeight() < setpoint && elevatorHeight() <= 1000 && motorOutput() < 0) {
    	//	disable();
    	//	stopAllMotors();
        //}
    }

    public void engageClimbingFeet() {
         climbingFeetSolenoid.set(Value.kForward);
    }

    public void moveFoot() {
        rearFoot.set(Robot.oi.operatorJoystick.getRawAxis(3)); //TODO: Set the AXIS
    }

    public void killFoot() {
        rearFoot.set(0);
    }

    public void returnFoot() {
        if (Robot.climbingPIDSubsystem.getProx(false)) {
            rearFoot.configPeakOutputReverse(-0.2);
        } else {
            rearFoot.configPeakOutputReverse(-0.8);
        }
        
        rearFoot.set(-1);
    }

    // This returns what prox sensor we are hitting
    public boolean getProx(String prox) {
        if (prox == "Max") {
            return !maxProx.get();
        } else if (prox == "Mid") {
            return !midProx.get();
        } else {
            return !lowProx.get();
        }
    }

    public void setBrakeState(boolean state){
        elevatorBrakeSolenoid.set(state);
    }

    public void resetpeakoutput() {
    	elevatorMotorMaster.configPeakOutputForward(1, 0);
    	elevatorMotorMaster.configPeakOutputReverse(-1, 0);
    }

    // This moves the elevator using a boolean
    public void moveElevator(boolean climbingMode, double speed) {
        if (!climbingMode) {
            if (getProx(desiredProx)) {
                disableElevator();
            } else {
                if (elevatorEncoderValue() < 8000) {
                    setMaxMinOutput(0.1,-0.7);
                }
                if (elevatorEncoderValue() >= 8000) {
                    setMaxMinOutput(0.5,-0.7);
                }
                setBrakeState(true);
                elevatorMotorMaster.set(Robot.oi.operatorJoystick.getRawAxis(1));
            }  
        } else {
            elevatorMotorMaster.set(speed);
        }
    }

    public void zeroTheTalon() {
    	elevatorMotorSlave.setSelectedSensorPosition(0, 0, 0);
    }

    public void homeEncoder() {
    	if (getProx("Low")) {
    	    zeroTheTalon();
    	}
    }

    /* This tells us the output of the Talon, and whether its in the forward or backward direction */
    public double motorOutput() {
    	return elevatorMotorMaster.getMotorOutputPercent();
    }

    // Disables the elevator, and turns on the brake
    public void disableElevator() {
        elevatorMotorMaster.set(0);
        setBrakeState(false);
    }

    public void setMaxMinOutput(double max, double min) {
        elevatorMotorMaster.configPeakOutputForward(max, 0);
        elevatorMotorMaster.configPeakOutputReverse(min, 0);
    }

    // Makes sure the elevator brake is not on when we start
    public void startupRoutine() {
        setBrakeState(false);
        desiredProx = "Low";
    }

    // Sets the output we want from the prox sensor
    public void setDesiredOutput(String setDesiredProx) {
        desiredProx = setDesiredProx;
    }

    // Returns our info from the prox
    public String returnDesiredProx() {
        return desiredProx;
    }

    public double elevatorEncoderValue() {
        return -elevatorMotorSlave.getSelectedSensorPosition(0);
    }

    public void zeroElevatorEncoder() {
        elevatorMotorSlave.setSelectedSensorPosition(0, 0, 0);
    }

    public void stopAllMotors() {
    	elevatorMotorMaster.stopMotor();
    }
}
