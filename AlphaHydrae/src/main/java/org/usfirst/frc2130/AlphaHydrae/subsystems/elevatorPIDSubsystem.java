package org.usfirst.frc2130.AlphaHydrae.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.usfirst.frc2130.AlphaHydrae.Robot;
import org.usfirst.frc2130.AlphaHydrae.commands.moveElevatorCommand;

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
    public boolean atClimbingSetpoint;

    // Initialize your subsystem here
    public elevatorPIDSubsystem() {
        // TODO: These gains need to be updated. Start with P gain.
        super("elevatorPIDSubsystem", 0.0009, 0.0, 0.0);
        setAbsoluteTolerance(100);
        getPIDController().setContinuous(false);
        getPIDController().setName("elevatorPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
        
        // Make sure to disable the brake on the rear leg
        climbingFeetSolenoid = new DoubleSolenoid(0, 5, 6);
        // Make sure the system is off to start with
        //disable();

        atClimbingSetpoint = false;

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
        setDefaultCommand(new moveElevatorCommand());
    }

    @Override
    protected double returnPIDInput() {
        return elevatorEncoderValue();
    }

    @Override
    protected void usePIDOutput(double output) {
        elevatorMotorMaster.set(-output);
    }

    public double elevatorEncoderValue() {
        return -elevatorMotorSlave.getSelectedSensorPosition(0);
    }

    public void setBrakeState (boolean on) {
        elevatorBrakeSolenoid.set(on);
    }

    public void stopAllMotors() {
        elevatorMotorMaster.set(0);
        elevatorMotorSlave.set(0);
    }

    public void resetpeakoutput() {
    	elevatorMotorMaster.configPeakOutputForward(1, 0);
    	elevatorMotorMaster.configPeakOutputReverse(-1, 0);
    }

    public void zeroTheTalon() {
    	elevatorMotorSlave.setSelectedSensorPosition(0, 0, 0);
    }

    public void homeEncoder() {
    	if (getProx("Low")) {
    	    zeroTheTalon();
    	}
    }

    public double motorOutput() {
    	return elevatorMotorMaster.getMotorOutputPercent();
    }

    public boolean getProx(String prox) {
        if (prox == "Max") {
            return !maxProx.get();
        } else if (prox == "Mid") {
            return !midProx.get();
        } else {
            return !lowProx.get();
        }
    }

    public String returnDesiredProx() {
        return desiredProx;
    }

    public void startupRoutine() {
        setBrakeState(false);
        desiredProx = "Low";
    }

    public void moveElevator(double speed) {{
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
        }
    }

    public void applyPower(double speed) {
        elevatorMotorMaster.set(speed);
    }

    public void setMaxMinOutput(double max, double min) {
        elevatorMotorMaster.configPeakOutputForward(max, 0);
        elevatorMotorMaster.configPeakOutputReverse(min, 0);
    }

    // Disables the elevator, and turns on the brake
    public void disableElevator() {
        stopAllMotors();
        setBrakeState(false);
    }

    public void setDesiredOutput(String setDesiredProx) {
        desiredProx = setDesiredProx;
    }
    
    public void setSetpointWithBrake(int setpoint) {    
    	/* This code functions as a sort of dampener. When the elevator is below a certain point it slows down and
    	 * gently parks at the bottom prox */
    	if (elevatorEncoderValue() < 4000) {
            setMaxMinOutput(0.1, -0.7);
    	}
    	if (elevatorEncoderValue() >= 4000) {
            setMaxMinOutput(0.5, -0.7);
    	}
    	
    	if(motorOutput() < 0 && getProx("Max") == true && setpoint < 15000) {
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
    	
    	//if(getProx("Low") && setpoint == 120) {
    	//	disable();
    	//	stopAllMotors();
        //}
    }

    public boolean returnAtClimbingSetpoint() {
        return atClimbingSetpoint;
    }

    public void climbingSetpointWithBrake(int setpoint) {    
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
    		setBrakeState(false);
            stopAllMotors();
            atClimbingSetpoint = true;
            disable();
            
    	}
    	else {
            setSetpoint(setpoint);
    		enable();
            setBrakeState(true);
            atClimbingSetpoint = false;
    	}
    	
    	//if(getProx("Low") && setpoint == 120) {
    	//	disable();
    	//	stopAllMotors();
        //}
    }

    public void engageClimbingFeet() {
        climbingFeetSolenoid.set(Value.kForward);
    }

    public void disengageClimbingFeet() {
        climbingFeetSolenoid.set(Value.kReverse);
    }

   public boolean setpointCheck(int setpoint) {

        if (getProx("Max")) {
            return true;
        }

        if (elevatorEncoderValue() < (setpoint + 200) && elevatorEncoderValue() > (setpoint - 200)) {
            return true;
        } else {
            return false;
        }
    }
}
