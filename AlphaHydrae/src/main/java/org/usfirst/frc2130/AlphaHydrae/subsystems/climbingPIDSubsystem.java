package org.usfirst.frc2130.AlphaHydrae.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc2130.AlphaHydrae.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class climbingPIDSubsystem extends PIDSubsystem {

    private WPI_TalonSRX rearLiftMotor;
    private DigitalInput bottomProx;
    private DigitalInput topProx;

    private double collectiveOutput;

    // Initialize your subsystem here
    public climbingPIDSubsystem() {
        // TODO: These gains need to be updated. Start with P gain.
        super("climbingPIDSubsystem", 0.001, 0.0, 0.0);

        // Initialize collective output to disable climbing
        collectiveOutput = 0.0;

        setAbsoluteTolerance(0.2);
        getPIDController().setContinuous(false);
        getPIDController().setName("climbingPIDSubsystem", "PIDSubsystem Controller");
        LiveWindow.add(getPIDController());
        
        rearLiftMotor = new WPI_TalonSRX(7);
        addChild("RearLiftMotor", rearLiftMotor);
        rearLiftMotor.setInverted(false);

        topProx = new DigitalInput(4);
        bottomProx = new DigitalInput(3);

        // Make sure the system is off to start with
        disable();
    }

    @Override
    public void initDefaultCommand() {
        // Unused. No default command should be set for this subsystem.
    }

    @Override
    protected double returnPIDInput() {
        // This may need to be inverted to get the correct direction.
        //
        // Assume here that positive pitch means the the front of the robot
        // is tilted up, that means that the front motor needs to slow down.
        return navXPitch();

    }

    public double navXPitch() {
        return Robot.navXSub.navXPitch();
    }

    @Override
    protected void usePIDOutput(double output) {
        // The PID output represents the controller's attempt to compensate for
        // perceived pitch error. Following our convention for positive pitch
        // meaning the robot is tilted up, for a simple P contoller:
        //
        //      output = Kp * error = Kp * (target - actual)
        //      output = Kp * (0 - pitch)
        //
        // So we see that if there is positive pitch, the controller output will
        // be negative. That means we should ADD the output of the controller to the
        // front motor and SUBTRACT the output from the back motor.

        // Add controller output on front to decrease speed if pitch is positive
        Robot.elevatorPIDSubsystem.moveElevator(true, (-collectiveOutput) + output);

        // Subtract controller output on back to increase speed if pitch is positive
        rearLiftMotor.set(collectiveOutput + output);
        
    }

    public void startClimb(double collectiveCommand) {
        // Starts climbing by setting the collective command and enabling the controller.
        //
        // By default, we target 0 tilt. If we want to get fancy we may want to maintain the
        // orientation the robot is when it starts to climb by remembering the pitch when the
        // command is sent. It may be more desirable for it to self-level in the end though.

        // Target 0 pitch
        setSetpoint(0.0);

        // Set the internal collective output
        collectiveOutput = collectiveCommand;

        

        // Start the PID controller
        enable();
    }

    public void stopClimb() {
        // Turn everything off
        Robot.elevatorSubsystem.setBrakeState(false);
        disable();
        rearLiftMotor.set(0.0);
        Robot.elevatorSubsystem.stopAllMotors();;
        collectiveOutput = 0.0;
    }

    public double footEncoderValue() {
        return -rearLiftMotor.getSelectedSensorPosition(0);
    }

    public void zeroEncoder() {
        rearLiftMotor.setSelectedSensorPosition(0,0,0);
    }

    public boolean getProx(boolean useTopProx) {
        if (useTopProx) {
            return topProx.get();
        }  else {
            return bottomProx.get();
        }
    }

    public void homeEncoder() {
    	if (getProx(false) == true) {
    	    zeroEncoder();
    	}
    }

    public boolean isAtMovableLevel() {

        if (getProx(true)) {
            return true;
        }

        if (footEncoderValue() >= 100) {
            return true;
        } else {
            return false;
        }
    }

    public boolean retractedLevel() {

        if (footEncoderValue() <= 10) {
            return true;
        } else {
            return false;
        }
    }

}
