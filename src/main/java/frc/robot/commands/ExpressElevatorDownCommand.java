package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Height;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ExpressElevatorDownCommand extends Command {

    public ExpressElevatorDownCommand() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.elevator);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Height newTargetHeight;
    	switch(Robot.elevator.currentTargetHeight) {
		case CLIMB:
			Robot.elevator.setTargetHeight(Height.SCALE);
			 break;
		case SCALE:
			Robot.elevator.setTargetHeight(Height.SWITCH);
	         break;
		case SWITCH:
			Robot.elevator.setTargetHeight(Height.GROUND);
	         break;
		case GROUND:
			Robot.elevator.setTargetHeight(Height.GROUND);
			 break;
	    default:
	        
		}
    	System.out.println("Initialize called");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}