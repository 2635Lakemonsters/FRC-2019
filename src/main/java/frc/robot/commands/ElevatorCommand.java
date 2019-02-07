package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Elevator.Height;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorCommand extends Command {
	public Height height;
    public ElevatorCommand(Height height) {
        // Use requires() here to declare subsystem dependencies
        //requires(Robot.elevator);
        this.height = height;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.setTargetHeight(height);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() { //Is continuously called until the isFinished command is returned as true.
    	//Robot.elevator.setTargetHeight(height);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() { 
    	boolean elevatorIsFinished = Robot.elevator.isWithinTolerance(height);
    	if(elevatorIsFinished){
    		System.out.println("Elevator Command finished");
    	}
        return elevatorIsFinished;
    	
    	//return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
