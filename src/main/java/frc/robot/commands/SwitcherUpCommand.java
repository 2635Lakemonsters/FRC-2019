/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Switcher;
import frc.robot.subsystems.Switcher.SwitcherState;

public class SwitcherUpCommand extends Command {
  public SwitcherUpCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.switcher);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.switcher.moveSwitch(Switcher.Position.CARGO);
    SwitcherState nextPosition = Robot.switcher.getNextPosition();
    //System.out.println("switcherupcommand init");
    Robot.switcher.moveSwitch(nextPosition);
    if(nextPosition == SwitcherState.HATCH){
      Robot.elevator.setTargetHeight(Robot.elevator.getSwappedState());
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
   // Robot.switcher.moveSwitch();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.switcher.moveSwitch();
  }
}

