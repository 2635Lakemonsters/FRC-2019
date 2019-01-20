/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class PathTestCommand extends Command {
  public PathTestCommand() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.driveSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("--PathTestCommand Init--");
    Robot.driveSubsystem.PathInit();
    try {
      Robot.driveSubsystem.LoadPath("/home/lvuser/deploy/Straight.left.pf1.csv","/home/lvuser/deploy/Straight.right.pf1.csv");
    } catch(IOException e) {
      System.out.println("Could Not Load Path in PathTestCommand");
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveSubsystem.MotionMagic();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    boolean isDone = Robot.driveSubsystem.isDone();
    if(isDone){
      System.out.println("Took " + (System.currentTimeMillis() - Robot.driveSubsystem.startTime) + " ms");
    }
    return isDone;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
