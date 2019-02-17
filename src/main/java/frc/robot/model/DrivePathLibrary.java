/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.model;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.PathCommand;

/**
 * Add your docs here.
 */
public class DrivePathLibrary {

    public static CommandGroup CenterLeftToCargoLeft()
    {
        CommandGroup output = new CommandGroup();

        //PathCommand pathCmd = new PathCommand("CenterLeftToCargoLeft");
       // output.addSequential(pathCmd);

        return output;
    }
}
