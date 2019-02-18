/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.model;

import frc.robot.subsystems.Elevator.Height;
import frc.robot.subsystems.Switcher.Position;
import frc.robot.subsystems.Flower.FlowerBud;
import frc.robot.subsystems.Flower.FlowerIO;
import frc.robot.Robot;


/**
 * Add your docs here.
 */
public class GameToolStateMachine {
    public GameTools currentState;

    public GameToolStateMachine(){
        currentState = GameTools.INITIAL;
    }
    
    public void increment() {
        GameTools nextState = incrementState();
        Robot.elevator.setTargetHeight(nextState.elevator);
        Robot.switcher.moveSwitch(nextState.switcher);
        Robot.flower.setFlowerIO(nextState.io);
        Robot.flower.setFlowerBud(nextState.bud);
        currentState = nextState;
        System.out.println(currentState);
    }

    GameTools incrementState() {
        switch(currentState) {
            case INITIAL:
                return GameTools.CARGO1;
            case CARGO1:
                return GameTools.CARGO2;
            case CARGO2:
                return GameTools.CARGO3;
            case CARGO3:
                return GameTools.CARGO4;
            case CARGO4:
                return GameTools.CARGO4;
            case HATCH1B:
                return GameTools.HATCH2B;
            case HATCH2B:
                return GameTools.HATCH3B;
            case HATCH1F:
                return GameTools.HATCH2F;
            case HATCH2F:
                return GameTools.HATCH3F;
            default:
                return currentState;
        }
    }

    public void decrement() {
        GameTools nextState = decrementState();
        Robot.elevator.setTargetHeight(nextState.elevator);
        Robot.switcher.moveSwitch(nextState.switcher);
        Robot.flower.setFlowerIO(nextState.io);
        Robot.flower.setFlowerBud(nextState.bud);
        currentState = nextState;
        System.out.println(currentState);
    }

    GameTools decrementState() {
        switch(currentState) {
            case CARGO1:
                return GameTools.CARGO1;
            case CARGO2:
                return GameTools.CARGO1;
            case CARGO3:
                return GameTools.CARGO2;
            case CARGO4:
                return GameTools.CARGO3;
            case HATCH2B:
                return GameTools.HATCH1B;
            case HATCH3B:
                return GameTools.HATCH2B;
            case HATCH2F:
                return GameTools.HATCH1F;
            case HATCH3F:
                return GameTools.HATCH2F;
            default:
                return currentState;
        }
    }

    public void swap(){
        GameTools nextState = swapState();
        Robot.elevator.setTargetHeight(nextState.elevator);
        Robot.switcher.moveSwitch(nextState.switcher);
        Robot.flower.setFlowerIO(nextState.io);
        Robot.flower.setFlowerBud(nextState.bud);
        currentState = nextState;
        System.out.println(currentState);
    }

    GameTools swapState(){
        switch(currentState) {
            case CARGO2:
                return GameTools.HATCH1B;
            case CARGO3:
                return GameTools.HATCH1B;
            case CARGO4:
                return GameTools.HATCH1B;
            case HATCH1B:
                return GameTools.CARGO2;
            case HATCH2B:
                return GameTools.CARGO2;
            case HATCH3B:
                return GameTools.CARGO2;
            default:
                return currentState;
        }
    }

    public void flower(){
        GameTools nextState = flowerState();
        Robot.elevator.setTargetHeight(nextState.elevator);
        Robot.switcher.moveSwitch(nextState.switcher);
        Robot.flower.setFlowerIO(nextState.io);
        Robot.flower.setFlowerBud(nextState.bud);
        currentState = nextState;
        System.out.println(currentState);
    }

    GameTools flowerState(){
        switch(currentState) {
            case HATCH1B:
                return GameTools.HATCH1F;
            case HATCH2B:
                return GameTools.HATCH2F;
            case HATCH3B:
                return GameTools.HATCH3F;
            case HATCH1F:
                return GameTools.HATCH1B;
            case HATCH2F:
                return GameTools.HATCH2B;
            case HATCH3F:
                return GameTools.HATCH3B;
            default:
                return currentState;
        }
    }

    public static enum GameTools {
        INITIAL(Height.GROUND, Position.FLOOR, FlowerBud.BUD, FlowerIO.IN),
        CARGO1(Height.GROUND, Position.CARGO, FlowerBud.BUD, FlowerIO.IN),
        CARGO2(Height.LEVEL1B, Position.CARGO, FlowerBud.BUD, FlowerIO.IN),
        CARGO3(Height.LEVEL2B, Position.CARGO, FlowerBud.BUD, FlowerIO.IN),
        CARGO4(Height.LEVEL3B, Position.REAR, FlowerBud.BUD, FlowerIO.IN),
        HATCH1B(Height.LEVEL1H, Position.HATCH, FlowerBud.BUD, FlowerIO.OUT),
        HATCH2B(Height.LEVEL2H, Position.HATCH, FlowerBud.BUD, FlowerIO.OUT),
        HATCH3B(Height.LEVEL3H, Position.HATCH, FlowerBud.BUD, FlowerIO.OUT),
        HATCH1F(Height.LEVEL1H, Position.HATCH, FlowerBud.FLOWER, FlowerIO.OUT),
        HATCH2F(Height.LEVEL2H, Position.HATCH, FlowerBud.FLOWER, FlowerIO.OUT),
        HATCH3F(Height.LEVEL3H, Position.HATCH, FlowerBud.FLOWER, FlowerIO.OUT);

        public Height elevator;
        public Position switcher;
        public FlowerBud bud;
        public FlowerIO io;
        private GameTools(Height elevator, Position switcher, FlowerBud bud, FlowerIO io) {
            this.elevator = elevator;
            this.switcher = switcher;
            this.bud = bud;
            this.io = io;
        }
    }
}