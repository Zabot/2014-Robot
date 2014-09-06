package org.frc4931.robot.command;

import org.frc4931.robot.Copyable;

import edu.wpi.first.wpilibj.command.Command;

public abstract class CommandBase extends Command implements Copyable{
	protected final boolean isContinuous;
	private boolean hasExecuted = false;
	
	protected CommandBase(){
		this(true);
	}
	
	protected CommandBase(boolean isContinuous){
		this.isContinuous=isContinuous;
	}
	
	public Copyable copy(){
		throw new UnsupportedOperationException("Copy is not defined for this command");
	}
	
	protected void initialize(){
		hasExecuted=false;
		//CompetitionRobot.output("Command <"+getName()+"> initialized.");
	}
	
	protected final void execute(){
		if(isContinuous||(!hasExecuted)){
			doExecute();
			hasExecuted = true;
		}
	}
	protected abstract void doExecute();
	
	protected void interrupted() {
		//CompetitionRobot.output("Command <"+getName()+"> canceled.");
		end();
	}
	
	protected void end() {
		hasExecuted=false;
		//CompetitionRobot.output("Command <"+getName()+"> has ended.");
	}
}
