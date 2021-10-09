package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutonomousParameter {

	private String name;
	private String type;
	private boolean boolVal;
	private int intVal;
	private Telemetry telem;
	private boolean selected;

	public AutonomousParameter(String name, boolean defVal, Telemetry telem) {
		this.name = name;
		this.boolVal = defVal;
		this.intVal = 0;
		this.type = "boolean";
		this.telem = telem;
	}

	public AutonomousParameter(String name, int defVal, Telemetry telem) {
		this.name = name;
		this.boolVal = false;
		this.intVal = defVal;
		this.type = "integer";
		this.telem = telem;
	}

	public void setVal(boolean val) {
		this.boolVal = val;
	}

	public void setVal(int val) {
		this.intVal = val;
	}

	public boolean getBoolVal() {
		return this.boolVal;
	}

	public int getIntVal() {
		return this.intVal;
	}

	public void setType(String type) {
		this.type = type;
	}

	public String getType() {
		return this.type;
	}

	public void setName(String name) {
		this.name = name;
	}

	public String getName() {
		return this.name;
	}

	public void setSelected(boolean selected) {
		this.selected = selected;
	}

	public void incrementVal(int incVal) {
		this.intVal += incVal;
		this.boolVal = !this.boolVal;
	}

	public void display() {
		if (!selected) {
			if (type == "integer")
				this.telem.addLine("Name: " + this.name + "  Value: " + this.intVal + "");
			else if (type == "boolean")
				this.telem.addLine("Name: " + this.name + "  Value: " + this.boolVal + "");
			else
				this.telem.addLine("ERROR: UNKNOWN TYPE '" + this.type + "' ON PARAMETER '" + this.name + "'");
		} else {
			if (type == "integer")
				this.telem.addLine(">>> Name: " + this.name + "  Value: " + this.intVal + " <<<");
			else if (type == "boolean")
				this.telem.addLine(">>> Name: " + this.name + "  Value: " + this.boolVal + " <<<");
			else
				this.telem.addLine(">>> ERROR: UNKNOWN TYPE '" + this.type + "' ON PARAMETER '" + this.name + "' <<<");
		}
	}

	public void display(Telemetry telem) {
		this.telem = telem;
		display();
	}

}
