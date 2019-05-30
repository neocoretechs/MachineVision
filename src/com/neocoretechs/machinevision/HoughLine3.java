package com.neocoretechs.machinevision;

public class HoughLine3 extends HoughLine {
	public double d;
	public HoughLine3(double theta, double r, double d) {
		super(theta, r);
		this.d = d;
	}

}
