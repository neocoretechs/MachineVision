package com.neocoretechs.machinevision.test;

import com.neocoretechs.machinevision.hough3d.Vector4d;

public class AngleTest {
	static Vector4d nv1 = new Vector4d(0, 1, 1);
	static Vector4d nv2 = new Vector4d(.009, 1, 1);
	public static void main(String[] args) {
		Vector4d cross = nv1.multiplyVectorial(nv2);
		double dot = nv1.and(nv2); // dot
		double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
		double angle = Math.atan2(crossMag, dot);
		System.out.println(nv1+"\r\n"+nv2+"\r\n"+Math.toDegrees(angle)+" "+Math.toDegrees(Math.acos(dot)));
	}
}
