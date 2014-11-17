package com.neocoretechs.machinevision.test;

import java.util.Iterator;

import com.neocoretechs.relatrix.Relatrix;

public class VisualCortex {
public static void main(String[] args) throws Exception {
	Relatrix.setTablespaceDirectory(args[0]);
	Iterator<?> iterator = Relatrix.findSet("?", "?", "?");
	int cnt = 0;
	while(iterator.hasNext()) {
		Comparable[] c = (Comparable[])iterator.next();
		for(int i = 0; i < c.length; i++) {
			System.out.println(cnt+" "+((Object)c[i]).toString());
		++cnt;
		}
	}
	System.out.println("Count: "+cnt);
}
}
