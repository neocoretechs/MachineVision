package com.neocoretechs.machinevision;

import java.util.Collection;
import java.util.List;
import java.awt.geom.Path2D;

public class PolyBezierPathUtil {
	 /**
	   * Computes a Poly-Bezier curve passing through a given list of knots.
	   * The curve will be twice-differentiable everywhere and satisfy natural
	   * boundary conditions at both ends.
	   *
	   * @param knots a list of knots
	   * @return      a Path representing the twice-differentiable curve
	   *              passing through all the given knots
	   */
	  public Path2D.Double computePathThroughKnots(List<EPointF> knots) {
	    throwExceptionIfInputIsInvalid(knots);
	    final Path2D.Double polyBezierPath = new Path2D.Double();
	    final EPointF firstKnot = knots.get(0);
	    polyBezierPath.moveTo(firstKnot.getX(), firstKnot.getY());
	    /*
	     * variable representing the number of Bezier curves we will join
	     * together
	     */
	    final int n = knots.size() - 1;

	    if (n == 1) {
	      final EPointF lastKnot = knots.get(1);
	      polyBezierPath.lineTo(lastKnot.getX(), lastKnot.getY());
	    } else {
	      final EPointF[] controlPoints = computeControlPoints(n, knots);
	      for (int i = 0; i < n; i++) {
	        final EPointF targetKnot = knots.get(i + 1);
	        appendCurveToPath(polyBezierPath, controlPoints[i], controlPoints[n + i], targetKnot);
	      }
	    }
	    return polyBezierPath;
	  }

	  private EPointF[] computeControlPoints(int n, List<EPointF> knots) {
	    final EPointF[] result = new EPointF[2 * n];
	    final EPointF[] target = constructTargetVector(n, knots);
	    final double[] lowerDiag = constructLowerDiagonalVector(n - 1);
	    final double[] mainDiag = constructMainDiagonalVector(n);
	    final double[] upperDiag = constructUpperDiagonalVector(n - 1);

	    final EPointF[] newTarget = new EPointF[n];
	    final double[] newUpperDiag = new double[n - 1];

	    // forward sweep for control points c_i,0:
	    newUpperDiag[0] = upperDiag[0] / mainDiag[0];
	    newTarget[0] = target[0].scaleBy(1 / mainDiag[0]);
	    for (int i = 1; i < n - 1; i++) {
	      newUpperDiag[i] = upperDiag[i] /
	          (mainDiag[i] - lowerDiag[i - 1] * newUpperDiag[i - 1]);
	    }
	    for (int i = 1; i < n; i++) {
	      final double targetScale = 1 /
	          (mainDiag[i] - lowerDiag[i - 1] * newUpperDiag[i - 1]);
	      newTarget[i] =
	          (target[i].minus(newTarget[i - 1].scaleBy(lowerDiag[i - 1]))).scaleBy(targetScale);
	    }
	    // backward sweep for control points c_i,0:
	    result[n - 1] = newTarget[n - 1];
	    for (int i = n - 2; i >= 0; i--) {
	      result[i] = newTarget[i].minus(newUpperDiag[i], result[i + 1]);
	    }
	    // calculate remaining control points c_i,1 directly:
	    for (int i = 0; i < n - 1; i++) {
	      result[n + i] = knots.get(i + 1).scaleBy(2).minus(result[i + 1]);
	    }
	    result[2 * n - 1] = knots.get(n).plus(result[n - 1]).scaleBy(0.5f);
	    return result;
	  }

	  private EPointF[] constructTargetVector(int n, List<EPointF> knots) {
	    final EPointF[] result = new EPointF[n];
	    result[0] = knots.get(0).plus(2, knots.get(1));
	    for (int i = 1; i < n - 1; i++) {
	      result[i] = (knots.get(i).scaleBy(2).plus(knots.get(i + 1))).scaleBy(2);
	    }
	    result[result.length - 1] = knots.get(n - 1).scaleBy(8).plus(knots.get(n));
	    return result;
	  }

	  private double[] constructLowerDiagonalVector(int length) {
	    final double[] result = new double[length];

	    for (int i = 0; i < result.length - 1; i++) {
	      result[i] = 1;
	    }
	    result[result.length - 1] = 2;
	    return result;
	  }

	  private double[] constructMainDiagonalVector(int n) {
	    final double[] result = new double[n];
	    result[0] = 2;
	    for (int i = 1; i < result.length - 1; i++) {
	      result[i] = 4;
	    }
	    result[result.length - 1] = 7;
	    return result;
	  }

	  private double[] constructUpperDiagonalVector(int length) {
	    final double[] result = new double[length];
	    for (int i = 0; i < result.length; i++) {
	      result[i] = 1;
	    }
	    return result;
	  }

	  private void appendCurveToPath(Path2D.Double path, EPointF control1, EPointF control2, EPointF targetKnot) {
	    path.curveTo(
	        control1.getX(),
	        control1.getY(),
	        control2.getX(),
	        control2.getY(),
	        targetKnot.getX(),
	        targetKnot.getY()
	    );
	  }

	  private void throwExceptionIfInputIsInvalid(Collection<EPointF> knots) {
	    if (knots.size() < 2) {
	      throw new IllegalArgumentException(
	          "Collection must contain at least two knots"
	      );
	    }
	  }
	  /**
	   * API inspired by the Apache Commons Math Vector2D class.
	   */
	  public class EPointF {

	    private final double x;
	    private final double y;

	    public EPointF(final double d, final double e) {
	      this.x = d;
	      this.y = e;
	    }

	    public double getX() {
	      return x;
	    }

	    public double getY() {
	      return y;
	    }

	    public EPointF plus(double factor, EPointF ePointF) {
	      return new EPointF(x + factor * ePointF.x, y + factor * ePointF.y);
	    }

	    public EPointF plus(EPointF ePointF) {
	      return plus(1.0f, ePointF);
	    }

	    public EPointF minus(double factor, EPointF ePointF) {
	      return new EPointF(x - factor * ePointF.x, y - factor * ePointF.y);
	    }

	    public EPointF minus(EPointF ePointF) {
	      return minus(1.0f, ePointF);
	    }

	    public EPointF scaleBy(double factor) {
	      return new EPointF(factor * x, factor * y);
	    }

	  }
}

