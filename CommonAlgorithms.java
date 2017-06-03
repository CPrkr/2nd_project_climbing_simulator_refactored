package com.craigcode.climbing_simulator_refactored;

import java.awt.Point;
import java.awt.geom.Arc2D;
import java.awt.geom.Area;
import java.awt.geom.Rectangle2D;

class CommonAlgorithms {
	
	static float Pi = constants.Pi;
	
	//I. BASIC CALCULATIONS
	
	static float findSquare (double a) {
		
		return (float) Math.pow(a, 2);
	}
	
	static Point roundPoint(float x, float y) {
		
		return new Point(Math.round(x),Math.round(y));
	}
	
	//II. METHODS TO DETERMINE POSITIONING INFORMATION
	
	static Point averageThePointsInAnArea(ClimbingWall.SetOfHolds<ClimbingWall.ClimbingHold> setOfHolds, Area containingArea) {
		
		int holdsInArea = 0;
		int totalX = 0;
		int totalY = 0;
		
		for (ClimbingWall.ClimbingHold h : setOfHolds) {
			
			if (containingArea.contains(h)) {
				
				totalX += h.x;
				totalY += h.y;
				holdsInArea++;
			}
		}
		
		if (holdsInArea > 0) {
			
			float averageX = totalX/holdsInArea;
			float averageY = totalY/holdsInArea;
			
			Point result = roundPoint(averageX, averageY);
			return result;
			
		} else {
			
			return null;
		}
	}
	
	static float findDistance(Point a,Point b) {
		
		return (float) Math.sqrt(findSquare(a.x - b.x)+findSquare(a.y - b.y));
	}
	
	static Point findMidpoint (Point a, Point b) {
		
		return roundPoint(((a.x+b.x)/2),((a.y+b.y)/2)); 
	}
	
	//calculate "slope" (change in y over change in x on a standard x,y graph) as a measure of radians relative to horizon
	//values range from -pi radians (-180 degrees) to pi radians (180 degrees) 
	static float findSlopeAsRadians (Point start, Point end) {
		
		float changeInYs = (float) (end.y-start.y);
		
		//adjust for orientation of y-axis in swing compared to trigonometric functions
		changeInYs = (-1.0f) * changeInYs;
		
		float changeInXs = (float) (end.x - start.x);
		
		float slopeAsRadians = (float) Math.atan2(changeInYs, changeInXs);
		
		return slopeAsRadians;
	}
	
	//determine angle between sides a and b of triangle with sides a, b, and c
	//side c is the side opposite the angle being calculated
	//values range from 0 radians (0 degrees) to pi radians (180 degrees)
	static float lawOfCosines (double a, double b, double c) {
		
		return (float) Math.acos((CommonAlgorithms.findSquare(a) + CommonAlgorithms.findSquare(b)-CommonAlgorithms.findSquare(c))/(2*a*b));
	}
	
	//calculate new point created by moving a certain length from starting point, either along or against a "slope" that is measured as a radial angle relative to horizon
	static Point shiftBySlopeAsRadians(Point start, float length, float slopeAsRadians, boolean withRadialSlope) {
		
		//reverse radial slope 180 degrees if movement is in opposite direction of radial slope
		if (!withRadialSlope) {
			
			slopeAsRadians = Pi + slopeAsRadians;
			
		}
		
		double xShifted = start.x + (Math.cos(slopeAsRadians) * length);
		
		//minus sign is due to flipped y-axis compared to trigonometric functions
		double yShifted = start.y - (Math.sin(slopeAsRadians) * length);
		
		Point result = new Point(Math.toIntExact(Math.round(xShifted)),Math.toIntExact(Math.round(yShifted)));
		
		return result;
	}
	
	//determine point that is a particular distance away from a starting point along a ray that is the result of pivoting a starting ray through a certain measure of radians, either positive (counterclockwise) or negative (clockwise)
	static Point shiftPointByShiftedOrientation (Point startingPoint, float distanceShifted, float startingRay, float changeInRadiansInCounterclockwiseDirection) {
		
		float shiftedRay = CommonAlgorithms.pivotRay(startingRay, changeInRadiansInCounterclockwiseDirection, true);
		
		return shiftBySlopeAsRadians(startingPoint, distanceShifted, shiftedRay, true);
	}
	
	//take a radial orientation and reverse it 180 degrees by adding pi
	//returns value ranging from -pi to pi 
	static float calculateOppositeRadialOrientation (float radialOrientation) {
		
		float result = radialOrientation + Pi;
		
		//ensure that result is within allowable range of values
		if (result > Pi) {
			result = result - (2 * Pi);
		}
		
		return result;
	}
	
	//find absolute value of convex radial pivot between two rays
	//value returned will range from 0 radians (0 degrees) to pi radians (180 degrees) 
	static float calculateConvexAngleBetweenTwoRays (float rayA, float rayB) {
		
		float result = Math.abs(rayA - rayB);
		
		if (result > Pi) {
			result = (2 * Pi) - result;
		}
		
		return result;
	}
	
	//calculate radian measure that results from moving a ray through a certain number of radians, either clockwise or counterclockwise
	//value returned will be between -pi radians (-180 degrees) and pi radians (180 degrees)
	static float pivotRay(float startingRay, float deltaRadians, boolean counterclockwise) {
		
		float adjuster = 1.0f;
		
		if (!counterclockwise) {
			
			adjuster = adjuster*(-1.0f);
		}
		
		float result = startingRay + (deltaRadians*adjuster);
		
		if (result > Pi) {
			
			result = result - (2 *Pi);		
		}
		
		if (result < -Pi) {
			
			result = result + (2 * Pi);
		}
	
		return result;
	}
	
	static Point farthestReachablePointPossibleInARadialDirection(float radialDirection, Point startingPoint, float reachDistance) {
		
		return shiftBySlopeAsRadians(startingPoint, reachDistance, radialDirection, true);
	}
	
	static Area createQuadrantOfProperHoldSpacingBasedOnReachableRange(ClimbingWall.ClimbingHold h, Direction dir, float interQuadrantGapFormattedAsDegreesInFloat) {			
		
		float outerRadius = constants.MaximumDistanceForProperSpacingInQuadrant * constants.SizeFactor;
		float innerRadius = constants.MinimumDistanceForProperSpacingInQuadrant * constants.SizeFactor;
		
		float arcStartingDegrees = 0;
		float arcExtent = 90 - (2 * interQuadrantGapFormattedAsDegreesInFloat);
			
		switch(dir) {
		
			case UPANDRIGHT:
				arcStartingDegrees = interQuadrantGapFormattedAsDegreesInFloat;
				break;
				
			case UPANDLEFT:
				arcStartingDegrees = 90 + interQuadrantGapFormattedAsDegreesInFloat;
				break;
				
			case DOWNANDLEFT:
				arcStartingDegrees = 180 + interQuadrantGapFormattedAsDegreesInFloat;
				break;
				
			case DOWNANDRIGHT:
				arcStartingDegrees = 270 + interQuadrantGapFormattedAsDegreesInFloat;
				break;
		
			default:
				System.out.println("I'm being given a Direction that doesn't make sense as a quadrant");
		}
		
		Rectangle2D enclosingOuterSquare = new Rectangle2D.Float(h.x - outerRadius, h.y - outerRadius, 2 * outerRadius, 2 * outerRadius);
		Rectangle2D enclosingInnerSquare = new Rectangle2D.Float(h.x - innerRadius, h.y - innerRadius, 2 * innerRadius, 2 * innerRadius);
		
		Area outerArc = new Area(new Arc2D.Float(enclosingOuterSquare, arcStartingDegrees, arcExtent, 2));
		Area innerArc = new Area(new Arc2D.Float(enclosingInnerSquare, arcStartingDegrees, arcExtent, 2));
		
		outerArc.subtract(innerArc);
		
		return outerArc;
	}
	
	static Area createQuadrantForDeterminingBorderHolds(ClimbingWall.ClimbingHold h, Direction dir) {			
		
		float radius = constants.MaximumDistanceForProperSpacingInQuadrant * constants.SizeFactor;
		
		float arcStartingDegrees = 0;
		
		switch(dir) {
		
			case UPANDRIGHT:
				arcStartingDegrees = 0;
				break;
				
			case UPANDLEFT:
				arcStartingDegrees = 90;
				break;
				
			case DOWNANDLEFT:
				arcStartingDegrees = 180;
				break;
				
			case DOWNANDRIGHT:
				arcStartingDegrees = 270;
				break;
		
			default:
				System.out.println("I'm being given a Direction that doesn't make sense as a quadrant");
		}
		
		Rectangle2D enclosingSquare = new Rectangle2D.Float(h.x - radius, h.y - radius, 2 * radius, 2 * radius);
		
		Area result = new Area(new Arc2D.Float(enclosingSquare, arcStartingDegrees, 90, 2));
		
		return result;
	}	
}