package com.craigcode.climbing_simulator_refactored;

import java.awt.Point;
import java.util.HashSet;

class ClimbingHold extends Point {
	
	private static final long serialVersionUID = 1L;
	
	boolean isVisibleInIntroductionAnimation;
	
	boolean hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI;
	boolean hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII;
	boolean hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII;
	boolean hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV;

	boolean hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants;
			
	boolean needsAHoldWithinReachableRangeSomewhereAbove;
	boolean needsAHoldWithinReachableRangeSomewhereBelow;
	boolean needsAHoldWithinReachableRangeInIOrIII;
	boolean needsAHoldWithinReachableRangeInIIOrIV;
	
	boolean withinTopOfWallZone;
	boolean withinBottomOfWallZone;
	
	float shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections;
	float shortestDistanceToATopHoldViaQuadrantSizeLimitsOnConnections;
	
	int adjacentHoldThatBeginsShortestPathToBottomOfWallViaQuadrantSizeLimitsOnConnections;
	
	boolean connectedToTop;
	boolean connectedToBottom;
	
	ConnectionAnalysisState connectionAnalysisState;
	
	boolean hasHoldInQuadrantI;
	boolean hasHoldInQuadrantII;
	boolean hasHoldInQuadrantIII;
	boolean hasHoldInQuadrantIV;
	
	boolean isABorderHold;

	HashSet<Integer> holdsThatAreCloseEnoughToPossiblyBeInAQuadrant;
	
	HashSet<LimbIdentifier> limbsThatHaveUsedHoldDuringAscent;
	HashSet<LimbIdentifier> limbsThatHaveUsedHoldDuringBacktrackingSequence;
	
	float opacityPercentage;
						
	ClimbingHold (int x, int y) {
		
		this.x = x;
		this.y = y;
		
		isVisibleInIntroductionAnimation = false;
		
		holdsThatAreCloseEnoughToPossiblyBeInAQuadrant = new HashSet<Integer>();
		
		hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = true;
		
		needsAHoldWithinReachableRangeSomewhereAbove = false;
		needsAHoldWithinReachableRangeSomewhereBelow = false;
		needsAHoldWithinReachableRangeInIOrIII = false;
		needsAHoldWithinReachableRangeInIIOrIV = false;
		
		connectedToTop = false;
		connectedToBottom = false;
		
		adjacentHoldThatBeginsShortestPathToBottomOfWallViaQuadrantSizeLimitsOnConnections = -1;
		
		limbsThatHaveUsedHoldDuringAscent = new HashSet<>();
		limbsThatHaveUsedHoldDuringBacktrackingSequence = new HashSet<>();
	}
}