package com.craigcode.climbing_simulator_refactored;

import java.applet.Applet;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.Shape;
import java.awt.geom.Arc2D;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;

import javax.swing.SwingWorker;

import com.craigcode.climbing_simulator_refactored.Climber.Arm;
import com.craigcode.climbing_simulator_refactored.Climber.ClimbingMovesProcessor;
import com.craigcode.climbing_simulator_refactored.Climber.LeftArm;
import com.craigcode.climbing_simulator_refactored.Climber.LeftLeg;
import com.craigcode.climbing_simulator_refactored.Climber.Leg;
import com.craigcode.climbing_simulator_refactored.Climber.Limb;
import com.craigcode.climbing_simulator_refactored.Climber.LimbsCoordinator;
import com.craigcode.climbing_simulator_refactored.Climber.RightArm;
import com.craigcode.climbing_simulator_refactored.Climber.RightLeg;

class Climber extends Applet {
	
	private static final long serialVersionUID = 1L;
	
	private static float Pi = Constants.Pi;
	
	private boolean iAmOnTheWall;
	private boolean iHaveLeftGroundAndConnectedToWallWithAllFourLimbs;
	private boolean iAmBacktracking;

	private Point mostRecentFourConnectionsPoint;
	private int difficultMoveTracker;
	private boolean hasReachedDestinationPointThatWasBeyondSeeminglyImpossiblePosition;
	private boolean iAmTryingToClimb;
	
	private ClimbingWall.SetOfHolds<ClimbingHold> finalizedSetOfUserEnteredHolds;
	
	private Point anchorPoint;
	
	private LeftArm leftArm;
	private RightArm rightArm;
	private LeftLeg leftLeg;
	private RightLeg rightLeg;
	
	private LimbsCoordinator<Limb> limbsCoordinator;
	
	private LinkedList<Integer> shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections;
	
	private LinkedList<Point> pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections;

	private Point desiredDestinationForBeltBuckle;
	
	private float desiredRadialDirectionOfTravel;
	private Integer averagePathIndexIterator;
	
	private boolean hasReachedSeeminglyImpossiblePosition;
	private boolean hasReturnedSafely;
	private int indexOfAveragePathDestinationPointAtTheSeeminglyImpossiblePosition;
	
	private Area currentReachableAreaAggregate;
	
	//fundamental points for current climber positioning
	private Point torsoCOG = new Point();
	
	private Point leftShoulderSocketPos = new Point();
	private Point rightShoulderSocketPos = new Point();
	private Point headBasePos = new Point();
	private Point headTopPos = new Point();
	private Point neckBaseBetweenShoulderSocketsPos = new Point();
	private Point leftElbowPos = new Point();
	private Point rightElbowPos = new Point();
	private Point leftWristPos = new Point();
	private Point rightWristPos = new Point();
	private Point leftHandCenterPos = new Point();
	private Point rightHandCenterPos = new Point();
	
	private Point beltBucklePos = new Point();
	private Point pelvisCenterPos = new Point();
	private Point leftHipSocketPos = new Point();
	private Point rightHipSocketPos = new Point();
	
	private Point leftKneePos = new Point();
	private Point leftAnklePos = new Point();
	private Point rightKneePos = new Point();
	private Point rightAnklePos = new Point();
	private Point leftFootHoldContactPoint = new Point();
	private Point rightFootHoldContactPoint = new Point();
	
	private Area currentHipBox;
	private Area prospectiveHipBox;
	
	private Point leftUpperOuterHipBoxPos = new Point();
	private Point rightUpperOuterHipBoxPos = new Point();
	private Point prospectiveLeftUpperOuterHipBoxPos = new Point();
	private Point prospectiveRightUpperOuterHipBoxPos = new Point();
	
	private Point leftUnjammedFootWaypoint;
	private Point rightUnjammedFootWaypoint;
		
	//slopes of limb segments measured as rays relative to horizon with proximal joint as origin 
	private float leftHumerusOrientation;
	private float rightHumerusOrientation;
	private float leftThighOrientation;
	private float rightThighOrientation;
	private float leftForearmOrientation;
	private float rightForearmOrientation;
	private float leftCalfOrientation;
	private float rightCalfOrientation;
	private float leftWristPivot;
	private float rightWristPivot;

	//points and distances for testing prospective torso move
	private Point prospectiveLeftShoulderSocketPos = new Point();
	private Point prospectiveRightShoulderSocketPos = new Point();
	private Point prospectiveBeltBucklePos = new Point();
	private Point prospectiveTorsoCOG = new Point();
	private Point prospectivePelvisCenterPos = new Point();
	private Point prospectiveLeftHipSocketPos = new Point();
	private Point prospectiveRightHipSocketPos = new Point();
	
	//points related to painting the upper body
	//instantiated to (0,0) here because formulae work directly with x and y values, which therefore should not be null
	private Point leftWaistDrawingPoint = new Point();
	private Point rightWaistDrawingPoint = new Point();
	
	private Point leftLatTorsoInsertionPoint = new Point();
	private Point rightLatTorsoInsertionPoint = new Point();
	private Point leftTrapNeckIntersectionPoint = new Point();
	private Point rightTrapNeckIntersectionPoint = new Point();
	
	private Point lateralProximalLeftHumerus = new Point();
	private Point medialProximalLeftHumerus = new Point();
	private Point lateralDistalLeftHumerus = new Point();
	private Point medialDistalLeftHumerus = new Point();
	private Point lateralProximalRightHumerus = new Point();
	private Point medialProximalRightHumerus = new Point();
	private Point lateralDistalRightHumerus = new Point();
	private Point medialDistalRightHumerus = new Point();
	
	private Point lateralProximalLeftForearm = new Point();
	private Point medialProximalLeftForearm = new Point();
	private Point lateralDistalLeftForearm = new Point();
	private Point medialDistalLeftForearm = new Point();
	private Point lateralProximalRightForearm = new Point();
	private Point medialProximalRightForearm = new Point();
	private Point lateralDistalRightForearm = new Point();
	private Point medialDistalRightForearm = new Point();
	
	private Point leftShoulderDrawingPoint = new Point();
	private Point rightShoulderDrawingPoint = new Point();
	private Point leftElbowDrawingPoint = new Point();
	private Point rightElbowDrawingPoint = new Point();	
	private Point leftHandDrawingPoint = new Point();
	private Point rightHandDrawingPoint = new Point();	
	
	//points related to painting the lower body
	private Point lateralProximalLeftThigh = new Point();
	private Point medialProximalLeftThigh = new Point();
	private Point lateralDistalLeftThigh = new Point();
	private Point medialDistalLeftThigh = new Point();
	private Point lateralProximalRightThigh = new Point();
	private Point medialProximalRightThigh = new Point();
	private Point lateralDistalRightThigh = new Point();
	private Point medialDistalRightThigh = new Point();
	
	private Point lateralProximalLeftCalf = new Point();
	private Point medialProximalLeftCalf = new Point();
	private Point lateralDistalLeftCalf = new Point();
	private Point medialDistalLeftCalf = new Point();
	private Point lateralProximalRightCalf = new Point();
	private Point medialProximalRightCalf = new Point();
	private Point lateralDistalRightCalf = new Point();
	private Point medialDistalRightCalf = new Point();
		
	private Point inseam = new Point();
	
	private Point leftOuterPelvisHinge = new Point();
	private Point leftInnerPelvisHinge = new Point();
	private Point rightOuterPelvisHinge = new Point();
	private Point rightInnerPelvisHinge = new Point();
	
	private Point leftHipSocketDrawingPoint = new Point();
	private Point rightHipSocketDrawingPoint = new Point();
	private Point leftKneeDrawingPoint = new Point();
	private Point rightKneeDrawingPoint = new Point();	
	private Point leftFootDrawingPoint = new Point();
	private Point rightFootDrawingPoint = new Point();
	
	//points related to painting the head
	private Point headVisualCenter = new Point();
	private Point headDrawingPoint = new Point();
	private Point neckDrawingPoint = new Point();
	
	//body part dimensions
	private static float shoulderSocketSpan;
	private static float humerusLength;
	private static float forearmLength;
	private static float torsoHeightFromTopOfTrapsToWaist;
	private static float beltBuckleToCenterPelvisDistance;
	private static float hipSocketSpan;
	private static float femurLength;
	private static float calfLength;
	private static float footLengthHorizontalFromAnkleToToe;
	private static float footLength;
	private static float footHeight;
	private static float headRadius;
	private static float distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle;
	private static float neckThickness;
	private static float upperArmWidth;
	private static float elbowWidth;
	private static float wristWidth;
	private static float handRadius;
	private static float upperThighWidth;
	private static float kneeWidth;
	private static float ankleWidth;
	private static float waistToInseamCentralVerticalDistance;
	private static float hipToOuterPelvisCreaseVerticalDistance;
	private static float inseamToInnerPelvisCreaseHorizontalDistance;
	private static float fingerLength;
	private static float fingerWidth;
	private static float thumbLength;
	private static float thumbWidth;
	
	private float highestPossibleExtensionFromFootToHand;
	private float widestPossibleExtensionFromHandToHand;
	
	boolean wallHasDrawnTheLatestClimberMoves;
	
	Climber () {
		
		iAmOnTheWall = false;
		iHaveLeftGroundAndConnectedToWallWithAllFourLimbs = false;
		
		shoulderSocketSpan = Constants.ShoulderSocketSpan * Constants.SizeFactor;
		humerusLength = Constants.HumerusLength * Constants.SizeFactor;
		forearmLength = Constants.ForearmLength * Constants.SizeFactor;
		torsoHeightFromTopOfTrapsToWaist = Constants.TorsoHeightFromTopOfTrapsToWaist * Constants.SizeFactor;
		beltBuckleToCenterPelvisDistance = Constants.BeltBuckleToCenterOfPelvisBetweenHipSocketsVerticalDistance * Constants.SizeFactor;
		hipSocketSpan = Constants.HipSocketSpan * Constants.SizeFactor;
		femurLength = Constants.FemurLength * Constants.SizeFactor;
		calfLength = Constants.CalfLength * Constants.SizeFactor;
		footLengthHorizontalFromAnkleToToe = Constants.FootLengthFromAnkle * Constants.SizeFactor;
		footLength = Constants.FootLength * Constants.SizeFactor;
		footHeight = Constants.FootHeight * Constants.SizeFactor;
		headRadius = Constants.HeadRadius * Constants.SizeFactor;
		distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle = Constants.DistanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.SizeFactor;
		neckThickness = Constants.NeckThickness * Constants.SizeFactor;
		upperArmWidth = Constants.UpperArmWidth * Constants.SizeFactor;
		elbowWidth = Constants.ElbowWidth * Constants.SizeFactor;
		wristWidth = Constants.WristWidth * Constants.SizeFactor;
		handRadius = Constants.HandRadius * Constants.SizeFactor;		
		upperThighWidth = Constants.UpperThighWidth * Constants.SizeFactor;
		kneeWidth = Constants.KneeWidth * Constants.SizeFactor;
		ankleWidth = Constants.AnkleWidth * Constants.SizeFactor;
		waistToInseamCentralVerticalDistance = Constants.WaistToInseamCentralVerticalDistance * Constants.SizeFactor;
		hipToOuterPelvisCreaseVerticalDistance = Constants.HipToOuterPelvisCreaseVerticalDistance * Constants.SizeFactor;
		inseamToInnerPelvisCreaseHorizontalDistance = Constants.InseamToInnerPelvisCreaseHorizontalDistance * Constants.SizeFactor;
		fingerLength = Constants.MiddleFingerLength * Constants.SizeFactor;
		fingerWidth = Constants.FingerWidth * Constants.SizeFactor;
		thumbLength = Constants.ThumbLength * Constants.SizeFactor;
		thumbWidth = Constants.ThumbWidth * Constants.SizeFactor;
		
		highestPossibleExtensionFromFootToHand = footHeight + calfLength + femurLength + (torsoHeightFromTopOfTrapsToWaist - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead)) + humerusLength + forearmLength + handRadius;
		widestPossibleExtensionFromHandToHand = shoulderSocketSpan + (2 * (humerusLength + forearmLength + handRadius)); 
		
		leftArm = new LeftArm();
		rightArm = new RightArm();
		leftLeg = new LeftLeg();
		rightLeg = new RightLeg();
		
		limbsCoordinator = new LimbsCoordinator<>();

		shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections = new LinkedList<>();
		pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections = new LinkedList<>();
	}
	
	abstract class Limb {
		
		LimbIdentifier limbIdentifier;
		
		Area currentlyReachableArea;
		Area prospectiveMoveReachableArea;
		
		ArrayList<Point> allReachableHoldPoints;
				
		boolean isConnectedToSomething;
		boolean hasMadeFirstConnectionToTheWall;
		
		boolean limbHadToAbandonHoldBecauseOfAViolationOfReachableArea;
		boolean limbHasMovedThroughAnAreaWhereItHadNoOptions;
		
		float maximumReachLength;
		float maximumLimbExtensionLength;
		float overallExtensionRatio;
				
		abstract Point getWallConnectionBodyPartPoint(); //hand or foot		
		abstract void setWallConnectionBodyPartPoint(Point p);
		
		abstract Point getProximalStartPointForLimb(); //shoulder or pelvis socket
		abstract Point getProspectiveProximalStartPointForLimb();
		
		abstract Point getDistalEndpointOfDistalSegment(); //wrist or ankle	
		
		boolean isLeft() {
			return Left.class.isAssignableFrom(this.getClass());
		}
		
		boolean isArm() {
			return Arm.class.isAssignableFrom(this.getClass());
		}
		
		abstract float getProximalSegmentLength();
		abstract float getDistalSegmentLength();
		abstract float getReachDistanceContributedByExtremity();		
		
		void calculateLimbMaxEntensionValues() {
			
			maximumReachLength = getProximalSegmentLength() + getDistalSegmentLength() + getReachDistanceContributedByExtremity();
			
			maximumLimbExtensionLength = getProximalSegmentLength() + getDistalSegmentLength();
		}

		void updateOverallExtensionRatio() {
			
			float totalDistanceBetweenLimbEndpoints = CommonAlgorithms.findDistance(getProximalStartPointForLimb(), getDistalEndpointOfDistalSegment());
			overallExtensionRatio = totalDistanceBetweenLimbEndpoints/maximumLimbExtensionLength;
		}
		
		abstract void updateReachableAreaForCurrentPosition();
		
		abstract void updateProspectiveMoveReachableArea();
				
		boolean limbCanContinueToUseCurrentHoldIfTorsoMakesProspectiveMove() {
			
			updateProspectiveMoveReachableArea();
			
			return prospectiveMoveReachableArea.contains(getWallConnectionBodyPartPoint());
		}
		
		void updateReachableHoldPoints() {
		
			updateReachableAreaForCurrentPosition();
			
			ArrayList<Point> result = new ArrayList<Point>();
			
			for (Point p : finalizedSetOfUserEnteredHolds) {
				
				if (currentlyReachableArea.contains(p)) {
					
					result.add(p);
				}
			}
			
			allReachableHoldPoints = result;
		}
		
		void getSetToClimb() {
			
			updateReachableHoldPoints();
							
			if(allReachableHoldPoints.size() == 0) {
					
				if (isArm()) {
						
					moveToReadyPoint();
					return;
						
				} else {
						
					return; //leg remains on floor
				}
			}
			
			Point destinationHoldPoint = findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(desiredRadialDirectionOfTravel, allReachableHoldPoints);
			
			moveLimbToPointOverrideable(destinationHoldPoint, true);
			
			indexOfAveragePathDestinationPointAtTheSeeminglyImpossiblePosition = Integer.MAX_VALUE;
		}
		
		boolean receiveARequestToAdvanceLimbFromCurrentHoldToAReachableHoldThatIsMoreExtremeInDesiredDirectionOfTravelThanIsCurrentHoldAndConfirmIfAMoveWasMade() {
			
			if (limbCanBeAdvancedFromCurrentHoldToAReachableHoldThatIsMoreExtremeInDesiredDirectionOfTravel()) {
				
				advanceLimbAsMuchAsPossibleToAConfirmedHoldThatIsMoreExtremeInDesiredDirectionOfTravelThanIsTheCurrentHold();
				
				return true;
				
			} else {
				
				return false;
				
			}
		}
		
		boolean limbCanBeAdvancedFromCurrentHoldToAReachableHoldThatIsMoreExtremeInDesiredDirectionOfTravel() {
			
			updateReachableHoldPoints();
			
			Point reachableHoldThatIsMostExtremeInDesiredDirectionOfTravel = findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(desiredRadialDirectionOfTravel, allReachableHoldPoints);
			
			if ((getWallConnectionBodyPartPoint().x == reachableHoldThatIsMostExtremeInDesiredDirectionOfTravel.x) && (getWallConnectionBodyPartPoint().y == reachableHoldThatIsMostExtremeInDesiredDirectionOfTravel.y)) {
				
				return false;
				
			} else {
				
				return true;
				
			}
		}
		
		void advanceLimbAsMuchAsPossibleToAConfirmedHoldThatIsMoreExtremeInDesiredDirectionOfTravelThanIsTheCurrentHold() {
			
			updateReachableHoldPoints();
			
			Point reachableHoldThatIsMostExtremeAndMoreExtremeThanCurrentHoldInDesiredDirectionOfTravel = findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(desiredRadialDirectionOfTravel, allReachableHoldPoints);
			
			moveLimbToPointOverrideable(reachableHoldThatIsMostExtremeAndMoreExtremeThanCurrentHoldInDesiredDirectionOfTravel, true);
		}
		
		boolean receiveARequestToMoveLimbToTheHoldThatIsFurthestInDesiredDirectionOfTravelAmongReachableHoldsOtherThanHoldsThatHaveBeenUsedDuringEitherAscentOrBacktrackingAndConfirmIfAMoveWasMade() {
			
			if (limbHasAHoldAvailableOtherThanHoldsThatHaveBeenUsedDuringEitherAscentOrBacktracking()) {
				
				ArrayList<Point> unusedPoints = new ArrayList<>();
				
				if (!iAmBacktracking) {
					
					for (Point p : allReachableHoldPoints) {
						
						int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
						
						if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringAscent.contains(limbIdentifier)) {
							
							unusedPoints.add(p);
							
						}
					}
					
				} else {
				
					for (Point p : allReachableHoldPoints) {
						
						int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
						
						if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringBacktrackingSequence.contains(limbIdentifier)) {
							
							unusedPoints.add(p);
							
						}
					}
				}
				
				if (unusedPoints.size() == 0) {
					
					return false;
					
				}
				
				Point destinationHoldPoint = findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(desiredRadialDirectionOfTravel, unusedPoints);
				
				moveLimbToPointOverrideable(destinationHoldPoint, true);
				
				return true;
				
			} else {
				
				return false;
				
			}	
		}
		
		boolean limbHasAHoldAvailableOtherThanHoldsThatHaveBeenUsedDuringEitherAscentOrBacktracking() {
			
			updateReachableHoldPoints();
					
			ArrayList<Point> filteredPoints = new ArrayList<>();
						
			if (!iAmBacktracking) {
					
				for (Point p : allReachableHoldPoints) {
					
					int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
					
					if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringAscent.contains(limbIdentifier)) {
						
						filteredPoints.add(p);
						
					}
				}
				
			} else {
			
				for (Point p : allReachableHoldPoints) {
					
					int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
					
					if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringBacktrackingSequence.contains(limbIdentifier)) {
						
						filteredPoints.add(p);
						
					}
				}
			}
				
			if (filteredPoints.size() > 0) {
				
				return true;
				
			} else {
				
				return false;
				
			}
		}		
		
		void moveLimbToTheReachableHoldThatIsFurthestInDesiredDirectionOfTravelAmongAllReachableHolds() {
			
			updateReachableHoldPoints();
			
			Point reachableHoldThatIsMostExtremeInDesiredDirectionOfTravel = findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(desiredRadialDirectionOfTravel, allReachableHoldPoints);
			
			moveLimbToPointOverrideable(reachableHoldThatIsMostExtremeInDesiredDirectionOfTravel, true);
		}

		Point findPointThatIsMostExtremeInRadialDirectionWithinSuppliedSetOfPoints(float radialDirection, ArrayList<Point> setOfPoints) {
			
			Point result = null;
			
			Point targetPoint = CommonAlgorithms.farthestReachablePointPossibleInARadialDirection(radialDirection, getProximalStartPointForLimb(), maximumReachLength);
			
			float distanceOfClosestHoldThusfar = 2 * Float.MAX_VALUE;
			
			for (Point p: setOfPoints) {
				
				float interholdDistance = CommonAlgorithms.findDistance(p, targetPoint);
				
				if (interholdDistance < distanceOfClosestHoldThusfar) {
					distanceOfClosestHoldThusfar = interholdDistance;
					result = p;
				}
			}
			
			return result;
		}		
		
		void moveLimbToPointOverrideable(Point destinationPoint, boolean connectToWallAfterMakingMove) {
			
			if (isConnectedToSomething) {
				disconnectFromWall();
			}
			
			ArrayList<float[]> waypointsFormattedAsFloat = calculateWaypointFloatCoordinates(getWallConnectionBodyPartPoint(), destinationPoint, Constants.LimbMovementIncrement * Constants.SizeFactor);
			
			Point waypoint = getWallConnectionBodyPartPoint();
					
			for (float[] waypointCoordinatesInFloat : waypointsFormattedAsFloat) {
				waypoint.x = Math.round(waypointCoordinatesInFloat[0]);
				waypoint.y = Math.round(waypointCoordinatesInFloat[1]);
	
				setWallConnectionBodyPartPoint(waypoint);
				
				updateLimbPoints();
				pauseForAnimationTiming();
			}
			
			if (connectToWallAfterMakingMove) {
				connectToWall(destinationPoint);
			}
		}
		
		void moveLimbToPointBasic(Point destinationPoint, boolean connectToWallAfterMakingMove) {
			
			if (isConnectedToSomething) {
				disconnectFromWall();
			}
			
			ArrayList<float[]> waypointsFormattedAsFloat = calculateWaypointFloatCoordinates(getWallConnectionBodyPartPoint(), destinationPoint, Constants.LimbMovementIncrement * Constants.SizeFactor);
			
			Point waypoint = getWallConnectionBodyPartPoint();
					
			for (float[] waypointCoordinatesInFloat : waypointsFormattedAsFloat) {
				waypoint.x = Math.round(waypointCoordinatesInFloat[0]);
				waypoint.y = Math.round(waypointCoordinatesInFloat[1]);
	
				setWallConnectionBodyPartPoint(waypoint);
				
				updateLimbPoints();
				pauseForAnimationTiming();
			}
			
			if (connectToWallAfterMakingMove) {
				connectToWall(destinationPoint);
			}
		}
		
		abstract boolean wouldMoveGoThroughAreaThatJamsLimb(Point prospectiveDestinationPoint);
		abstract void moveLimbToUnjammedWaypointFirst(Point ultimateDestinationPoint, boolean connectToWallAfterMakingMove);
		
		abstract void moveToReadyPoint();
	
		abstract void updateLimbPoints();
		
		void disconnectFromWall() {
			
			isConnectedToSomething = false;
		}
			
		void connectToWall(Point connectionPoint) {
			
			hasMadeFirstConnectionToTheWall = true;
			
			limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = false;
			limbHasMovedThroughAnAreaWhereItHadNoOptions = false;
			
			isConnectedToSomething = true;
			
			updateSetOfLimbIdentifiersForHoldThatClimberJustConnectedTo(connectionPoint);
		}
		
		void updateSetOfLimbIdentifiersForHoldThatClimberJustConnectedTo(Point connectionPoint) {
		
			int indexOfHoldThatLimbIsCurrentlyUsingInFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(connectionPoint);
			
			if (!iAmBacktracking) {
				
				finalizedSetOfUserEnteredHolds.get(indexOfHoldThatLimbIsCurrentlyUsingInFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringAscent.add(limbIdentifier);
				
			} else {
				
				finalizedSetOfUserEnteredHolds.get(indexOfHoldThatLimbIsCurrentlyUsingInFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringBacktrackingSequence.add(limbIdentifier);
				
			}
		}
	}
	
	abstract class Arm extends Limb {
		
		@Override
		float getProximalSegmentLength() {
			return humerusLength;
		}

		@Override
		float getDistalSegmentLength() {
			return forearmLength;
		}

		@Override
		float getReachDistanceContributedByExtremity() {
			return handRadius;
		}
		
		@Override
		void moveToReadyPoint() {
			
			if(isLeft()) {

				moveLimbToPointOverrideable(CommonAlgorithms.roundPoint((getProximalStartPointForLimb().x - (1 * Constants.SizeFactor)), (getProximalStartPointForLimb().y + (55 * Constants.SizeFactor))), false);
				
			} else {

				moveLimbToPointOverrideable(CommonAlgorithms.roundPoint((getProximalStartPointForLimb().x + (1 * Constants.SizeFactor)), (getProximalStartPointForLimb().y + (55 * Constants.SizeFactor))), false);
			}
		}
		
		void updateReachableAreaForCurrentPosition() {
			
			currentlyReachableArea = calculateReachableAreaForArm(getProximalStartPointForLimb());
		}
	
		void updateProspectiveMoveReachableArea() {
			
			prospectiveMoveReachableArea = calculateReachableAreaForArm(getProspectiveProximalStartPointForLimb());
		}
		
		Area calculateReachableAreaForArm(Point shoulderSocket) {
			
			Area result;

			Shape armCircle = new Ellipse2D.Float(shoulderSocket.x - maximumReachLength, shoulderSocket.y - maximumReachLength, 2 * maximumReachLength, 2 * maximumReachLength);
			result = new Area(armCircle);
	
			Shape otherSideOfBody;
			
			if (isLeft()) {
				otherSideOfBody = new Rectangle2D.Float(shoulderSocket.x + (Constants.ShoulderSocketSpan * Constants.SizeFactor), shoulderSocket.y - maximumReachLength, maximumReachLength, 2 * maximumReachLength);
			} else {
				otherSideOfBody = new Rectangle2D.Float(shoulderSocket.x - ((Constants.ShoulderSocketSpan * Constants.SizeFactor) + maximumReachLength), shoulderSocket.y - maximumReachLength, maximumReachLength, 2 * maximumReachLength);
			}
	
			Area areaToSubtract = new Area(otherSideOfBody);
			
			result.subtract(areaToSubtract);
			
			return result;
		}
		
		boolean wouldMoveGoThroughAreaThatJamsLimb(Point prospectiveDestinationPoint) {
			
			return false;
		}
		
		void moveLimbToUnjammedWaypointFirst(Point ultimateDestinationPoint, boolean connectToWallAfterMakingMove) {
			
		}
	}
	
	abstract class Leg extends Limb {
		
		@Override
		float getProximalSegmentLength() {
			return femurLength;
		}

		@Override
		float getDistalSegmentLength() {
			return calfLength;
		}

		@Override
		float getReachDistanceContributedByExtremity() {
			return Math.min(footHeight + ((Constants.HoldDiameter/2) * Constants.SizeFactor), (footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)); //the second part of this equation could be a bit longer if it took into account the distance between ankle and foot-hold contact point, but reaching that far tends to trigger cramps in the author's feet
		}
		
		@Override
		void moveToReadyPoint() {

			if(isLeft()) {
				
				moveLimbToPointOverrideable(CommonAlgorithms.roundPoint((getProximalStartPointForLimb().x - (5 * Constants.SizeFactor)), (getProximalStartPointForLimb().y + (83 * Constants.SizeFactor))), false);

			} else {
				
				moveLimbToPointOverrideable(CommonAlgorithms.roundPoint((getProximalStartPointForLimb().x + (5 * Constants.SizeFactor)), (getProximalStartPointForLimb().y + (83 * Constants.SizeFactor))), false);
			}
		}
		
		void updateReachableAreaForCurrentPosition() {
			
			updateCurrentHipBox();
			
			currentlyReachableArea = calculateReachableAreaForLeg(getProximalStartPointForLimb(), torsoCOG, pelvisCenterPos, currentHipBox, getCurrentUpperOuterHipBoxPoint());
		}
		
		void updateProspectiveMoveReachableArea() {
			
			updateProspectiveHipBox();
			
			prospectiveMoveReachableArea = calculateReachableAreaForLeg(getProspectiveProximalStartPointForLimb(), prospectiveTorsoCOG, prospectivePelvisCenterPos, prospectiveHipBox, getProspectiveUpperOuterHipBoxPoint());
		}
		
		Area calculateReachableAreaForLeg (Point hipSocket, Point torsoCOG, Point pelvisCenter, Area relevantHipBox, Point upperOuterHipBoxPoint) {
			
			Area result;

			Shape legCircle = new Ellipse2D.Float(hipSocket.x - maximumReachLength, hipSocket.y - maximumReachLength, 2 * maximumReachLength, 2 * maximumReachLength);
			result = new Area(legCircle);
			
			Area totalAreaToSubtract;
			
			Area areaTooHighForInflexibleClimber;
			Area areaOfLegReachArchThatRisesDistallyAwayFromUpperOuterHipBoxPoint;
			Area areaOnOtherSideOfBody;
			Area areaOfLegReachArchThatRunsDistallyAwayFromPelvisCenter;
			
			Shape tooHighForInflexibleClimber;
			Shape legReachArcThatRisesAwayFromTorso;
			Shape otherSideOfBody;
			Shape legReachArcThatRunsAwayFromPelvisCenter;
			
			tooHighForInflexibleClimber = new Rectangle2D.Float(torsoCOG.x - (maximumReachLength + footLength), torsoCOG.y - (maximumReachLength + ((Constants.HoldDiameter/2) * Constants.SizeFactor)), 2 * (maximumReachLength + footLength), maximumReachLength + ((Constants.HoldDiameter/2) * Constants.SizeFactor));
						
			if (isLeft()) {
				legReachArcThatRisesAwayFromTorso = new Arc2D.Float(upperOuterHipBoxPoint.x - (maximumReachLength + footLength), upperOuterHipBoxPoint.y - (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), (180.0f - Constants.DegreesOfLegReachHeightGainedAsFootMovesAwayFromUpperOuterHipBoxPoint), Constants.DegreesOfLegReachHeightGainedAsFootMovesAwayFromUpperOuterHipBoxPoint, Arc2D.PIE);
			} else {
				legReachArcThatRisesAwayFromTorso = new Arc2D.Float(upperOuterHipBoxPoint.x - (maximumReachLength + footLength), upperOuterHipBoxPoint.y - (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 0, Constants.DegreesOfLegReachHeightGainedAsFootMovesAwayFromUpperOuterHipBoxPoint, Arc2D.PIE);
			}
								
			if (isLeft()) {
				otherSideOfBody = new Rectangle2D.Float(pelvisCenter.x + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), pelvisCenter.y - maximumReachLength, maximumReachLength + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), 2 * (maximumReachLength + ((Constants.HoldDiameter/2) * Constants.SizeFactor)));
			} else {
				otherSideOfBody = new Rectangle2D.Float(pelvisCenter.x - maximumReachLength, pelvisCenter.y - maximumReachLength, maximumReachLength - ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), 2 * (maximumReachLength + ((Constants.HoldDiameter/2) * Constants.SizeFactor)));
			}
			
			if (isLeft()) {
				legReachArcThatRunsAwayFromPelvisCenter = new Arc2D.Float((pelvisCenter.x + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe))) - (maximumReachLength + footLength), pelvisCenter.y - (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 270.0f, Constants.DegreesOfLegReachWidthGainedAsFootMovesAwayFromInseam, Arc2D.PIE);
			} else {
				legReachArcThatRunsAwayFromPelvisCenter = new Arc2D.Float((pelvisCenter.x - ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe))) - (maximumReachLength + footLength), pelvisCenter.y - (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 2 * (maximumReachLength + footLength), 270.0f - Constants.DegreesOfLegReachWidthGainedAsFootMovesAwayFromInseam, Constants.DegreesOfLegReachWidthGainedAsFootMovesAwayFromInseam, Arc2D.PIE);
			}
									
			areaTooHighForInflexibleClimber = new Area(tooHighForInflexibleClimber);
			areaOfLegReachArchThatRisesDistallyAwayFromUpperOuterHipBoxPoint = new Area(legReachArcThatRisesAwayFromTorso);
			areaTooHighForInflexibleClimber.subtract(areaOfLegReachArchThatRisesDistallyAwayFromUpperOuterHipBoxPoint);
			
			areaOnOtherSideOfBody = new Area(otherSideOfBody);
			areaOfLegReachArchThatRunsDistallyAwayFromPelvisCenter = new Area(legReachArcThatRunsAwayFromPelvisCenter);
			areaOnOtherSideOfBody.subtract(areaOfLegReachArchThatRunsDistallyAwayFromPelvisCenter);
			
			totalAreaToSubtract = new Area();
						
			totalAreaToSubtract.add(areaOnOtherSideOfBody);
			totalAreaToSubtract.add(areaTooHighForInflexibleClimber);
			totalAreaToSubtract.add(relevantHipBox);
			
			result.subtract(totalAreaToSubtract);
			
			return result;
		}
		
		@Override
		void moveLimbToPointOverrideable(Point p, boolean connectToWallAfterMakingMove) {
			
			if (wouldMoveGoThroughAreaThatJamsLimb(p)) {
				
				moveLimbToUnjammedWaypointFirst(p, connectToWallAfterMakingMove);
				
				return;
			}
			
			moveLimbToPointBasic(p, connectToWallAfterMakingMove);
		}
		
		abstract Point getCurrentUpperOuterHipBoxPoint();
		abstract Point getProspectiveUpperOuterHipBoxPoint();		
		
		abstract boolean wouldMoveGoThroughAreaThatJamsLimb(Point prospectiveDestinationPoint);
		
		abstract void moveLimbToUnjammedWaypointFirst(Point ultimateDestinationPoint, boolean connectToWallAfterMakingMove);
	}
	
	class LeftArm extends Arm implements Left {
		
		LeftArm() {
		
			limbIdentifier = LimbIdentifier.LEFTARM;
			
			calculateLimbMaxEntensionValues();
									
			isConnectedToSomething = true;
		}

		@Override
		Point getWallConnectionBodyPartPoint() {
			return leftHandCenterPos;
		}

		@Override
		void setWallConnectionBodyPartPoint(Point p) {
			leftHandCenterPos = p;
		}

		@Override
		Point getProximalStartPointForLimb() {
			return leftShoulderSocketPos;
		}

		@Override
		Point getProspectiveProximalStartPointForLimb() {
			return prospectiveLeftShoulderSocketPos;
		}
		
		@Override
		Point getDistalEndpointOfDistalSegment() {
			return leftWristPos;
		}

		@Override
		void updateLimbPoints() {
			
			try {
				updateLeftArmWireframeValues();
			} catch (Exception e) {
				System.out.println("I'm having trouble moving my left arm only");
			}
				
			updateLeftArmDrawingPoints();
		}
	}
	
	class RightArm extends Arm implements Right {
		
		RightArm() {
			
			limbIdentifier = LimbIdentifier.RIGHTARM;
			
			calculateLimbMaxEntensionValues();
			
			isConnectedToSomething = true;
		}

		@Override
		Point getWallConnectionBodyPartPoint() {
			return rightHandCenterPos;
		}

		@Override
		void setWallConnectionBodyPartPoint(Point p) {
			rightHandCenterPos = p;
		}

		@Override
		Point getProximalStartPointForLimb() {
			return rightShoulderSocketPos;
		}

		@Override
		Point getProspectiveProximalStartPointForLimb() {
			return prospectiveRightShoulderSocketPos;
		}
		
		@Override
		Point getDistalEndpointOfDistalSegment() {
			return rightWristPos;
		}

		@Override
		void updateLimbPoints() {
			
			try {
				updateRightArmWireframeValues();
			} catch (Exception e) {
				System.out.println("I'm having trouble moving my right arm only");
			}
				
			updateRightArmDrawingPoints();
		}
	}
	
	class LeftLeg extends Leg implements Left {
		
		LeftLeg() {
			
			limbIdentifier = LimbIdentifier.LEFTLEG;
			
			calculateLimbMaxEntensionValues();
		}

		//shifted because foot stands on top of hold, whereas hand grasps hold
		@Override
		Point getWallConnectionBodyPartPoint() {
			
			Point shiftFromBallOfFootToHoldCenter = new Point();
			
			shiftFromBallOfFootToHoldCenter.x = leftFootHoldContactPoint.x;
			shiftFromBallOfFootToHoldCenter.y = Math.round(leftFootHoldContactPoint.y + (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1));
			
			return shiftFromBallOfFootToHoldCenter;
		}

		//shifted because foot stands on top of hold, whereas hand grasps hold
		@Override
		void setWallConnectionBodyPartPoint(Point p) {
			
			Point shiftFromHoldCenterToBallOfFoot = new Point();
			
			shiftFromHoldCenterToBallOfFoot.x = p.x;
			shiftFromHoldCenterToBallOfFoot.y = Math.round(p.y - (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1));
			
			leftFootHoldContactPoint = shiftFromHoldCenterToBallOfFoot;
		}

		@Override
		Point getProximalStartPointForLimb() {
			return leftHipSocketPos;
		}

		@Override
		Point getProspectiveProximalStartPointForLimb() {
			return prospectiveLeftHipSocketPos;
		}
		
		@Override
		Point getDistalEndpointOfDistalSegment() {
			return leftAnklePos;
		}

		@Override
		void updateLimbPoints() {
			
			try {
				updateLeftLegWireframeValues();
			} catch (Exception e) {
				System.out.println("I'm having trouble moving my left leg only");
			}
			
			updateLeftLegDrawingPoints();
		}
		
		//constructs a triangle using the prospective route as a side that faces the torso, then tests whether unjammed point lies within the triangle
		boolean wouldMoveGoThroughAreaThatJamsLimb(Point prospectiveDestinationPoint) {
			
			updateCurrentHipBox();
			updateLeftUnjammedFootWaypoint();
			
			float distanceFromBottomOfFootToHoldCenter = (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1);
			
			Point prospectiveFootHoldContactPoint = CommonAlgorithms.roundPoint(prospectiveDestinationPoint.x, prospectiveDestinationPoint.y - distanceFromBottomOfFootToHoldCenter);
			
			int otherTrianglePointX = Math.min(leftFootHoldContactPoint.x, prospectiveFootHoldContactPoint.x);
			int otherTrianglePointY = Math.max(leftFootHoldContactPoint.y, prospectiveFootHoldContactPoint.y);
			
			Point otherTrianglePoint = new Point(otherTrianglePointX - 1, otherTrianglePointY + 1);
			
			Path2D.Float testingTrianglePerimeter = new Path2D.Float();
			testingTrianglePerimeter.moveTo(otherTrianglePoint.x, otherTrianglePoint.y);
			testingTrianglePerimeter.lineTo(leftFootHoldContactPoint.x, leftFootHoldContactPoint.y);
			testingTrianglePerimeter.lineTo(prospectiveFootHoldContactPoint.x, prospectiveFootHoldContactPoint.y);
			testingTrianglePerimeter.closePath();
			
			Area testingTriangleArea = new Area(testingTrianglePerimeter);
			
			return testingTriangleArea.contains(leftUnjammedFootWaypoint);
		}
		
		Point getCurrentUpperOuterHipBoxPoint() {
			
			return leftUpperOuterHipBoxPos;
					
		}
		
		Point getProspectiveUpperOuterHipBoxPoint() {
			
			return prospectiveLeftUpperOuterHipBoxPos;
					
		}
		
		void updateLeftUnjammedFootWaypoint() {
		
			leftUnjammedFootWaypoint = CommonAlgorithms.roundPoint((leftShoulderSocketPos.x - ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe))) - 1, (beltBucklePos.y + (waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor))) + 1);
		}
		
		void moveLimbToUnjammedWaypointFirst(Point ultimateDestinationPoint, boolean connectToWallAfterMakingMove) {
			
			moveLimbToPointBasic(leftUnjammedFootWaypoint, false);
			moveLimbToPointBasic(ultimateDestinationPoint, connectToWallAfterMakingMove);
		}
	}
		
	class RightLeg extends Leg implements Right {
		
		RightLeg() {
			
			limbIdentifier = LimbIdentifier.RIGHTLEG;
			
			calculateLimbMaxEntensionValues();
		}
		
		//shifted because foot stands on top of hold, whereas hand grasps hold
		@Override
		Point getWallConnectionBodyPartPoint() {
			
			Point shiftFromBallOfFootToHoldCenter = new Point();
			
			shiftFromBallOfFootToHoldCenter.x = rightFootHoldContactPoint.x;
			shiftFromBallOfFootToHoldCenter.y = Math.round(rightFootHoldContactPoint.y + (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1));
			
			return shiftFromBallOfFootToHoldCenter;
		}

		//shifted because foot stands on top of hold, whereas hand grasps hold
		@Override
		void setWallConnectionBodyPartPoint(Point p) {
			
			Point shiftFromHoldCenterToBallOfFoot = new Point();
			
			shiftFromHoldCenterToBallOfFoot.x = p.x;
			shiftFromHoldCenterToBallOfFoot.y = Math.round(p.y - (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1));
			
			rightFootHoldContactPoint = shiftFromHoldCenterToBallOfFoot;
		}

		@Override
		Point getProximalStartPointForLimb() {
			return rightHipSocketPos;
		}

		@Override
		Point getProspectiveProximalStartPointForLimb() {
			return prospectiveRightHipSocketPos;
		}
		
		@Override
		Point getDistalEndpointOfDistalSegment() {
			return rightAnklePos;
		}

		@Override
		void updateLimbPoints() {
			
			try {
				updateRightLegWireframeValues();
			} catch (Exception e) {
				System.out.println("I'm having trouble moving my right leg only");
			}
			
			updateRightLegDrawingPoints();
		}
		
		//constructs a triangle using the prospective route as a side that faces the torso, then tests whether unjammed point lies within the triangle
		boolean wouldMoveGoThroughAreaThatJamsLimb(Point prospectiveDestinationPoint) {
			
			updateCurrentHipBox();
			updateRightUnjammedFootWaypoint();
			
			float distanceFromBottomOfFootToHoldCenter = (((Constants.HoldDiameter/2) * Constants.SizeFactor) - 1);
			
			Point prospectiveFootHoldContactPoint = CommonAlgorithms.roundPoint(prospectiveDestinationPoint.x, prospectiveDestinationPoint.y - distanceFromBottomOfFootToHoldCenter);
			
			int otherTrianglePointX = Math.max(rightFootHoldContactPoint.x, prospectiveFootHoldContactPoint.x);
			int otherTrianglePointY = Math.max(rightFootHoldContactPoint.y, prospectiveFootHoldContactPoint.y);
			
			Point otherTrianglePoint = new Point(otherTrianglePointX + 1, otherTrianglePointY + 1);
			
			Path2D.Float testingTrianglePerimeter = new Path2D.Float();
			testingTrianglePerimeter.moveTo(otherTrianglePoint.x, otherTrianglePoint.y);
			testingTrianglePerimeter.lineTo(rightFootHoldContactPoint.x, rightFootHoldContactPoint.y);
			testingTrianglePerimeter.lineTo(prospectiveFootHoldContactPoint.x, prospectiveFootHoldContactPoint.y);
			testingTrianglePerimeter.closePath();
			
			Area testingTriangleArea = new Area(testingTrianglePerimeter);

			return testingTriangleArea.contains(rightUnjammedFootWaypoint);
		}
		
		Point getCurrentUpperOuterHipBoxPoint() {
			
			return rightUpperOuterHipBoxPos;
					
		}
		
		Point getProspectiveUpperOuterHipBoxPoint() {
			
			return prospectiveRightUpperOuterHipBoxPos;
					
		}
		
		void updateRightUnjammedFootWaypoint() {
			
			rightUnjammedFootWaypoint = CommonAlgorithms.roundPoint((rightShoulderSocketPos.x + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe))) + 1, (beltBucklePos.y + (waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor))) + 1);
		}
		
		void moveLimbToUnjammedWaypointFirst(Point ultimateDestinationPoint, boolean connectToWallAfterMakingMove) {
			
			moveLimbToPointBasic(rightUnjammedFootWaypoint, false);
			moveLimbToPointBasic(ultimateDestinationPoint, connectToWallAfterMakingMove);
		}
	}
	
	class LimbsCoordinator<T extends Limb> extends ArrayList<T> {

		private static final long serialVersionUID = 1L;
		
		Point leftShoulderDestination;
		
		int connectionsCount;
		
		boolean thereIsADisconnectedLimb;
		int disconnectedLimb;
		
		boolean thereIsALimbThatNeedsToMoveOrDisconnect;
		int limbThatNeedsToMoveOrDisconnect;
								
		@SuppressWarnings("unchecked")
		LimbsCoordinator () {

			add((T) leftArm);
			add((T) rightArm);
			add((T) leftLeg);
			add((T) rightLeg);
			
			hasReachedSeeminglyImpossiblePosition = false;
		}
		
		void establishLegs() {
			
			for (int i = 0; i < size(); i++) {
				
				if(!get(i).isArm()) {
					
					get(i).isConnectedToSomething = true;
	
				}
			}
		}
		
		void moveEachLimbIntoItsStartingPosition() {
			
			updateDestinationPointForBeltBuckle();
													
			updateDesiredRadialDirectionOfTravel();
			
			for (int i = 0; i < size(); i++) {
									
				get(i).getSetToClimb();
				
			}
			
			updateWallConnectionStatuses();
		}
		
		void updateWallConnectionStatuses() {
			
			connectionsCount = 0;
			
			for (int i = 0; i < size(); i++) {
				
				if (get(i).isConnectedToSomething) {
					
					connectionsCount++;
					
				} else {
					
					thereIsADisconnectedLimb = true;
					disconnectedLimb = i;

				}
			}
			
			if (connectionsCount == 4) {
				
				thereIsADisconnectedLimb = false;
				disconnectedLimb = -1;
			}
			
			if (!iHaveLeftGroundAndConnectedToWallWithAllFourLimbs) {
				
				checkWhetherIHaveLeftGroundAndConnectedToWallWithAllFourLimbs();
			}
		}
		
		void checkWhetherIHaveLeftGroundAndConnectedToWallWithAllFourLimbs() {
			
			int numberOfLimbsThatHaveMadeFirstConnectionToTheWall = 0;
			
			for (int i = 0; i < size(); i++) {
				
				if (get(i).hasMadeFirstConnectionToTheWall) {
					numberOfLimbsThatHaveMadeFirstConnectionToTheWall++;
				}
			}
			
			if (numberOfLimbsThatHaveMadeFirstConnectionToTheWall == 4) {
				iHaveLeftGroundAndConnectedToWallWithAllFourLimbs = true;
			}
		}
		
		void moveClimberAlongShortestPathToTop() {

			updateDestinationPointForBeltBuckle();
			
			if (climberHasToStopClimbingBecauseThereAreNoAttemptsRemaining()) {
				return;			
			}
			
			updateDesiredRadialDirectionOfTravel();
			
			updateWallConnectionStatuses();
			
			updateWhetherClimberHasMovedPastASeeminglyImpossiblePositionAndConnectedWithAllFourLimbs();
			
			if (thereIsADisconnectedLimb) {
					
				try {
					
					moveBothTorsoAndDisconnectedLimbAsFarAsPossibleWhileConstantlyTryingToConnectDisconnectedLimb();
					
					return;
					
				} catch (ImpossibleRouteException e) {

					iAmTryingToClimb = false;
					
					return;
					
				}
					
			} else {
			
				moveOnlyTorsoAsFarAsPossible();
			
				if (thereIsALimbThatNeedsToMoveOrDisconnect) {
					
					tryToMoveLimbThatNeedsToBeMovedByUsingAReachableHoldThatHasNotBeenUsedByLimbDuringEitherAscentOrBacktrackingAndIsAsFarInDesiredDirectionOfTravelAsPossible();
					
					if (!thereIsALimbThatNeedsToMoveOrDisconnect) {
						
						return;
					
					} else {
						
						disconnectAndMoveToTheReadyPositionTheLimbThatNeedsToMoveOrDisconnect();
						
						return;
						
					}
				}	
			}
		}
		
		void updateDestinationPointForBeltBuckle() {
			
			if (!hasReachedSeeminglyImpossiblePosition) {
				
				updateDestinationPointForBeltBuckleUsingPathIterator();
				
			} else if (hasReachedSeeminglyImpossiblePosition && hasReturnedSafely) {
				
				averagePathIndexIterator = null;
				
				updateDestinationPointForBeltBuckleUsingPathIterator();
				
				hasReachedSeeminglyImpossiblePosition = false;
				
				difficultMoveTracker++;
				
			} else {
				
				desiredDestinationForBeltBuckle = mostRecentFourConnectionsPoint;
				
				difficultMoveTracker++;
				
				iAmBacktracking = true;
				
			}
		}
		
		boolean climberHasToStopClimbingBecauseThereAreNoAttemptsRemaining() {
			
			if (difficultMoveTracker > 5) {
				
				iAmTryingToClimb = false;
				return true;
				
			} else {
				
				return false;
				
			}
		}
		
		void updateWhetherClimberHasMovedPastASeeminglyImpossiblePositionAndConnectedWithAllFourLimbs() {
			
			if (hasReachedDestinationPointThatWasBeyondSeeminglyImpossiblePosition && !thereIsADisconnectedLimb) {
				
				hasReachedSeeminglyImpossiblePosition = false;
				difficultMoveTracker = 0;
				indexOfAveragePathDestinationPointAtTheSeeminglyImpossiblePosition = Integer.MAX_VALUE;
				
			}	
		}
		
		void moveOnlyTorsoAsFarAsPossible() {
			
			calculateLeftShoulderDestinationBasedOnBeltBuckleDestinationPoint();
							
			ArrayList<float[]> waypointsFormattedAsFloat = calculateWaypointFloatCoordinates(leftShoulderSocketPos, leftShoulderDestination, Constants.TorsoMovementIncrement * Constants.SizeFactor);

			Point waypoint = new Point();
			
			for (float[] waypointCoordinatesInFloat : waypointsFormattedAsFloat) {
				
				waypoint = CommonAlgorithms.roundPoint(waypointCoordinatesInFloat[0], waypointCoordinatesInFloat[1]);
				
				try {
					
					prospectiveLeftShoulderSocketPos = waypoint;
					testAllProspectiveProximalLimbSegmentOrientationsAndLimbConnectionReachableAreas();
					
				} catch (LimbMustMoveFirstException e) {
					
					LimbIdentifier limbIdentifierPassedByException = e.getLimbIdentifier();
					
					if (limbIdentifierPassedByException == LimbIdentifier.LEFTARM) {
						limbThatNeedsToMoveOrDisconnect = 0;
					} else if (limbIdentifierPassedByException == LimbIdentifier.RIGHTARM) {
						limbThatNeedsToMoveOrDisconnect = 1;
					} else if (limbIdentifierPassedByException == LimbIdentifier.LEFTLEG) {
						limbThatNeedsToMoveOrDisconnect = 2;
					} else if (limbIdentifierPassedByException == LimbIdentifier.RIGHTLEG) {
						limbThatNeedsToMoveOrDisconnect = 3;
					}
					
					thereIsALimbThatNeedsToMoveOrDisconnect = true;
					
					return;
				}
				
				leftShoulderSocketPos = waypoint;
												
				updateAllWireframeAndDrawingPointsInProperSequence();
				updateLimbUsageRecordForReachableHolds();
				
				pauseForAnimationTiming();
			}
		}
		
		void moveBothTorsoAndDisconnectedLimbAsFarAsPossibleWhileConstantlyTryingToConnectDisconnectedLimb() throws ImpossibleRouteException {
			
			calculateLeftShoulderDestinationBasedOnBeltBuckleDestinationPoint();
			
			int xOffsetBetweenLeftShoulderSocketPosAndDisconnectedLimb = leftShoulderSocketPos.x - get(disconnectedLimb).getWallConnectionBodyPartPoint().x;
			int yOffsetBetweenLeftShoulderSocketPosAndDisconnectedLimb = leftShoulderSocketPos.y - get(disconnectedLimb).getWallConnectionBodyPartPoint().y;
			
			Point destinationOfWallConnectionPointForDisconnectedLimb = new Point(leftShoulderDestination.x - xOffsetBetweenLeftShoulderSocketPosAndDisconnectedLimb, leftShoulderDestination.y - yOffsetBetweenLeftShoulderSocketPosAndDisconnectedLimb);
	
			ArrayList<float[]> torsoMovementWaypointsFormattedAsFloat = calculateWaypointFloatCoordinates(leftShoulderSocketPos, leftShoulderDestination, Constants.TorsoMovementIncrement * Constants.SizeFactor);
			ArrayList<float[]> disconnectedLimbMovementWaypointsFormattedAsFloat = calculateWaypointFloatCoordinates(get(disconnectedLimb).getWallConnectionBodyPartPoint(), destinationOfWallConnectionPointForDisconnectedLimb, Constants.TorsoMovementIncrement * Constants.SizeFactor);
	
			Point torsoWaypoint = new Point();
			Point disconnectedLimbWaypoint = new Point();
			
			Point previousDisconnectedAndReadiedLimbWallConnectionBodyPartPoint = get(disconnectedLimb).getWallConnectionBodyPartPoint();
			
			for (int i = 0; i < torsoMovementWaypointsFormattedAsFloat.size(); i++) {
				
				torsoWaypoint = CommonAlgorithms.roundPoint(torsoMovementWaypointsFormattedAsFloat.get(i)[0], torsoMovementWaypointsFormattedAsFloat.get(i)[1]);
				disconnectedLimbWaypoint = CommonAlgorithms.roundPoint(disconnectedLimbMovementWaypointsFormattedAsFloat.get(i)[0], disconnectedLimbMovementWaypointsFormattedAsFloat.get(i)[1]);
			
				try {
					
					prospectiveLeftShoulderSocketPos = torsoWaypoint;
					testAllProspectiveProximalLimbSegmentOrientationsAndLimbConnectionReachableAreas();
					
				} catch (LimbMustMoveFirstException e) {
	
					LimbIdentifier limbIdentifierPassedByException = e.getLimbIdentifier();
					
					if (limbIdentifierPassedByException == LimbIdentifier.LEFTARM) {
						limbThatNeedsToMoveOrDisconnect = 0;
					} else if (limbIdentifierPassedByException == LimbIdentifier.RIGHTARM) {
						limbThatNeedsToMoveOrDisconnect = 1;
					} else if (limbIdentifierPassedByException == LimbIdentifier.LEFTLEG) {
						limbThatNeedsToMoveOrDisconnect = 2;
					} else if (limbIdentifierPassedByException == LimbIdentifier.RIGHTLEG) {
						limbThatNeedsToMoveOrDisconnect = 3;
					}
					
					hasReachedSeeminglyImpossiblePosition = true;
					indexOfAveragePathDestinationPointAtTheSeeminglyImpossiblePosition = averagePathIndexIterator;
					hasReachedDestinationPointThatWasBeyondSeeminglyImpossiblePosition = false;
					hasReturnedSafely = false;

					if (iAmBacktracking) {
						
						throw new ImpossibleRouteException();
						
					}
					
					return;
				}
				
				leftShoulderSocketPos = torsoWaypoint;
				get(disconnectedLimb).setWallConnectionBodyPartPoint(disconnectedLimbWaypoint);
								
				updateAllWireframeAndDrawingPointsInProperSequence();
				updateLimbUsageRecordForReachableHolds();
				
				pauseForAnimationTiming();
			
				if (disconnectedLimbDoesHaveOptions()) {
							
					connectDisconnectedLimbToTheReachableHoldThatIsFurthestInDesiredRadialDirectionOfTravelAmongAllReachableHolds();
					
					break;
					
				} else if (CommonAlgorithms.findDistance(previousDisconnectedAndReadiedLimbWallConnectionBodyPartPoint, disconnectedLimbWaypoint) > 1) {
					
					get(disconnectedLimb).limbHasMovedThroughAnAreaWhereItHadNoOptions = true;
				}
			}
			
			if (hasReachedSeeminglyImpossiblePosition) {
				
				if (beltBucklePos.x == mostRecentFourConnectionsPoint.x && beltBucklePos.y == mostRecentFourConnectionsPoint.y) {
									
					hasReturnedSafely = true;
				}
			}
		}

		void tryToMoveLimbThatNeedsToBeMovedByUsingAReachableHoldThatHasNotBeenUsedByLimbDuringEitherAscentOrBacktrackingAndIsAsFarInDesiredDirectionOfTravelAsPossible(){
			
			if (get(limbThatNeedsToMoveOrDisconnect).receiveARequestToMoveLimbToTheHoldThatIsFurthestInDesiredDirectionOfTravelAmongReachableHoldsOtherThanHoldsThatHaveBeenUsedDuringEitherAscentOrBacktrackingAndConfirmIfAMoveWasMade()) {
				
				thereIsALimbThatNeedsToMoveOrDisconnect = false;
				
			}
		}
		
		void calculateLeftShoulderDestinationBasedOnBeltBuckleDestinationPoint() {
			
			leftShoulderDestination = CommonAlgorithms.roundPoint(desiredDestinationForBeltBuckle.x - (shoulderSocketSpan/2), (desiredDestinationForBeltBuckle.y - (torsoHeightFromTopOfTrapsToWaist - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead))));
		}
		
		void testEachLimbForViolationOfReachableAreaUponProspectiveTorsoMove() throws LimbMustMoveFirstException {
			
			for (int i = 0; i < size(); i++) {
				
				if (i != disconnectedLimb) {
					
					if (!get(i).limbCanContinueToUseCurrentHoldIfTorsoMakesProspectiveMove()) {
						
						get(i).limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = true;
						
						throw new LimbMustMoveFirstException(get(i).limbIdentifier);
						
					}
				}
			}
		}
		
		void disconnectAndMoveToTheReadyPositionTheLimbThatNeedsToMoveOrDisconnect() {
			
			mostRecentFourConnectionsPoint = (Point) beltBucklePos.clone();

			advanceAsMuchAsPossibleAllLimbsThatArentGoingToBeDisconnected();
			
			disconnectAndReadyALimb();
			
		}
				
		void advanceAsMuchAsPossibleAllLimbsThatArentGoingToBeDisconnected() {
			
			for (int i = 0; i < size(); i++) {
				
				if (i != limbThatNeedsToMoveOrDisconnect) {
						
					get(i).receiveARequestToAdvanceLimbFromCurrentHoldToAReachableHoldThatIsMoreExtremeInDesiredDirectionOfTravelThanIsCurrentHoldAndConfirmIfAMoveWasMade();
				}
			}
		}
		
		void disconnectAndReadyALimb () {
			
			if (connectionsCount == 4) {
				
				disconnectedLimb = limbThatNeedsToMoveOrDisconnect;
				
				get(limbThatNeedsToMoveOrDisconnect).moveToReadyPoint();
				
				thereIsALimbThatNeedsToMoveOrDisconnect = false;
				limbThatNeedsToMoveOrDisconnect = -1;
				
				
			} else {
				
				System.out.println("I can't disconnect a limb if there aren't four connections at the time");
			}
		}
		
		boolean disconnectedLimbDoesHaveOptions() {
			
			get(disconnectedLimb).updateReachableHoldPoints();
			
			if (get(disconnectedLimb).allReachableHoldPoints.size() > 0 && get(disconnectedLimb).limbHadToAbandonHoldBecauseOfAViolationOfReachableArea && get(disconnectedLimb).limbHasMovedThroughAnAreaWhereItHadNoOptions) {
				
				return true;
				
			}
			
			ArrayList<Point> filteredPoints = new ArrayList<>();
						
			if (!iAmBacktracking) {
					
				for (Point p : get(disconnectedLimb).allReachableHoldPoints) {
					
					int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
					
					if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringAscent.contains(get(disconnectedLimb).limbIdentifier)) {
						
						filteredPoints.add(p);
						
					}
				}
				
			} else {
			
				for (Point p : get(disconnectedLimb).allReachableHoldPoints) {
					
					int indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds = finalizedSetOfUserEnteredHolds.indexOf(p);
					
					if (!finalizedSetOfUserEnteredHolds.get(indexOfHoldAtPointPWithinFinalizedSetOfUserEnteredHolds).limbsThatHaveUsedHoldDuringBacktrackingSequence.contains(get(disconnectedLimb).limbIdentifier)) {
						
						filteredPoints.add(p);
						
					}
				}
			}
				
			if (filteredPoints.size() > 0) {
				
				return true;
				
			} else {
				
				return false;
				
			}
		}
		
		void connectDisconnectedLimbToTheReachableHoldThatIsFurthestInDesiredRadialDirectionOfTravelAmongAllReachableHolds() {
			
			get(disconnectedLimb).moveLimbToTheReachableHoldThatIsFurthestInDesiredDirectionOfTravelAmongAllReachableHolds();
			
			thereIsADisconnectedLimb = false;
			disconnectedLimb = -1;
		}
		
		void updateAllCurrentReachableAreas() {
			
			for (int i = 0; i < size(); i++) {
				get(i).updateReachableAreaForCurrentPosition();
			}
		}
	}
	
	//I. METHODS RELATED TO SEQUENCING THE STEPS IN THE CLIMBING PROCESS
	
	@SuppressWarnings("unchecked")
	void receiveMessageThatTheWallIsReadyForClimber(ClimbingWall cw) {
		
		finalizedSetOfUserEnteredHolds = (ClimbingWall.SetOfHolds<ClimbingHold>) cw.setOfUserEnteredHolds.clone();
		
		findMostEfficientPathUpTheWallThroughTheHoldPoints();
		
		findThePathThroughEachSecondIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections();
		makeExtremePointBookendsForThePathThroughEachSecondIterationAveragePointOfHoldsWithinMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections();
			
		initiallyOrientClimber();
		
		iAmOnTheWall = true;
		iAmTryingToClimb = true;
		
		cw.receiveAndProcessNoticeThatClimberIsBeginningClimbingProcess();
		
		executeStepsForClimbingTheWall();
	}
	
	void initiallyOrientClimber() {
		
		Point lowestPointInAveragePath = new Point(pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.get(0).x, ClimbingWall.heightOfWall);
		
		Shape startingAreaRectangle = new Rectangle2D.Float(lowestPointInAveragePath.x - widestPossibleExtensionFromHandToHand/2, ClimbingWall.heightOfWall - highestPossibleExtensionFromFootToHand, widestPossibleExtensionFromHandToHand, highestPossibleExtensionFromFootToHand);
		Area startingArea = new Area(startingAreaRectangle);
		
		Point averagePointInStartingArea = CommonAlgorithms.averageThePointsInAnArea(finalizedSetOfUserEnteredHolds, startingArea);

		anchorPoint = new Point(averagePointInStartingArea.x, 0);
		
		setLoweringPosition();
	}	
	
	void setLoweringPosition(){
		
		leftShoulderSocketPos = CommonAlgorithms.roundPoint((anchorPoint.x - (shoulderSocketSpan/2)), ((-1) * (footHeight + calfLength + femurLength + torsoHeightFromTopOfTrapsToWaist)));
		updateTorsoPoints();
				
		leftHandCenterPos = CommonAlgorithms.roundPoint(anchorPoint.x, (leftShoulderSocketPos.y - (50 * Constants.SizeFactor)));
		rightHandCenterPos = CommonAlgorithms.roundPoint(anchorPoint.x, (leftShoulderSocketPos.y - (20 * Constants.SizeFactor)));
		leftFootHoldContactPoint = CommonAlgorithms.roundPoint((pelvisCenterPos.x - (30 * Constants.SizeFactor)), (pelvisCenterPos.y + (66 * Constants.SizeFactor)));
		rightFootHoldContactPoint = CommonAlgorithms.roundPoint((pelvisCenterPos.x + (30 * Constants.SizeFactor)), (pelvisCenterPos.y + (66 * Constants.SizeFactor)));
		
		updateAllCriticalPoints();
	}
	
	void executeStepsForClimbingTheWall() {
		
		ClimbingMovesProcessor movesProcessor = new ClimbingMovesProcessor();
		movesProcessor.execute();
	}
	
	class ClimbingMovesProcessor extends SwingWorker<String,String> {
		
		protected String doInBackground() {
			
			String dummyString = "";
			
			try {
				
				lowerClimber();
				prepareToClimb();
				ascendWall();
				
			} catch (Exception e) {
				e.printStackTrace();
			}
			
			return dummyString;
		}
	}
	
	void lowerClimber() {
		
		int tD = Math.round(Constants.LoweringTickDistance * Constants.SizeFactor);
		
		while(leftFootHoldContactPoint.y < ClimbingWall.heightOfWall) {
			
			leftShoulderSocketPos.y += tD;
			leftHandCenterPos.y += tD;
			rightHandCenterPos.y += tD;
			leftFootHoldContactPoint.y += tD;
			rightFootHoldContactPoint.y += tD;
			
			try {
				updateAllWireframeAndDrawingPointsInProperSequence();
			} catch (Exception e) {
				System.out.println("I'm having trouble lowering");
			}
			
			pauseForAnimationTiming();
		}

		limbsCoordinator.establishLegs();
	}
	
	void updateAllWireframeAndDrawingPointsInProperSequence() {
		
		updateTorsoPoints();
		updateAllCriticalPoints();
	}
	
	void updateAllCriticalPoints() {
		
		updateAllLimbWireframeValues();
		updateAllDrawingPoints();	
	}
		
	void prepareToClimb () {

		difficultMoveTracker = 0;
		
		limbsCoordinator.moveEachLimbIntoItsStartingPosition();
	}

	void ascendWall() {

		while (cannotTopOut() && iAmTryingToClimb) {
			
			limbsCoordinator.moveClimberAlongShortestPathToTop();
	
		}

		if (iAmTryingToClimb) {
		
			topOut();
		
		}
	}	

	boolean cannotTopOut() {
		
		return !(hasGottenWithinReachOfTop() && ((limbsCoordinator.connectionsCount == 4) || !leftArm.isConnectedToSomething));
		
	}
		
	void topOut() {
		
		Point leftHandTopout = new Point (leftShoulderSocketPos.x, 0);
		Point rightHandTopout = new Point (rightShoulderSocketPos.x, 0);
		
		leftArm.moveLimbToPointOverrideable(leftHandTopout, false);
		leftArm.isConnectedToSomething = true;
		rightArm.moveLimbToPointOverrideable(rightHandTopout, false);
		rightArm.isConnectedToSomething = true;
		
		int tD = Math.round(Constants.ToppingOutDistance * Constants.SizeFactor);
		
		while (beltBucklePos.y > 0) {
			
			leftShoulderSocketPos.y -= tD;

			leftLeg.updateOverallExtensionRatio();
			rightLeg.updateOverallExtensionRatio();
			
			if (leftLeg.overallExtensionRatio > Constants.MaximumOverallExtension) {
				
				leftFootHoldContactPoint.y -= tD;
				
				if(leftAnklePos.x != leftHipSocketPos.x) {
					
					leftFootHoldContactPoint.x += ((leftHipSocketPos.x - leftAnklePos.x)/Math.abs(leftHipSocketPos.x - leftAnklePos.x));
				}
			}
			
			if (rightLeg.overallExtensionRatio > Constants.MaximumOverallExtension) {
				
				rightFootHoldContactPoint.y -= tD;
				
				if(rightAnklePos.x != rightHipSocketPos.x) {
					
					rightFootHoldContactPoint.x += ((rightHipSocketPos.x - rightAnklePos.x)/Math.abs(rightHipSocketPos.x - rightAnklePos.x));
				}
			}
			
			try {
				updateAllWireframeAndDrawingPointsInProperSequence();
			} catch (Exception e) {
				System.out.println("I'm having trouble topping out");
			}
			
			pauseForAnimationTiming();
		}
		
		leftLeg.moveLimbToPointOverrideable(new Point(Math.round(leftShoulderSocketPos.x - (30 * Constants.SizeFactor)), 0), false);
		leftLeg.isConnectedToSomething = true;
		
		while(rightLeg.getWallConnectionBodyPartPoint().y > 0) {
			
			leftShoulderSocketPos.y -= tD;
			rightFootHoldContactPoint.y -= tD;
			
			updateAllWireframeAndDrawingPointsInProperSequence();
			
			pauseForAnimationTiming();
		}
		
		iAmOnTheWall = false;
	}
		
	boolean hasGottenWithinReachOfTop() {
		
		return (leftShoulderSocketPos.y < leftArm.maximumReachLength);
	}
	
	public boolean isClimberOnTheWall() {
		
		return iAmOnTheWall;
	}
	
	public boolean isClimberTryingToClimb() {
		
		return iAmTryingToClimb;
	}
	
	//II. GENERAL METHODS TO CALCULATE ANGLES/SLOPE, DISTANCES, COORDINATES, AND RELATIVE ORIENTATION 
	
	//calculate the orientation of a limb's endpoints (e.g., shoulder and wrist) as a radial angle pivoting on proximal endpoint (shoulder)
	float calculateLimbOrSegmentEndpointOrientation(Point proximal, Point distal) {
		return CommonAlgorithms.findSlopeAsRadians(proximal, distal);
	}
	
	//calculate distance between endpoints of limb's two major segments (e.g., shoulder and wrist)
	float calculateDistanceBetweenLimbEndpoints(Point proximal, Point distal) {
		return (float) CommonAlgorithms.findDistance(proximal, distal);
	}
			
	//result will be between pi radians (180 degrees) and -pi radians (-180 degrees)
	float calculateProximalLimbSegmentOrientation (float proximalSegmentLength, float distalSegmentLength, float distanceBetweenEndpointsOfSegments, float radialSlopeCreatedByEndpoints, boolean hingeFormedByLimbSegmentsIsAboveJoint, boolean leftSide) throws LawOfCosinesException {
		
		float proximalAngleInJointTriangle = (float) CommonAlgorithms.lawOfCosines(proximalSegmentLength, distanceBetweenEndpointsOfSegments, distalSegmentLength);

		if (proximalAngleInJointTriangle != proximalAngleInJointTriangle) {
			
			throw new LawOfCosinesException();
		}
		
		//variable adjuster used to account for joint orientation and side of body
		float adjuster = 1.0f;
		
		if (!hingeFormedByLimbSegmentsIsAboveJoint) {adjuster = adjuster * (-1.0f);}
		if (!leftSide) {adjuster = adjuster * (-1.0f);}
		
		float result = radialSlopeCreatedByEndpoints + (proximalAngleInJointTriangle * adjuster);
		if (result > Pi) {result = result - (2*Pi);}
		if (result < -Pi) {result = result + (2*Pi);}
		return result;	
	}
	
	Point calculateMidlimbJointPoint (Point proximalEndpoint, float proximalSegmentLength, float proximalLimbSegmentOrientation) {
		return CommonAlgorithms.shiftBySlopeAsRadians(proximalEndpoint, proximalSegmentLength, proximalLimbSegmentOrientation, true);
	}
	
	//III. METHODS RELATED TO SPECIFIC BODY PART POSITIONING
	
	//update torso points (including drawing points) based on location of left shoulder
	void updateTorsoPoints() {
		
		rightShoulderSocketPos.x = Math.round(leftShoulderSocketPos.x + shoulderSocketSpan);
		rightShoulderSocketPos.y = leftShoulderSocketPos.y;
		
		leftWaistDrawingPoint.x = Math.round(leftShoulderSocketPos.x + (shoulderSocketSpan * Constants.LatHorizontalCutInWidthAsRatioOfOverallShoulderSocket));
		leftWaistDrawingPoint.y = Math.round(leftShoulderSocketPos.y + (torsoHeightFromTopOfTrapsToWaist - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead)));
		rightWaistDrawingPoint.x = Math.round(rightShoulderSocketPos.x - (shoulderSocketSpan * Constants.LatHorizontalCutInWidthAsRatioOfOverallShoulderSocket));
		rightWaistDrawingPoint.y = Math.round(rightShoulderSocketPos.y + (torsoHeightFromTopOfTrapsToWaist - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead)));
		
		leftLatTorsoInsertionPoint.x = leftWaistDrawingPoint.x;
		leftLatTorsoInsertionPoint.y = Math.round(leftWaistDrawingPoint.y - (torsoHeightFromTopOfTrapsToWaist * Constants.LatInsertionPointRatioOfOverallTorsoHeightStartingFromWaist));
		rightLatTorsoInsertionPoint.x = rightWaistDrawingPoint.x;
		rightLatTorsoInsertionPoint.y = Math.round(rightWaistDrawingPoint.y - (torsoHeightFromTopOfTrapsToWaist * Constants.LatInsertionPointRatioOfOverallTorsoHeightStartingFromWaist));
		
		leftTrapNeckIntersectionPoint.x = Math.round(leftShoulderSocketPos.x + (shoulderSocketSpan/2 - neckThickness/2));
		leftTrapNeckIntersectionPoint.y = Math.round(leftShoulderSocketPos.y - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead));
		rightTrapNeckIntersectionPoint.x = Math.round(rightShoulderSocketPos.x - (shoulderSocketSpan/2 - neckThickness/2));
		rightTrapNeckIntersectionPoint.y = Math.round(rightShoulderSocketPos.y - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead));
		
		beltBucklePos = CommonAlgorithms.findMidpoint(leftWaistDrawingPoint, rightWaistDrawingPoint);
		pelvisCenterPos.x = beltBucklePos.x;
		pelvisCenterPos.y = Math.round(beltBucklePos.y + beltBuckleToCenterPelvisDistance);
		leftHipSocketPos.x = Math.round(pelvisCenterPos.x - hipSocketSpan/2);
		leftHipSocketPos.y = pelvisCenterPos.y;
		rightHipSocketPos.x = Math.round(pelvisCenterPos.x + hipSocketSpan/2);
		rightHipSocketPos.y = pelvisCenterPos.y;
		
		inseam.x = beltBucklePos.x;
		inseam.y = Math.round(beltBucklePos.y + waistToInseamCentralVerticalDistance);
		
		leftOuterPelvisHinge.x = leftWaistDrawingPoint.x;
		leftOuterPelvisHinge.y = Math.round(leftWaistDrawingPoint.y + hipToOuterPelvisCreaseVerticalDistance);
		leftInnerPelvisHinge.x = Math.round(beltBucklePos.x - inseamToInnerPelvisCreaseHorizontalDistance);
		leftInnerPelvisHinge.y = inseam.y;

		rightOuterPelvisHinge.x = rightWaistDrawingPoint.x;
		rightOuterPelvisHinge.y = Math.round(rightWaistDrawingPoint.y + hipToOuterPelvisCreaseVerticalDistance);
		rightInnerPelvisHinge.x = Math.round(beltBucklePos.x + inseamToInnerPelvisCreaseHorizontalDistance);
		rightInnerPelvisHinge.y = inseam.y;
		
		torsoCOG.x = beltBucklePos.x;
		torsoCOG.y = Math.round(leftTrapNeckIntersectionPoint.y + (torsoHeightFromTopOfTrapsToWaist * Constants.DistanceFromTopOfTrapsToTorsoCOGComparedToDistanceFromTopOfTrapsToWaistRatio));
	}

	void updateCurrentHipBox() {
		
		leftUpperOuterHipBoxPos = CommonAlgorithms.roundPoint(leftShoulderSocketPos.x - (footLength + (upperArmWidth/2)), torsoCOG.y);
		rightUpperOuterHipBoxPos = CommonAlgorithms.roundPoint(leftShoulderSocketPos.x + shoulderSocketSpan + (footLength + (upperArmWidth/2)), torsoCOG.y);
		Point lowerRightHipBoxVertex = CommonAlgorithms.roundPoint(leftShoulderSocketPos.x + shoulderSocketSpan + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), beltBucklePos.y + waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor));
		Point lowerLeftHipBoxVertex = CommonAlgorithms.roundPoint(leftShoulderSocketPos.x - ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), beltBucklePos.y + waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor));
		
		Path2D.Float hipBoxShape = new Path2D.Float();
		
		hipBoxShape.moveTo(leftUpperOuterHipBoxPos.x, leftUpperOuterHipBoxPos.y);
		hipBoxShape.lineTo(rightUpperOuterHipBoxPos.x, rightUpperOuterHipBoxPos.y);
		hipBoxShape.lineTo(lowerRightHipBoxVertex.x, lowerRightHipBoxVertex.y);
		hipBoxShape.lineTo(lowerLeftHipBoxVertex.x, lowerLeftHipBoxVertex.y);
		hipBoxShape.closePath();
		
		currentHipBox = new Area(hipBoxShape);
	}
	
	void updateProspectiveHipBox() {
		
		prospectiveLeftUpperOuterHipBoxPos = CommonAlgorithms.roundPoint(prospectiveLeftShoulderSocketPos.x - (footLength + (upperArmWidth/2)), prospectiveTorsoCOG.y);
		prospectiveRightUpperOuterHipBoxPos = CommonAlgorithms.roundPoint(prospectiveLeftShoulderSocketPos.x + shoulderSocketSpan + (footLength + (upperArmWidth/2)), prospectiveTorsoCOG.y);
		Point prospectiveLowerRightHipBoxVertex = CommonAlgorithms.roundPoint(prospectiveLeftShoulderSocketPos.x + shoulderSocketSpan + ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), prospectiveBeltBucklePos.y + waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor));
		Point prospectiveLowerLeftHipBoxVertex = CommonAlgorithms.roundPoint(prospectiveLeftShoulderSocketPos.x - ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe)), prospectiveBeltBucklePos.y + waistToInseamCentralVerticalDistance + footHeight + (Constants.HoldDiameter * Constants.SizeFactor));
		
		Path2D.Float prospectiveHipBoxShape = new Path2D.Float();
		
		prospectiveHipBoxShape.moveTo(prospectiveLeftUpperOuterHipBoxPos.x, prospectiveLeftUpperOuterHipBoxPos.y);
		prospectiveHipBoxShape.lineTo(prospectiveRightUpperOuterHipBoxPos.x, prospectiveRightUpperOuterHipBoxPos.y);
		prospectiveHipBoxShape.lineTo(prospectiveLowerRightHipBoxVertex.x, prospectiveLowerRightHipBoxVertex.y);
		prospectiveHipBoxShape.lineTo(prospectiveLowerLeftHipBoxVertex.x, prospectiveLowerLeftHipBoxVertex.y);
		prospectiveHipBoxShape.closePath();
		
		prospectiveHipBox = new Area(prospectiveHipBoxShape);
	}
		
	void updateLeftArmWireframeValues() {
		
		float apparentLeftHumerusLength = humerusLength;
		float apparentLeftForearmLength = forearmLength + handRadius;
		
		float distanceBetweenLeftShoulderAndLeftHand = CommonAlgorithms.findDistance(leftShoulderSocketPos, leftHandCenterPos);
		float leftShoulderToLeftHandOrientation = calculateLimbOrSegmentEndpointOrientation(leftShoulderSocketPos, leftHandCenterPos);
		
		float humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward = 1;
		float humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = 1;
		
		float forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward = 1;
		float forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint = 1;
		
		if((leftShoulderSocketPos.x == leftHandCenterPos.x) && (leftShoulderSocketPos.y == leftHandCenterPos.y)) {
			leftHumerusOrientation = (-0.75f) * Pi;
			leftElbowPos = calculateMidlimbJointPoint(leftShoulderSocketPos, humerusLength, leftHumerusOrientation);
			leftForearmOrientation = calculateLimbOrSegmentEndpointOrientation(leftElbowPos, leftHandCenterPos);
			leftWristPivot = leftForearmOrientation;
			leftWristPos = findWristPointFromHandPoint(leftHandCenterPos, leftWristPivot);	
			return;
		}
		
		Point lowestLeftElbowPointPossible = CommonAlgorithms.roundPoint(leftShoulderSocketPos.x, (leftShoulderSocketPos.y + humerusLength));
		float distanceBetweenLeftHandAndLowestLeftElbowPoint = CommonAlgorithms.findDistance(lowestLeftElbowPointPossible, leftHandCenterPos);
		
		Point furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.roundPoint((leftShoulderSocketPos.x + (forearmLength + handRadius)), leftShoulderSocketPos.y);
		float distanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.findDistance(leftHandCenterPos, furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		
		float verticalJammingComponent = Math.abs(lowestLeftElbowPointPossible.y - leftHandCenterPos.y);
		float horizontalJammingComponent = Math.abs(furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket.x - leftHandCenterPos.x);
		
		float verticalDistanceAwayFromShoulderXAxis = leftHandCenterPos.y - leftShoulderSocketPos.y;
		float horizontalDistanceAwayFromShoulderYAxis = leftHandCenterPos.x - leftShoulderSocketPos.x;
		
		boolean jammedWithinHumerusLengthDownward = (verticalJammingComponent < humerusLength && Math.abs(horizontalDistanceAwayFromShoulderYAxis) < (forearmLength + handRadius));
		boolean jammedWithinForearmAndHandLengthInward = (horizontalJammingComponent < (forearmLength + handRadius) && Math.abs(verticalDistanceAwayFromShoulderXAxis) < humerusLength);
		
		boolean jammedByProximityToLowestElbowPoint = (distanceBetweenLeftHandAndLowestLeftElbowPoint < (forearmLength + handRadius));
		boolean jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder = (distanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket < (humerusLength));
		
		if (jammedWithinHumerusLengthDownward) {
			forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward *= Math.abs((horizontalDistanceAwayFromShoulderYAxis)/(forearmLength + handRadius));
		}
		
		if (jammedByProximityToLowestElbowPoint) {
			forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint *= Math.sqrt((distanceBetweenLeftHandAndLowestLeftElbowPoint)/(forearmLength + handRadius));
		}
		
		if (jammedWithinForearmAndHandLengthInward) {
			humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward *= Math.abs((verticalDistanceAwayFromShoulderXAxis)/(humerusLength));
		}
		
		if (jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder) {
			humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket *= Math.sqrt((distanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket)/(humerusLength));
		}
		
		float firstStageHumerusShortener = ((humerusLength - horizontalJammingComponent) * (1 - humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward));
		float firstStageForearmShortener = (((forearmLength + handRadius) - verticalJammingComponent) * (1 - forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward));
		
		apparentLeftHumerusLength -= firstStageHumerusShortener;
		apparentLeftForearmLength -= firstStageForearmShortener;
		
		float remainingHumerusLeeway = Math.min(((apparentLeftHumerusLength) + distanceBetweenLeftShoulderAndLeftHand) - apparentLeftForearmLength, (apparentLeftHumerusLength + apparentLeftForearmLength) - distanceBetweenLeftShoulderAndLeftHand);
		float remainingForearmLeeway = Math.min(((apparentLeftForearmLength) + distanceBetweenLeftShoulderAndLeftHand) - apparentLeftHumerusLength, ((apparentLeftForearmLength) + apparentLeftHumerusLength) - distanceBetweenLeftShoulderAndLeftHand);
		
		float secondStageHumerusShortener = remainingHumerusLeeway * (1- humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		float secondStageForearmShortener = remainingForearmLeeway * (1- forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint);
		
		apparentLeftHumerusLength -= secondStageHumerusShortener;		
		apparentLeftForearmLength -= secondStageForearmShortener;
		
		try {
			if (leftHandCenterPos.y >= leftShoulderSocketPos.y && leftHandCenterPos.x >= leftShoulderSocketPos.x ) {
				leftHumerusOrientation = calculateProximalLimbSegmentOrientation(apparentLeftHumerusLength, apparentLeftForearmLength, distanceBetweenLeftShoulderAndLeftHand, leftShoulderToLeftHandOrientation, false, true);
			} else {
				leftHumerusOrientation = calculateProximalLimbSegmentOrientation(apparentLeftHumerusLength, apparentLeftForearmLength, distanceBetweenLeftShoulderAndLeftHand, leftShoulderToLeftHandOrientation, true, true);
			}
		} catch (LawOfCosinesException e) {
			
		}
		
		leftElbowPos = calculateMidlimbJointPoint(leftShoulderSocketPos, apparentLeftHumerusLength, leftHumerusOrientation);
		
		leftForearmOrientation = calculateLimbOrSegmentEndpointOrientation(leftElbowPos, leftHandCenterPos);
		
		leftWristPivot = leftForearmOrientation;
	
		leftWristPos = findWristPointFromHandPoint(leftHandCenterPos, leftWristPivot);
	}
	
	void updateRightArmWireframeValues() {
		
		float apparentRightHumerusLength = humerusLength;
		float apparentRightForearmLength = forearmLength + handRadius;
		
		float distanceBetweenRightShoulderAndRightHand = CommonAlgorithms.findDistance(rightShoulderSocketPos, rightHandCenterPos);
		float rightShoulderToRightHandOrientation = calculateLimbOrSegmentEndpointOrientation(rightShoulderSocketPos, rightHandCenterPos);
		
		float humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward = 1;
		float humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = 1;
		
		float forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward = 1;
		float forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint = 1;

		if((rightShoulderSocketPos.x == rightHandCenterPos.x) && (rightShoulderSocketPos.y == rightHandCenterPos.y)) {
			rightHumerusOrientation = (-0.25f) * Pi;
			rightElbowPos = calculateMidlimbJointPoint(rightShoulderSocketPos, humerusLength, rightHumerusOrientation);
			rightForearmOrientation = calculateLimbOrSegmentEndpointOrientation(rightElbowPos, rightHandCenterPos);
			rightWristPivot = rightForearmOrientation;
			rightWristPos = findWristPointFromHandPoint(rightHandCenterPos, rightWristPivot);		
			return;
		}
		
		Point lowestRightElbowPointPossible = CommonAlgorithms.roundPoint(rightShoulderSocketPos.x, (rightShoulderSocketPos.y + humerusLength));
		float distanceBetweenRightHandAndLowestRightElbowPoint = CommonAlgorithms.findDistance(lowestRightElbowPointPossible, rightHandCenterPos);
		
		Point furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.roundPoint((rightShoulderSocketPos.x - (forearmLength + handRadius)), rightShoulderSocketPos.y);
		float distanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.findDistance(rightHandCenterPos, furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		
		float verticalJammingComponent = Math.abs(lowestRightElbowPointPossible.y- rightHandCenterPos.y);
		float horizontalJammingComponent = Math.abs(rightHandCenterPos.x - furthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket.x);
		float verticalDistanceAwayFromShoulderXAxis = rightHandCenterPos.y - rightShoulderSocketPos.y;
		float horizontalDistanceAwayFromShoulderYAxis = rightShoulderSocketPos.x - rightHandCenterPos.x;
		
		boolean jammedWithinHumerusLengthDownward = (verticalJammingComponent < humerusLength && Math.abs(horizontalDistanceAwayFromShoulderYAxis) < (forearmLength + handRadius));
		boolean jammedWithinForearmAndHandLengthInward = (horizontalJammingComponent < (forearmLength + handRadius) && Math.abs(verticalDistanceAwayFromShoulderXAxis) < humerusLength);
		boolean jammedByProximityToLowestElbowPoint = (distanceBetweenRightHandAndLowestRightElbowPoint < (forearmLength + handRadius));
		boolean jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder = (distanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket < (humerusLength));
		
		if (jammedWithinHumerusLengthDownward) {
			forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward *= Math.abs((horizontalDistanceAwayFromShoulderYAxis)/(forearmLength + handRadius));
		}
	
		if (jammedByProximityToLowestElbowPoint) {
			forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint *= Math.sqrt((distanceBetweenRightHandAndLowestRightElbowPoint)/(forearmLength + handRadius));
		}
		
		if (jammedWithinForearmAndHandLengthInward) {
			humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward *= Math.abs((verticalDistanceAwayFromShoulderXAxis)/(humerusLength));
		}
		
		if (jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder) {
			humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket *= Math.sqrt((distanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket)/(humerusLength));
		}

		float firstStageHumerusShortener = ((humerusLength - horizontalJammingComponent) * (1 - humerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward));
		float firstStageForearmShortener = (((forearmLength + handRadius) - verticalJammingComponent) * (1 - forearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward));
	
		apparentRightHumerusLength -= firstStageHumerusShortener;
		apparentRightForearmLength -= firstStageForearmShortener;
	
		float remainingHumerusLeeway = Math.min(((apparentRightHumerusLength) + distanceBetweenRightShoulderAndRightHand) - apparentRightForearmLength, (apparentRightHumerusLength + apparentRightForearmLength) - distanceBetweenRightShoulderAndRightHand);
		float remainingForearmLeeway = Math.min(((apparentRightForearmLength) + distanceBetweenRightShoulderAndRightHand) - apparentRightHumerusLength, ((apparentRightForearmLength) + apparentRightHumerusLength) - distanceBetweenRightShoulderAndRightHand);
			
		float secondStageHumerusShortener = remainingHumerusLeeway * (1 - humerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		float secondStageForearmShortener = remainingForearmLeeway * (1- forearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint);
		
		apparentRightHumerusLength -= secondStageHumerusShortener;
		apparentRightForearmLength -= secondStageForearmShortener;

		try { 
			if (rightHandCenterPos.y >= rightShoulderSocketPos.y && rightHandCenterPos.x <= rightShoulderSocketPos.x ) {
				rightHumerusOrientation = calculateProximalLimbSegmentOrientation(apparentRightHumerusLength, apparentRightForearmLength, distanceBetweenRightShoulderAndRightHand, rightShoulderToRightHandOrientation, false, false);
			} else {
				rightHumerusOrientation = calculateProximalLimbSegmentOrientation(apparentRightHumerusLength, apparentRightForearmLength, distanceBetweenRightShoulderAndRightHand, rightShoulderToRightHandOrientation, true, false);
			}
		} catch (LawOfCosinesException e) {
			
		}
			
		rightElbowPos = calculateMidlimbJointPoint(rightShoulderSocketPos, apparentRightHumerusLength, rightHumerusOrientation);
		
		rightForearmOrientation = calculateLimbOrSegmentEndpointOrientation(rightElbowPos, rightHandCenterPos);
		
		rightWristPivot = rightForearmOrientation;
		
		rightWristPos = findWristPointFromHandPoint(rightHandCenterPos, rightWristPivot);
	}
	
	void updateLeftLegWireframeValues() {
	
		leftAnklePos = findAnklePointFromFootPoint(leftFootHoldContactPoint,true);
		
		float pelvisToLeftAnkleOrientation = calculateLimbOrSegmentEndpointOrientation(leftHipSocketPos, leftAnklePos);
		float distanceBetweenPelvisAndLeftAnkle = CommonAlgorithms.findDistance(leftHipSocketPos, leftAnklePos);
		
		try {
			leftThighOrientation = calculateProximalLimbSegmentOrientation(femurLength, calfLength, distanceBetweenPelvisAndLeftAnkle, pelvisToLeftAnkleOrientation, false, true);
		} catch (LawOfCosinesException e) {
			
		}
				
		leftKneePos = calculateMidlimbJointPoint(leftHipSocketPos, femurLength, leftThighOrientation);
		
		leftCalfOrientation = calculateLimbOrSegmentEndpointOrientation(leftKneePos, leftAnklePos);
	}
	
	void updateRightLegWireframeValues() {
		
		rightAnklePos = findAnklePointFromFootPoint(rightFootHoldContactPoint,false);
		
		float pelvisToRightAnkleOrientation = calculateLimbOrSegmentEndpointOrientation(rightHipSocketPos, rightAnklePos);
		float distanceBetweenPelvisAndRightAnkle = CommonAlgorithms.findDistance(rightHipSocketPos, rightAnklePos);
		
		try {
			rightThighOrientation = calculateProximalLimbSegmentOrientation(femurLength, calfLength, distanceBetweenPelvisAndRightAnkle, pelvisToRightAnkleOrientation, false, false);
		} catch (LawOfCosinesException e) {
			
		}
			
			
		rightKneePos = calculateMidlimbJointPoint(rightHipSocketPos, femurLength, rightThighOrientation);
		
		rightCalfOrientation = calculateLimbOrSegmentEndpointOrientation(rightKneePos, rightAnklePos);
	}
	
	void testAllProspectiveProximalLimbSegmentOrientationsAndLimbConnectionReachableAreas() throws LimbMustMoveFirstException {
		
		updateProspectiveTorsoPoints();
		
		testAllProspectiveProximalLimbSegmentOrientationsForExceedingLimbExtensionLimits();
		
		limbsCoordinator.testEachLimbForViolationOfReachableAreaUponProspectiveTorsoMove();
	}
		
	void testAllProspectiveProximalLimbSegmentOrientationsForExceedingLimbExtensionLimits() throws LimbMustMoveFirstException {

		updateProspectiveLeftHumerusOrientationBasedOnProspectiveTorsoMove();
		updateProspectiveRightHumerusOrientationBasedOnProspectiveTorsoMove();
		updateProspectiveLeftThighOrientationBasedOnProspectiveTorsoMove();
		updateProspectiveRightThighOrientationBasedOnProspectiveTorsoMove();
	}
	
	void updateProspectiveTorsoPoints() {
		
		prospectiveRightShoulderSocketPos.x = Math.round(prospectiveLeftShoulderSocketPos.x + shoulderSocketSpan);
		prospectiveRightShoulderSocketPos.y = prospectiveLeftShoulderSocketPos.y;
		
		prospectiveBeltBucklePos.x = Math.round(prospectiveLeftShoulderSocketPos.x + (shoulderSocketSpan/2));
		prospectiveBeltBucklePos.y = Math.round(prospectiveLeftShoulderSocketPos.y + (torsoHeightFromTopOfTrapsToWaist - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead)));
		
		prospectivePelvisCenterPos.x = prospectiveBeltBucklePos.x;
		prospectivePelvisCenterPos.y = Math.round(prospectiveBeltBucklePos.y + beltBuckleToCenterPelvisDistance);
		
		prospectiveLeftHipSocketPos.x = Math.round(prospectivePelvisCenterPos.x - hipSocketSpan/2);
		prospectiveLeftHipSocketPos.y = prospectivePelvisCenterPos.y;
		prospectiveRightHipSocketPos.x = Math.round(prospectivePelvisCenterPos.x + hipSocketSpan/2);
		prospectiveRightHipSocketPos.y = prospectivePelvisCenterPos.y;
		
		Point prospectiveLeftTrapNeckIntersectionPoint = new Point();
		
		prospectiveLeftTrapNeckIntersectionPoint.x = Math.round(prospectiveLeftShoulderSocketPos.x + (shoulderSocketSpan/2 - neckThickness/2));
		prospectiveLeftTrapNeckIntersectionPoint.y = Math.round(prospectiveLeftShoulderSocketPos.y - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle * Constants.FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead));
		
		prospectiveTorsoCOG.x = prospectiveBeltBucklePos.x;
		prospectiveTorsoCOG.y = Math.round(prospectiveLeftTrapNeckIntersectionPoint.y + (torsoHeightFromTopOfTrapsToWaist * Constants.DistanceFromTopOfTrapsToTorsoCOGComparedToDistanceFromTopOfTrapsToWaistRatio));
	}
	
	void updateProspectiveLeftHumerusOrientationBasedOnProspectiveTorsoMove() throws LimbMustMoveFirstException {
		
		@SuppressWarnings("unused")
		float prospectiveLeftHumerusOrientation;
		
		float prospectiveApparentLeftHumerusLength = humerusLength;
		float prospectiveApparentLeftForearmLength = forearmLength + handRadius;
		
		float prospectiveDistanceBetweenLeftShoulderAndLeftHand = CommonAlgorithms.findDistance(prospectiveLeftShoulderSocketPos, leftHandCenterPos);
		float prospectiveLeftShoulderToLeftHandOrientation = calculateLimbOrSegmentEndpointOrientation(prospectiveLeftShoulderSocketPos, leftHandCenterPos);
		

		float prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward = 1;
		float prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = 1;
		
		float prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward = 1;
		float prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint = 1;
		
		if((prospectiveLeftShoulderSocketPos.x == leftHandCenterPos.x) && (prospectiveLeftShoulderSocketPos.y == leftHandCenterPos.y)) {
			prospectiveLeftHumerusOrientation = (-0.75f) * Pi;
			return;
		}
		
		Point prospectiveLowestLeftElbowPointPossible = CommonAlgorithms.roundPoint(prospectiveLeftShoulderSocketPos.x, (prospectiveLeftShoulderSocketPos.y + humerusLength));
		float prospectiveDistanceBetweenLeftHandAndLowestLeftElbowPoint = CommonAlgorithms.findDistance(prospectiveLowestLeftElbowPointPossible, leftHandCenterPos);
		
		Point prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.roundPoint((prospectiveLeftShoulderSocketPos.x + (forearmLength + handRadius)), prospectiveLeftShoulderSocketPos.y);
		float prospectiveDistanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.findDistance(leftHandCenterPos, prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		
		float prospectiveVerticalJammingComponent = Math.abs(prospectiveLowestLeftElbowPointPossible.y - leftHandCenterPos.y);
		float prospectiveHorizontalJammingComponent = Math.abs(prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket.x - leftHandCenterPos.x);
		
		float prospectiveVerticalDistanceAwayFromShoulderXAxis = leftHandCenterPos.y - prospectiveLeftShoulderSocketPos.y;
		float prospectiveHorizontalDistanceAwayFromShoulderYAxis = leftHandCenterPos.x - prospectiveLeftShoulderSocketPos.x;
		
		boolean jammedWithinHumerusLengthDownward = (prospectiveVerticalJammingComponent < humerusLength && Math.abs(prospectiveHorizontalDistanceAwayFromShoulderYAxis) < (forearmLength + handRadius));
		boolean jammedWithinForearmAndHandLengthInward = (prospectiveHorizontalJammingComponent < (forearmLength + handRadius) && Math.abs(prospectiveVerticalDistanceAwayFromShoulderXAxis) < humerusLength);
		
		boolean jammedByProximityToLowestElbowPoint = (prospectiveDistanceBetweenLeftHandAndLowestLeftElbowPoint < (forearmLength + handRadius));
		boolean jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder = (prospectiveDistanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket < (humerusLength));
		
		if (jammedWithinHumerusLengthDownward) {
			prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward *= Math.abs((prospectiveHorizontalDistanceAwayFromShoulderYAxis)/(forearmLength + handRadius));
		}
		
		if (jammedByProximityToLowestElbowPoint) {
			prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint *= Math.sqrt((prospectiveDistanceBetweenLeftHandAndLowestLeftElbowPoint)/(forearmLength + handRadius));
		}
		
		if (jammedWithinForearmAndHandLengthInward) {
			prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward *= Math.abs((prospectiveVerticalDistanceAwayFromShoulderXAxis)/(humerusLength));
		}
		
		if (jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder) {
			prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket *= Math.sqrt((prospectiveDistanceBetweenLeftHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket)/(humerusLength));
		}
		
		float firstStageHumerusShortener = ((humerusLength - prospectiveHorizontalJammingComponent) * (1 - prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward));
		float firstStageForearmShortener = (((forearmLength + handRadius) - prospectiveVerticalJammingComponent) * (1 - prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward));
		
		prospectiveApparentLeftHumerusLength -= firstStageHumerusShortener;
		prospectiveApparentLeftForearmLength -= firstStageForearmShortener;
		
		float remainingHumerusLeeway = Math.min(((prospectiveApparentLeftHumerusLength) + prospectiveDistanceBetweenLeftShoulderAndLeftHand) - prospectiveApparentLeftForearmLength, (prospectiveApparentLeftHumerusLength + prospectiveApparentLeftForearmLength) - prospectiveDistanceBetweenLeftShoulderAndLeftHand);
		float remainingForearmLeeway = Math.min(((prospectiveApparentLeftForearmLength) + prospectiveDistanceBetweenLeftShoulderAndLeftHand) - prospectiveApparentLeftHumerusLength, ((prospectiveApparentLeftForearmLength) + prospectiveApparentLeftHumerusLength) - prospectiveDistanceBetweenLeftShoulderAndLeftHand);
		
		float secondStageHumerusShortener = remainingHumerusLeeway * (1 - prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		float secondStageForearmShortener = remainingForearmLeeway * (1 - prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint);
		
		prospectiveApparentLeftHumerusLength -= secondStageHumerusShortener;		
		prospectiveApparentLeftForearmLength -= secondStageForearmShortener;
		
		try {
			if (leftHandCenterPos.y >= prospectiveLeftShoulderSocketPos.y && leftHandCenterPos.x >= prospectiveLeftShoulderSocketPos.x ) {
				prospectiveLeftHumerusOrientation = calculateProximalLimbSegmentOrientation(prospectiveApparentLeftHumerusLength, prospectiveApparentLeftForearmLength, prospectiveDistanceBetweenLeftShoulderAndLeftHand, prospectiveLeftShoulderToLeftHandOrientation, false, true);
			} else {
				prospectiveLeftHumerusOrientation = calculateProximalLimbSegmentOrientation(prospectiveApparentLeftHumerusLength, prospectiveApparentLeftForearmLength, prospectiveDistanceBetweenLeftShoulderAndLeftHand, prospectiveLeftShoulderToLeftHandOrientation, true, true);
			}
		} catch (LawOfCosinesException e) {
			
			leftArm.limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = true;
			
			throw new LimbMustMoveFirstException(LimbIdentifier.LEFTARM);
		}
	}
	
	void updateProspectiveRightHumerusOrientationBasedOnProspectiveTorsoMove() throws LimbMustMoveFirstException {
		
		@SuppressWarnings("unused")
		float prospectiveRightHumerusOrientation;
		
		float prospectiveApparentRightHumerusLength = humerusLength;
		float prospectiveApparentRightForearmLength = forearmLength + handRadius;
		
		float prospectiveDistanceBetweenRightShoulderAndRightHand = CommonAlgorithms.findDistance(prospectiveRightShoulderSocketPos, rightHandCenterPos);
		float prospectiveRightShoulderToRightHandOrientation = calculateLimbOrSegmentEndpointOrientation(prospectiveRightShoulderSocketPos, rightHandCenterPos);
		
		float prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward = 1;
		float prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = 1;
		
		float prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward = 1;
		float prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint = 1;
		
		if((prospectiveRightShoulderSocketPos.x == rightHandCenterPos.x) && (prospectiveRightShoulderSocketPos.y == rightHandCenterPos.y)) {
			prospectiveRightHumerusOrientation = (-0.25f) * Pi;
			return;
		}
		
		Point prospectiveLowestRightElbowPointPossible = CommonAlgorithms.roundPoint(prospectiveRightShoulderSocketPos.x, (prospectiveRightShoulderSocketPos.y + humerusLength));
		float prospectiveDistanceBetweenRightHandAndLowestRightElbowPoint = CommonAlgorithms.findDistance(prospectiveLowestRightElbowPointPossible, rightHandCenterPos);
		
		Point prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.roundPoint((prospectiveRightShoulderSocketPos.x - (forearmLength + handRadius)), prospectiveRightShoulderSocketPos.y);
		float prospectiveDistanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket = CommonAlgorithms.findDistance(rightHandCenterPos, prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		
		float prospectiveVerticalJammingComponent = Math.abs(prospectiveLowestRightElbowPointPossible.y- rightHandCenterPos.y);
		float prospectiveHorizontalJammingComponent = Math.abs(rightHandCenterPos.x - prospectiveFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket.x);
		
		float prospectiveVerticalDistanceAwayFromShoulderXAxis = rightHandCenterPos.y - prospectiveRightShoulderSocketPos.y;
		float prospectiveHorizontalDistanceAwayFromShoulderYAxis = prospectiveRightShoulderSocketPos.x - rightHandCenterPos.x;
		
		boolean jammedWithinHumerusLengthDownward = (prospectiveVerticalJammingComponent < humerusLength && Math.abs(prospectiveHorizontalDistanceAwayFromShoulderYAxis) < (forearmLength + handRadius));
		boolean jammedWithinForearmAndHandLengthInward = (prospectiveHorizontalJammingComponent < (forearmLength + handRadius) && Math.abs(prospectiveVerticalDistanceAwayFromShoulderXAxis) < humerusLength);
		
		boolean jammedByProximityToLowestElbowPoint = (prospectiveDistanceBetweenRightHandAndLowestRightElbowPoint < (forearmLength + handRadius));
		boolean jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder = (prospectiveDistanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket < (humerusLength));
		
		if (jammedWithinHumerusLengthDownward) {
			prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward *= Math.abs((prospectiveHorizontalDistanceAwayFromShoulderYAxis)/(forearmLength + handRadius));
		}
	
		if (jammedByProximityToLowestElbowPoint) {
			prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint *= Math.sqrt((prospectiveDistanceBetweenRightHandAndLowestRightElbowPoint)/(forearmLength + handRadius));
		}
		
		if (jammedWithinForearmAndHandLengthInward) {
			prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward *= Math.abs((prospectiveVerticalDistanceAwayFromShoulderXAxis)/(humerusLength));
		}
		
		if (jammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulder) {
			prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket *= Math.sqrt((prospectiveDistanceBetweenRightHandAndFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket)/(humerusLength));
		}

		float firstStageHumerusShortener = ((humerusLength - prospectiveHorizontalJammingComponent) * (1 - prospectiveHumerusFreedomFactorDueToBeingJammedWithinForearmLengthAndHandRadiusInward));
		float firstStageForearmShortener = (((forearmLength + handRadius) - prospectiveVerticalJammingComponent) * (1 - prospectiveForearmFreedomFactorDueToBeingJammedWithinHumerusLengthDownward));
	
		prospectiveApparentRightHumerusLength -= firstStageHumerusShortener;
		prospectiveApparentRightForearmLength -= firstStageForearmShortener;
	
		float remainingHumerusLeeway = Math.min(((prospectiveApparentRightHumerusLength) + prospectiveDistanceBetweenRightShoulderAndRightHand) - prospectiveApparentRightForearmLength, (prospectiveApparentRightHumerusLength + prospectiveApparentRightForearmLength) - prospectiveDistanceBetweenRightShoulderAndRightHand);
		float remainingForearmLeeway = Math.min(((prospectiveApparentRightForearmLength) + prospectiveDistanceBetweenRightShoulderAndRightHand) - prospectiveApparentRightHumerusLength, ((prospectiveApparentRightForearmLength) + prospectiveApparentRightHumerusLength) - prospectiveDistanceBetweenRightShoulderAndRightHand);
			
		float secondStageHumerusShortener = remainingHumerusLeeway * (1 - prospectiveHumerusFreedomFactorDueToBeingJammedByProximityToFurthestInteriorPointAlongShoulderHorizontalAxisWithElbowStillOutsideShoulderSocket);
		float secondStageForearmShortener = remainingForearmLeeway * (1- prospectiveForearmFreedomFactorDueToBeingJammedByProximityToLowestElbowPoint);
		
		prospectiveApparentRightHumerusLength -= secondStageHumerusShortener;
		prospectiveApparentRightForearmLength -= secondStageForearmShortener;

		try { 
			if (rightHandCenterPos.y >= prospectiveRightShoulderSocketPos.y && rightHandCenterPos.x <= prospectiveRightShoulderSocketPos.x ) {
				prospectiveRightHumerusOrientation = calculateProximalLimbSegmentOrientation(prospectiveApparentRightHumerusLength, prospectiveApparentRightForearmLength, prospectiveDistanceBetweenRightShoulderAndRightHand, prospectiveRightShoulderToRightHandOrientation, false, false);
			} else {
				prospectiveRightHumerusOrientation = calculateProximalLimbSegmentOrientation(prospectiveApparentRightHumerusLength, prospectiveApparentRightForearmLength, prospectiveDistanceBetweenRightShoulderAndRightHand, prospectiveRightShoulderToRightHandOrientation, true, false);
			}
		} catch (LawOfCosinesException e) {
			
			rightArm.limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = true;
			
			throw new LimbMustMoveFirstException(LimbIdentifier.RIGHTARM);
		}
	}
	
	void updateProspectiveLeftThighOrientationBasedOnProspectiveTorsoMove() throws LimbMustMoveFirstException {
		
		@SuppressWarnings("unused")
		float prospectiveLeftThighOrientation;
		
		float prospectivePelvisToLeftAnkleOrientation = calculateLimbOrSegmentEndpointOrientation(prospectiveLeftHipSocketPos, leftAnklePos);
		float prospectiveDistanceBetweenPelvisAndLeftAnkle = CommonAlgorithms.findDistance(prospectiveLeftHipSocketPos, leftAnklePos);
		
		try {
			prospectiveLeftThighOrientation = calculateProximalLimbSegmentOrientation(femurLength, calfLength, prospectiveDistanceBetweenPelvisAndLeftAnkle, prospectivePelvisToLeftAnkleOrientation, false, true);
		} catch (LawOfCosinesException e) {
			
			leftLeg.limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = true;
			
			throw new LimbMustMoveFirstException(LimbIdentifier.LEFTLEG);
		}
	}
	
	void updateProspectiveRightThighOrientationBasedOnProspectiveTorsoMove() throws LimbMustMoveFirstException {
		
		@SuppressWarnings("unused")
		float prospectiveRightThighOrientation;
		
		float prospectivePelvisToRightAnkleOrientation = calculateLimbOrSegmentEndpointOrientation(prospectiveRightHipSocketPos, rightAnklePos);
		float prospectiveDistanceBetweenPelvisAndRightAnkle = CommonAlgorithms.findDistance(prospectiveRightHipSocketPos, rightAnklePos);
		
		try {
			prospectiveRightThighOrientation = calculateProximalLimbSegmentOrientation(femurLength, calfLength, prospectiveDistanceBetweenPelvisAndRightAnkle, prospectivePelvisToRightAnkleOrientation, false, false);
		} catch (LawOfCosinesException e) {
			
			rightLeg.limbHadToAbandonHoldBecauseOfAViolationOfReachableArea = true;
			
			throw new LimbMustMoveFirstException(LimbIdentifier.RIGHTLEG);
		}
	}
	
	//find wrist point from center of hand point and angle of hand compared to wrist
	Point findWristPointFromHandPoint (Point centerOfHand, float radialSlopeFromWristToHand) {
		return CommonAlgorithms.shiftBySlopeAsRadians(centerOfHand, handRadius, radialSlopeFromWristToHand, false);
	}
	
	Point findAnklePointFromFootPoint(Point footStandingPoint, boolean isLeftSide) {
		
		float anklePointDeltaX = ((footLength * Constants.RelativeDistanceFromHeelToBallOfFoot) - (footLength - footLengthHorizontalFromAnkleToToe));
		if (isLeftSide) {anklePointDeltaX = anklePointDeltaX * (-1);}
		
		float anklePointDeltaY = footHeight;
		
		return CommonAlgorithms.roundPoint((footStandingPoint.x - anklePointDeltaX),(footStandingPoint.y - anklePointDeltaY));
	}
	
	void updateHeadPoints(){
		neckBaseBetweenShoulderSocketsPos = CommonAlgorithms.findMidpoint(leftShoulderSocketPos, rightShoulderSocketPos);
		headBasePos.x = neckBaseBetweenShoulderSocketsPos.x;
		headBasePos.y = Math.round(neckBaseBetweenShoulderSocketsPos.y - distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle);
		headTopPos.x = headBasePos.x;
		headTopPos.y = Math.round(headBasePos.y - (2 * headRadius));
	}	
	
	void updateAllLimbWireframeValues() {
		
		updateLeftArmWireframeValues();
		updateRightArmWireframeValues();
		updateLeftLegWireframeValues();
		updateRightLegWireframeValues();
		updateCurrentHipBox();
		updateHeadPoints();
	}
	
	//IV. METHODS RELATED TO MOVEMENT
	void findMostEfficientPathUpTheWallThroughTheHoldPoints() {
		
		findMostEfficientPathFromBottomOfWallToTopOfWallThroughTheHoldPointsUsingQuadrantRadiusLimitForEachHoldInTheZoneAtTheTopOfTheWall();
		
		assemblePointsOfMostEfficientPathThroughTheHoldPointsFromBottomOfWallToTopOfWallUsingQuadrantRadiusLimit();
	}
		
	//Dijkstra's Algorithm
	void findMostEfficientPathFromBottomOfWallToTopOfWallThroughTheHoldPointsUsingQuadrantRadiusLimitForEachHoldInTheZoneAtTheTopOfTheWall() {
		
		for (int i = 0; i < finalizedSetOfUserEnteredHolds.size(); i++) {
			finalizedSetOfUserEnteredHolds.get(i).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections = Float.MAX_VALUE;
		}
		
		LinkedList<Integer> distanceAnalysisQueue = new LinkedList<Integer>();
		
		for (Integer i : finalizedSetOfUserEnteredHolds.holdsWithinBottomOfWallZone) {
			
			finalizedSetOfUserEnteredHolds.get(i).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections = 0;
			distanceAnalysisQueue.add(i);
		}
		
		while (!distanceAnalysisQueue.isEmpty()) {
			
			Integer indexOfHoldBeingAnalyzed = distanceAnalysisQueue.removeFirst();
			
			float distanceFromBottomOfWallToHoldBeingAnalyzed = finalizedSetOfUserEnteredHolds.get(indexOfHoldBeingAnalyzed).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections;
						
			for (int indexOfAdjacentHold : finalizedSetOfUserEnteredHolds.get(indexOfHoldBeingAnalyzed).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
				
				float interholdDistance = CommonAlgorithms.findDistance(finalizedSetOfUserEnteredHolds.get(indexOfHoldBeingAnalyzed), finalizedSetOfUserEnteredHolds.get(indexOfAdjacentHold));
				
				if ((distanceFromBottomOfWallToHoldBeingAnalyzed + interholdDistance) < finalizedSetOfUserEnteredHolds.get(indexOfAdjacentHold).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections) {
					
					finalizedSetOfUserEnteredHolds.get(indexOfAdjacentHold).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections = (distanceFromBottomOfWallToHoldBeingAnalyzed + interholdDistance);
					finalizedSetOfUserEnteredHolds.get(indexOfAdjacentHold).adjacentHoldThatBeginsShortestPathToBottomOfWallViaQuadrantSizeLimitsOnConnections = indexOfHoldBeingAnalyzed;
					distanceAnalysisQueue.add(indexOfAdjacentHold);
				}	
			}
		}
	}
	
	void assemblePointsOfMostEfficientPathThroughTheHoldPointsFromBottomOfWallToTopOfWallUsingQuadrantRadiusLimit() {
		
		shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections = new LinkedList<>();
		
		float shortestPathDistanceThusfar = Float.MAX_VALUE;
		Integer indexOfShortestPathFinishingHoldAtTopOfWall = -1;
		
		for (Integer i : finalizedSetOfUserEnteredHolds.holdsWithinTopOfWallZone) {
			
			float distanceFromHoldInTopOfWallZoneToBottomOfWall = finalizedSetOfUserEnteredHolds.get(i).shortestDistanceToABottomHoldViaQuadrantSizeLimitsOnConnections;
			
			if (distanceFromHoldInTopOfWallZoneToBottomOfWall < shortestPathDistanceThusfar) {
				
				shortestPathDistanceThusfar = distanceFromHoldInTopOfWallZoneToBottomOfWall;
				indexOfShortestPathFinishingHoldAtTopOfWall = i;
			}
		}
		
		shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.addFirst(indexOfShortestPathFinishingHoldAtTopOfWall);
			
		while (finalizedSetOfUserEnteredHolds.get(shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.peekFirst()).adjacentHoldThatBeginsShortestPathToBottomOfWallViaQuadrantSizeLimitsOnConnections != -1) {
			
			Integer indexOfImmediatelyPreceedingHoldAlongPath = finalizedSetOfUserEnteredHolds.get(shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.peekFirst()).adjacentHoldThatBeginsShortestPathToBottomOfWallViaQuadrantSizeLimitsOnConnections;
			shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.addFirst(indexOfImmediatelyPreceedingHoldAlongPath);
		}
	}
	
	void findThePathThroughEachSecondIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections() {
		
		LinkedList<Point> firstIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections = new LinkedList<>();
		
		for (int i = 0; i < shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.size(); i++) {
			
			Point milestoneAlongShortestPathThroughHoldPoints = finalizedSetOfUserEnteredHolds.get(shortestPathThroughHoldPointsFromBottomToTopBasedOnQuadrantConnections.get(i));
			
			Point averageFirstIterationPointOfTheHoldPointsWithinHalfTheReachOfBothArmsForPathMilestone = calculateAverageHoldPointWithinArmsReachOfAGivenPoint(milestoneAlongShortestPathThroughHoldPoints); 
			
			firstIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.add(averageFirstIterationPointOfTheHoldPointsWithinHalfTheReachOfBothArmsForPathMilestone);
		}
		
		for (int i = 0; i < firstIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.size(); i++) {
			
			Point milestoneAlongFirstIterationAveragePathThroughHoldPoints = firstIterationAveragePointOfHoldsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.get(i);
			
			Point averageSecondIterationPointOfTheHoldPointsWithinHalfTheReachOfBothArmsForPathMilestone = calculateAverageHoldPointWithinArmsReachOfAGivenPoint(milestoneAlongFirstIterationAveragePathThroughHoldPoints); 
			
			pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.add(averageSecondIterationPointOfTheHoldPointsWithinHalfTheReachOfBothArmsForPathMilestone);
		}
	}
	
	void makeExtremePointBookendsForThePathThroughEachSecondIterationAveragePointOfHoldsWithinMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections() {
		
		Point lowestPointExtended = new Point(pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.getFirst().x, ClimbingWall.heightOfWall + 1);
		Point highestPointExtended = new Point(pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.getLast().x, -1);
		
		pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.addFirst(lowestPointExtended);
		pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.addLast(highestPointExtended);
	}
	
	void updateDestinationPointForBeltBuckleUsingPathIterator() {
		
		iAmBacktracking = false;
		
		if (averagePathIndexIterator == null) {
			
			if (!iHaveLeftGroundAndConnectedToWallWithAllFourLimbs) {
				
				establishFirstPathIteratorIndex();
				
			} else {
				
				reestablishPathIteratorIndex();
				
			}
			
		} else if (beltBucklePos.x == desiredDestinationForBeltBuckle.x && beltBucklePos.y == desiredDestinationForBeltBuckle.y) {

			updateWhetherClimberHasReachedDestinationPointThatWasBeyondASeeminglyImpossiblePosition();
			
			moveIteratorAlongPath();
		}
		
		calculateDestinationPointForBeltBuckleBasedOnPathIterator();
	}
	
	void establishFirstPathIteratorIndex() {
		
		float closestDistance = Float.MAX_VALUE;
		float secondClosestDistance = Float.MAX_VALUE;
		
		Point closestPointOnPath = new Point();
		Point secondClosestPointOnPath = new Point();
		
		for (Point p : pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections) {
			
			float distanceFromBeltBuckle = CommonAlgorithms.findDistance(beltBucklePos, p);
			boolean holdIsAboveBeltBuckle = (beltBucklePos.y - p.y) > 0;
			
			if (holdIsAboveBeltBuckle && (distanceFromBeltBuckle < secondClosestDistance)) {
				
				if (distanceFromBeltBuckle <= closestDistance) {
					
					secondClosestDistance = closestDistance;
					secondClosestPointOnPath = closestPointOnPath;
					
					closestDistance = distanceFromBeltBuckle;
					closestPointOnPath = p;
				
				} else {
					
					secondClosestDistance = distanceFromBeltBuckle;
					secondClosestPointOnPath = p;
		
				}		
			}
		}
		
		Integer theIndexWithinLinkedListOfAveragePointsNearEachPointAlongShortestPathThroughHoldPontsThatIsFurtherAlongBetweenTheTwoClosestPointToBeltBuckle = Math.max(pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.indexOf(closestPointOnPath), pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.indexOf(secondClosestPointOnPath));
		
		averagePathIndexIterator = theIndexWithinLinkedListOfAveragePointsNearEachPointAlongShortestPathThroughHoldPontsThatIsFurtherAlongBetweenTheTwoClosestPointToBeltBuckle;
	}
	
	void reestablishPathIteratorIndex() {
		
		float closestDistance = Float.MAX_VALUE;
		float secondClosestDistance = Float.MAX_VALUE;
		
		Point closestPointOnPath = new Point();
		Point secondClosestPointOnPath = new Point();
		
		for (Point p : pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections) {
			
			float distanceFromBeltBuckle = CommonAlgorithms.findDistance(beltBucklePos, p);

			if (distanceFromBeltBuckle < secondClosestDistance) {
				
				if (distanceFromBeltBuckle <= closestDistance) {
					
					secondClosestDistance = closestDistance;
					secondClosestPointOnPath = closestPointOnPath;
					
					closestDistance = distanceFromBeltBuckle;
					closestPointOnPath = p;
				
				} else {
					
					secondClosestDistance = distanceFromBeltBuckle;
					secondClosestPointOnPath = p;
		
				}		
			}
		}
		
		Integer theIndexWithinLinkedListOfAveragePointsNearEachPointAlongShortestPathThroughHoldPontsThatIsFurtherAlongBetweenTheTwoClosestPointToBeltBuckle = Math.max(pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.indexOf(closestPointOnPath), pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.indexOf(secondClosestPointOnPath));
		
		averagePathIndexIterator = theIndexWithinLinkedListOfAveragePointsNearEachPointAlongShortestPathThroughHoldPontsThatIsFurtherAlongBetweenTheTwoClosestPointToBeltBuckle;
	}
	
	void updateWhetherClimberHasReachedDestinationPointThatWasBeyondASeeminglyImpossiblePosition() {
		
		if (averagePathIndexIterator >= indexOfAveragePathDestinationPointAtTheSeeminglyImpossiblePosition) {
			
			hasReachedDestinationPointThatWasBeyondSeeminglyImpossiblePosition = true;
		
		}		
	}
	
	void moveIteratorAlongPath() {
		
		averagePathIndexIterator++;
	}
	
	void calculateDestinationPointForBeltBuckleBasedOnPathIterator() {
		
		desiredDestinationForBeltBuckle = pathThroughEachSecondIterationAveragePointOfTheHoldPointsWithinHalfOfTheMaximumReachOfBothArmsOfEachHoldPointOnTheShortestPathFromTopToBottomBasedOnQuadrantConnections.get(averagePathIndexIterator);
	}
	
	void updateDesiredRadialDirectionOfTravel() {
		
		desiredRadialDirectionOfTravel = CommonAlgorithms.findSlopeAsRadians(beltBucklePos, desiredDestinationForBeltBuckle);
	}
	
	//provide a list of x and y coordinates (float formatted) of points on a line from start point to end point
	ArrayList<float[]> calculateWaypointFloatCoordinates(Point startingPoint, Point destinationPoint, float travelDistanceIncrement) {
		
		float distanceToTravel = CommonAlgorithms.findDistance(startingPoint, destinationPoint);
				
		int numberOfTicks = Math.round((distanceToTravel/(travelDistanceIncrement)));
		
		ArrayList<float[]> result = new ArrayList<float[]>();
		
		float[] coordinateChanges = floatIncrementalMovementAtRate(startingPoint, destinationPoint, travelDistanceIncrement);
		
		float xDelta = coordinateChanges[0];
		float yDelta = coordinateChanges[1];
		
		float waypointFloatCoordinateX = startingPoint.x;
		float waypointFloatCoordinateY = startingPoint.y;
		
		for (int i = 1; i <= numberOfTicks; i++) {
			
			waypointFloatCoordinateX += xDelta;
			waypointFloatCoordinateY += yDelta;
			
			float[] newWaypoint = {waypointFloatCoordinateX, waypointFloatCoordinateY};
			
			result.add(newWaypoint);
		}

		float[] finalPoint = {destinationPoint.x, destinationPoint.y};
		result.add(finalPoint);

		return result;
	}
	
	
	float[] floatIncrementalMovementAtRate (Point firstPoint, Point destinationPoint, float rate) {
			
		float[] coordinateChanges = singleUnitIncrementFromOnePointToAnother(firstPoint, destinationPoint);
		
		float changeInX = (coordinateChanges[0] * rate);
		float changeInY = (coordinateChanges[1] * rate);
		
		float[] result = {changeInX, changeInY};
	
		return result;
	}
	
	float[] singleUnitIncrementFromOnePointToAnother (Point firstPoint, Point destinationPoint) {
		
		float movementSlope = CommonAlgorithms.findSlopeAsRadians(firstPoint, destinationPoint);
		
		float xShift = (float) Math.cos(movementSlope);
		float yShift = (float) ((-1) * (Math.sin(movementSlope)));
				
		float[] result = {xShift, yShift};
		return result;
	}
	
	void updateCurrentlyReachableAreaAggregate() {
		
		limbsCoordinator.updateAllCurrentReachableAreas();
		
		currentReachableAreaAggregate = new Area();
		
		currentReachableAreaAggregate.add(leftArm.currentlyReachableArea);
		currentReachableAreaAggregate.add(rightArm.currentlyReachableArea);
		currentReachableAreaAggregate.add(leftLeg.currentlyReachableArea);
		currentReachableAreaAggregate.add(rightLeg.currentlyReachableArea);		
	}
	
	void updateLimbUsageRecordForReachableHolds() {
		
		if (iAmBacktracking) {
			
			updateCurrentlyReachableAreaAggregate();
			
			clearLimbUsageRecordForHoldsInCurrentReachableAreaAggregate();
			
		}		
	}
	
	void clearLimbUsageRecordForHoldsInCurrentReachableAreaAggregate() {
		
		for (ClimbingHold h : finalizedSetOfUserEnteredHolds) {
			
			if (currentReachableAreaAggregate.contains(h)) {
				
				h.limbsThatHaveUsedHoldDuringAscent = new HashSet<>();
				
			}
		}
	}
	
	Point calculateAverageHoldPointWithinArmsReachOfAGivenPoint(Point p) {
		
		Area circleWithRadiusOfHalfTheDistanceBetweenHandsWhenFullyOutstretched = calculateAreaWithinHalfTheMaximumReachOfArmsOfAGivenPoint(p);
		
		return CommonAlgorithms.averageThePointsInAnArea(finalizedSetOfUserEnteredHolds, circleWithRadiusOfHalfTheDistanceBetweenHandsWhenFullyOutstretched);		
	}
	
	Area calculateAreaWithinHalfTheMaximumReachOfArmsOfAGivenPoint(Point p) {
		
		Ellipse2D.Float circle = new Ellipse2D.Float(p.x - (widestPossibleExtensionFromHandToHand/2), p.y - (widestPossibleExtensionFromHandToHand/2), widestPossibleExtensionFromHandToHand, widestPossibleExtensionFromHandToHand);
		
		return new Area(circle);
	}
	
	//V. METHODS RELATED TO DRAWING
	
	void updateAllDrawingPoints () {
		updateLeftArmDrawingPoints();
		updateRightArmDrawingPoints();
		updateLeftLegDrawingPoints();
		updateRightLegDrawingPoints();
		updateHeadDrawingPoints();
	}
	
	void updateLeftArmDrawingPoints() {
		leftShoulderDrawingPoint = calculateCircleDrawingPoint(leftShoulderSocketPos, upperArmWidth/2);
		leftElbowDrawingPoint = calculateCircleDrawingPoint(leftElbowPos, elbowWidth/2);
		leftHandDrawingPoint = calculateCircleDrawingPoint(leftHandCenterPos, handRadius);
		
		lateralProximalLeftHumerus = shiftByOrientationPerpendicularly(leftShoulderSocketPos, upperArmWidth, leftHumerusOrientation, false);
		medialProximalLeftHumerus = shiftByOrientationPerpendicularly(leftShoulderSocketPos, upperArmWidth, leftHumerusOrientation, true);
		lateralDistalLeftHumerus = shiftByOrientationPerpendicularly(leftElbowPos, elbowWidth, leftHumerusOrientation, false);
		medialDistalLeftHumerus = shiftByOrientationPerpendicularly(leftElbowPos, elbowWidth, leftHumerusOrientation, true);
		
		lateralProximalLeftForearm = shiftByOrientationPerpendicularly(leftElbowPos, elbowWidth, leftForearmOrientation, false);
		medialProximalLeftForearm = shiftByOrientationPerpendicularly(leftElbowPos, elbowWidth, leftForearmOrientation, true);
		lateralDistalLeftForearm = shiftByOrientationPerpendicularly(leftWristPos, wristWidth, leftForearmOrientation, false);
		medialDistalLeftForearm = shiftByOrientationPerpendicularly(leftWristPos, wristWidth, leftForearmOrientation, true);	
	}
	
	void updateRightArmDrawingPoints() {
		
		rightShoulderDrawingPoint = calculateCircleDrawingPoint(rightShoulderSocketPos, upperArmWidth/2);
		rightElbowDrawingPoint = calculateCircleDrawingPoint(rightElbowPos, elbowWidth/2);
		rightHandDrawingPoint = calculateCircleDrawingPoint(rightHandCenterPos, handRadius);
	
		lateralProximalRightHumerus = shiftByOrientationPerpendicularly(rightShoulderSocketPos, upperArmWidth, rightHumerusOrientation, true);
		medialProximalRightHumerus = shiftByOrientationPerpendicularly(rightShoulderSocketPos, upperArmWidth, rightHumerusOrientation, false);
		lateralDistalRightHumerus = shiftByOrientationPerpendicularly(rightElbowPos, elbowWidth, rightHumerusOrientation, true);
		medialDistalRightHumerus = shiftByOrientationPerpendicularly(rightElbowPos, elbowWidth, rightHumerusOrientation, false);
		
		lateralProximalRightForearm = shiftByOrientationPerpendicularly(rightElbowPos, elbowWidth, rightForearmOrientation, true);
		medialProximalRightForearm = shiftByOrientationPerpendicularly(rightElbowPos, elbowWidth, rightForearmOrientation, false);
		lateralDistalRightForearm = shiftByOrientationPerpendicularly(rightWristPos, wristWidth, rightForearmOrientation, true);
		medialDistalRightForearm = shiftByOrientationPerpendicularly(rightWristPos, wristWidth, rightForearmOrientation, false);
	}
	
	void updateLeftLegDrawingPoints() {
				
		leftFootDrawingPoint = calculateFootDrawingPoint(leftAnklePos, true);
		leftHipSocketDrawingPoint = calculateCircleDrawingPoint(leftHipSocketPos, upperThighWidth/2);
		leftKneeDrawingPoint = calculateCircleDrawingPoint(leftKneePos, kneeWidth/2);
		
		lateralProximalLeftThigh = shiftByOrientationPerpendicularly(leftHipSocketPos, upperThighWidth, leftThighOrientation, false);
		medialProximalLeftThigh = shiftByOrientationPerpendicularly(leftHipSocketPos, upperThighWidth, leftThighOrientation, true);
		lateralDistalLeftThigh = shiftByOrientationPerpendicularly(leftKneePos, kneeWidth, leftThighOrientation, false);
		medialDistalLeftThigh = shiftByOrientationPerpendicularly(leftKneePos, kneeWidth, leftThighOrientation, true);
		
		lateralProximalLeftCalf = shiftByOrientationPerpendicularly(leftKneePos, kneeWidth, leftCalfOrientation, false);
		medialProximalLeftCalf = shiftByOrientationPerpendicularly(leftKneePos, kneeWidth, leftCalfOrientation, true);
		lateralDistalLeftCalf = shiftByOrientationPerpendicularly(leftAnklePos, ankleWidth, leftCalfOrientation, false);
		medialDistalLeftCalf = shiftByOrientationPerpendicularly(leftAnklePos, ankleWidth, leftCalfOrientation, true);
		
	}
	
	void updateRightLegDrawingPoints() {
		
		rightFootDrawingPoint = calculateFootDrawingPoint(rightAnklePos, false);
		rightHipSocketDrawingPoint = calculateCircleDrawingPoint(rightHipSocketPos, upperThighWidth/2);
		rightKneeDrawingPoint = calculateCircleDrawingPoint(rightKneePos, kneeWidth/2);
		
		lateralProximalRightThigh = shiftByOrientationPerpendicularly(rightHipSocketPos, upperThighWidth, rightThighOrientation, true);
		medialProximalRightThigh = shiftByOrientationPerpendicularly(rightHipSocketPos, upperThighWidth, rightThighOrientation, false);
		lateralDistalRightThigh = shiftByOrientationPerpendicularly(rightKneePos, kneeWidth, rightThighOrientation, true);
		medialDistalRightThigh = shiftByOrientationPerpendicularly(rightKneePos, kneeWidth, rightThighOrientation, false);
		
		lateralProximalRightCalf = shiftByOrientationPerpendicularly(rightKneePos, kneeWidth, rightCalfOrientation, true);
		medialProximalRightCalf = shiftByOrientationPerpendicularly(rightKneePos, kneeWidth, rightCalfOrientation, false);
		lateralDistalRightCalf = shiftByOrientationPerpendicularly(rightAnklePos, ankleWidth, rightCalfOrientation, true);
		medialDistalRightCalf = shiftByOrientationPerpendicularly(rightAnklePos, ankleWidth, rightCalfOrientation, false);
		
	}
	
	void updateHeadDrawingPoints(){
		headVisualCenter = CommonAlgorithms.findMidpoint(headBasePos, headTopPos);
		headDrawingPoint = calculateCircleDrawingPoint(headVisualCenter, headRadius);
		neckDrawingPoint.x = Math.round(neckBaseBetweenShoulderSocketsPos.x -(neckThickness/2));
		neckDrawingPoint.y = Math.round(neckBaseBetweenShoulderSocketsPos.y - (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle + (headRadius * Constants.FactorForDeterminingAdditionalVerticalDistanceOfNeckDrawingAreaDeterminedAsAFractionOfHeadRadius)));		
	}

	//determine point that is a particular distance away from a body segment point along a ray that is perpendicular to the body segment's orientation, either clockwise or counterclockwise
	Point shiftByOrientationPerpendicularly(Point startingPoint, float bodySegmentWidth, float bodySegmentOrientation, boolean counterclockwise) {
		
		float ninetyDegreesInProperRotationalDirection = Pi/2;
		
		if (!counterclockwise) {
			
			ninetyDegreesInProperRotationalDirection = ninetyDegreesInProperRotationalDirection * (-1);
		
		}
		
		float halfWidth = bodySegmentWidth/2;
		
		return CommonAlgorithms.shiftPointByShiftedOrientation(startingPoint, halfWidth, bodySegmentOrientation, ninetyDegreesInProperRotationalDirection);
	
	}
	
	Point calculateCircleDrawingPoint(Point circleCenter, float circleRadius) {
		
		return CommonAlgorithms.roundPoint((circleCenter.x - circleRadius), (circleCenter.y - circleRadius));
		
	}
	
	Point calculateFootDrawingPoint(Point anklePoint, boolean leftSide) {
		
		float adjuster = footLengthHorizontalFromAnkleToToe;
		
		if (!leftSide) {
		
			adjuster = (footLength - adjuster);
		
		}
		
		return CommonAlgorithms.roundPoint((anklePoint.x - adjuster), anklePoint.y);
	
	}
	
	void pauseForAnimationTiming() {
		
		while (!wallHasDrawnTheLatestClimberMoves) {
			try {
				Thread.sleep(Constants.MovementPromptPauseLength);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		
		wallHasDrawnTheLatestClimberMoves = false;
	}
	
	void drawClimber(Graphics2D g2d) {
		
		Area upperBodyLimbsArea;
		Area lowerBodyArea;
		Area shirtTorsoArea;
		Area leftFoot;
		Area rightFoot;
		
		Path2D.Double leftHumerus;
		Path2D.Double rightHumerus;
		Path2D.Double leftForearm;
		Path2D.Double rightForearm;
		
		Path2D.Double pantsPelvis;
		Path2D.Double leftThigh;
		Path2D.Double rightThigh;
		Path2D.Double leftCalf;
		Path2D.Double rightCalf;
		
		Path2D.Double torsoShirt;
		
		//paint hands
		g2d.setPaint(Constants.SkinTone);
		g2d.fill(new Ellipse2D.Float(leftHandDrawingPoint.x,leftHandDrawingPoint.y,2 * handRadius, 2 * handRadius));
		g2d.fill(new Ellipse2D.Float(rightHandDrawingPoint.x,rightHandDrawingPoint.y,2 * handRadius,2 * handRadius));
		
		if(!leftArm.isConnectedToSomething) {
			Point thumbTip = CommonAlgorithms.shiftBySlopeAsRadians(leftWristPos, thumbLength, (float) (leftWristPivot - Pi/3.8), true);
			Point indexTip = CommonAlgorithms.shiftBySlopeAsRadians(leftWristPos, (float) (fingerLength * 0.95), leftWristPivot - Pi/15, true);
			Point middleTip = CommonAlgorithms.shiftBySlopeAsRadians(leftWristPos, fingerLength, leftWristPivot, true);
			Point ringTip = CommonAlgorithms.shiftBySlopeAsRadians(leftWristPos, (float) (fingerLength * 0.95), leftWristPivot + Pi/18, true);
			Point pinkieTip = CommonAlgorithms.shiftBySlopeAsRadians(leftWristPos, (float) (fingerLength * 0.85), leftWristPivot + Pi/8, true);
			
			g2d.setStroke(new BasicStroke(thumbWidth));
			g2d.drawLine(leftWristPos.x, leftWristPos.y, thumbTip.x, thumbTip.y);
			
			g2d.setStroke(new BasicStroke(fingerWidth));
			g2d.drawLine(leftWristPos.x, leftWristPos.y, indexTip.x, indexTip.y);
			g2d.drawLine(leftWristPos.x, leftWristPos.y, middleTip.x, middleTip.y);
			g2d.drawLine(leftWristPos.x, leftWristPos.y, ringTip.x, ringTip.y);
			g2d.drawLine(leftWristPos.x, leftWristPos.y, pinkieTip.x, pinkieTip.y);
		}
		
		if(!rightArm.isConnectedToSomething) {
			Point thumbTip = CommonAlgorithms.shiftBySlopeAsRadians(rightWristPos, thumbLength, (float) (rightWristPivot + Pi/3.8), true);
			Point indexTip = CommonAlgorithms.shiftBySlopeAsRadians(rightWristPos, (float) (fingerLength * 0.95), rightWristPivot + Pi/15, true);
			Point middleTip = CommonAlgorithms.shiftBySlopeAsRadians(rightWristPos, fingerLength, rightWristPivot, true);
			Point ringTip = CommonAlgorithms.shiftBySlopeAsRadians(rightWristPos, (float) (fingerLength * 0.95), rightWristPivot - Pi/18, true);
			Point pinkieTip = CommonAlgorithms.shiftBySlopeAsRadians(rightWristPos, (float) (fingerLength * 0.85), rightWristPivot - Pi/8, true);
			
			g2d.setStroke(new BasicStroke(thumbWidth));
			g2d.drawLine(rightWristPos.x, rightWristPos.y, thumbTip.x, thumbTip.y);
			
			g2d.setStroke(new BasicStroke(fingerWidth));
			g2d.drawLine(rightWristPos.x, rightWristPos.y, indexTip.x, indexTip.y);
			g2d.drawLine(rightWristPos.x, rightWristPos.y, middleTip.x, middleTip.y);
			g2d.drawLine(rightWristPos.x, rightWristPos.y, ringTip.x, ringTip.y);
			g2d.drawLine(rightWristPos.x, rightWristPos.y, pinkieTip.x, pinkieTip.y);
		}
		
		g2d.setStroke(new BasicStroke());
		
		//paint shoes
		leftFoot = new Area(new Rectangle2D.Float(leftFootDrawingPoint.x, leftFootDrawingPoint.y, footLength, footHeight));
		leftFoot.add(new Area(new Ellipse2D.Float(Math.round(leftFootDrawingPoint.x - (footHeight/2)), leftFootDrawingPoint.y, footHeight, footHeight)));
		
		rightFoot = new Area(new Rectangle2D.Float(rightFootDrawingPoint.x, rightFootDrawingPoint.y, footLength, footHeight));
		rightFoot.add(new Area(new Ellipse2D.Float(Math.round(rightFootDrawingPoint.x + footLength - (footHeight/2)), rightFootDrawingPoint.y, footHeight, footHeight)));
			
		g2d.setPaint(Color.BLACK);
		g2d.fill(leftFoot);
		g2d.fill(rightFoot);
		
		//paint rope
		g2d.setPaint(Constants.RopeColor);
		g2d.setStroke(new BasicStroke(Constants.RopeDiameter * Constants.SizeFactor));
		g2d.drawLine(anchorPoint.x, anchorPoint.y, beltBucklePos.x, beltBucklePos.y);
		
		g2d.setStroke(new BasicStroke());

		//paint shirt sleeves
		upperBodyLimbsArea = new Area();
		
		leftHumerus = new Path2D.Double();
		leftHumerus.moveTo(lateralProximalLeftHumerus.x, lateralProximalLeftHumerus.y);
		leftHumerus.lineTo(lateralDistalLeftHumerus.x, lateralDistalLeftHumerus.y);
		leftHumerus.lineTo(medialDistalLeftHumerus.x, medialDistalLeftHumerus.y);
		leftHumerus.lineTo(medialProximalLeftHumerus.x, medialProximalLeftHumerus.y);
		leftHumerus.closePath();
		Area leftHumerusArea = new Area(leftHumerus);
		upperBodyLimbsArea.add(leftHumerusArea);
		
		rightHumerus = new Path2D.Double();
		rightHumerus.moveTo(lateralProximalRightHumerus.x, lateralProximalRightHumerus.y);
		rightHumerus.lineTo(lateralDistalRightHumerus.x, lateralDistalRightHumerus.y);
		rightHumerus.lineTo(medialDistalRightHumerus.x, medialDistalRightHumerus.y);
		rightHumerus.lineTo(medialProximalRightHumerus.x, medialProximalRightHumerus.y);
		rightHumerus.closePath();
		Area rightHumerusArea = new Area(rightHumerus);
		upperBodyLimbsArea.add(rightHumerusArea);
		
		leftForearm = new Path2D.Double();
		leftForearm.moveTo(lateralProximalLeftForearm.x, lateralProximalLeftForearm.y);
		leftForearm.lineTo(lateralDistalLeftForearm.x, lateralDistalLeftForearm.y);
		leftForearm.lineTo(medialDistalLeftForearm.x, medialDistalLeftForearm.y);
		leftForearm.lineTo(medialProximalLeftForearm.x, medialProximalLeftForearm.y);
		leftForearm.closePath();
		Area leftForearmArea = new Area(leftForearm);
		upperBodyLimbsArea.add(leftForearmArea);
		
		rightForearm = new Path2D.Double();
		rightForearm.moveTo(lateralProximalRightForearm.x, lateralProximalRightForearm.y);
		rightForearm.lineTo(lateralDistalRightForearm.x, lateralDistalRightForearm.y);
		rightForearm.lineTo(medialDistalRightForearm.x, medialDistalRightForearm.y);
		rightForearm.lineTo(medialProximalRightForearm.x, medialProximalRightForearm.y);
		rightForearm.closePath();
		Area rightForearmArea = new Area(rightForearm);
		upperBodyLimbsArea.add(rightForearmArea);
		
		upperBodyLimbsArea.add(new Area(new Ellipse2D.Float(leftShoulderDrawingPoint.x, leftShoulderDrawingPoint.y, upperArmWidth, upperArmWidth)));
		upperBodyLimbsArea.add(new Area(new Ellipse2D.Float(rightShoulderDrawingPoint.x, rightShoulderDrawingPoint.y, upperArmWidth, upperArmWidth)));
		upperBodyLimbsArea.add(new Area(new Ellipse2D.Float(leftElbowDrawingPoint.x, leftElbowDrawingPoint.y, elbowWidth, elbowWidth)));
		upperBodyLimbsArea.add(new Area(new Ellipse2D.Float(rightElbowDrawingPoint.x, rightElbowDrawingPoint.y, elbowWidth, elbowWidth)));
		
		g2d.setPaint(Constants.ShirtColor);
		g2d.fill(upperBodyLimbsArea);
				
		//paint pants
		lowerBodyArea = new Area();
		
		pantsPelvis = new Path2D.Double();
		pantsPelvis.moveTo(leftWaistDrawingPoint.x, leftWaistDrawingPoint.y);
		pantsPelvis.lineTo(leftOuterPelvisHinge.x, leftOuterPelvisHinge.y);
		pantsPelvis.lineTo(leftInnerPelvisHinge.x, leftInnerPelvisHinge.y);
		pantsPelvis.lineTo(inseam.x, inseam.y);
		pantsPelvis.lineTo(rightInnerPelvisHinge.x, rightInnerPelvisHinge.y);
		pantsPelvis.lineTo(rightOuterPelvisHinge.x, rightOuterPelvisHinge.y);
		pantsPelvis.lineTo(rightWaistDrawingPoint.x, rightWaistDrawingPoint.y);
		pantsPelvis.closePath();
		Area pantsPelvisArea = new Area(pantsPelvis);
		lowerBodyArea.add(pantsPelvisArea);
		
		leftThigh = new Path2D.Double();
		leftThigh.moveTo(lateralProximalLeftThigh.x, lateralProximalLeftThigh.y);
		leftThigh.lineTo(lateralDistalLeftThigh.x, lateralDistalLeftThigh.y);
		leftThigh.lineTo(medialDistalLeftThigh.x, medialDistalLeftThigh.y);
		leftThigh.lineTo(medialProximalLeftThigh.x, medialProximalLeftThigh.y);
		leftThigh.closePath();
		Area leftThighArea = new Area(leftThigh);
		lowerBodyArea.add(leftThighArea);
		
		rightThigh = new Path2D.Double();
		rightThigh.moveTo(lateralProximalRightThigh.x, lateralProximalRightThigh.y);
		rightThigh.lineTo(lateralDistalRightThigh.x, lateralDistalRightThigh.y);
		rightThigh.lineTo(medialDistalRightThigh.x, medialDistalRightThigh.y);
		rightThigh.lineTo(medialProximalRightThigh.x, medialProximalRightThigh.y);
		rightThigh.closePath();
		Area rightThighArea = new Area(rightThigh);
		lowerBodyArea.add(rightThighArea);
		
		leftCalf = new Path2D.Double();
		leftCalf.moveTo(lateralProximalLeftCalf.x, lateralProximalLeftCalf.y);
		leftCalf.lineTo(lateralDistalLeftCalf.x, lateralDistalLeftCalf.y);
		leftCalf.lineTo(medialDistalLeftCalf.x, medialDistalLeftCalf.y);
		leftCalf.lineTo(medialProximalLeftCalf.x, medialProximalLeftCalf.y);
		leftCalf.closePath();
		Area leftCalfArea = new Area(leftCalf);
		lowerBodyArea.add(leftCalfArea);
		
		rightCalf = new Path2D.Double();
		rightCalf.moveTo(lateralProximalRightCalf.x, lateralProximalRightCalf.y);
		rightCalf.lineTo(lateralDistalRightCalf.x, lateralDistalRightCalf.y);
		rightCalf.lineTo(medialDistalRightCalf.x, medialDistalRightCalf.y);
		rightCalf.lineTo(medialProximalRightCalf.x, medialProximalRightCalf.y);
		rightCalf.closePath();
		Area rightCalfArea = new Area(rightCalf);
		lowerBodyArea.add(rightCalfArea);
		
		lowerBodyArea.add(new Area(new Ellipse2D.Float(leftHipSocketDrawingPoint.x, leftHipSocketDrawingPoint.y, upperThighWidth, upperThighWidth)));
		lowerBodyArea.add(new Area(new Ellipse2D.Float(rightHipSocketDrawingPoint.x, rightHipSocketDrawingPoint.y, upperThighWidth, upperThighWidth)));
		lowerBodyArea.add(new Area(new Ellipse2D.Float(leftKneeDrawingPoint.x, leftKneeDrawingPoint.y, kneeWidth, kneeWidth)));
		lowerBodyArea.add(new Area(new Ellipse2D.Float(rightKneeDrawingPoint.x, rightKneeDrawingPoint.y, kneeWidth, kneeWidth)));

		g2d.setPaint(Constants.PantsColor);
		g2d.fill(lowerBodyArea);
		
		//paint neck
		g2d.setPaint(Constants.SkinTone);
		g2d.fill(new Rectangle2D.Float(neckDrawingPoint.x, neckDrawingPoint.y, neckThickness, (distanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle + (headRadius * Constants.FactorForDeterminingAdditionalVerticalDistanceOfNeckDrawingAreaDeterminedAsAFractionOfHeadRadius))));
		
		//paint head
		g2d.setPaint(Constants.HairColor);
		g2d.fill(new Ellipse2D.Float(headDrawingPoint.x,headDrawingPoint.y,(2 * headRadius),(2 * headRadius)));
	
		//paint torso element of shirt
		torsoShirt = new Path2D.Double();
		torsoShirt.moveTo(Math.round(leftShoulderSocketPos.x - upperArmWidth/2), leftShoulderSocketPos.y);
		torsoShirt.lineTo(leftLatTorsoInsertionPoint.x, leftLatTorsoInsertionPoint.y);
		torsoShirt.lineTo(leftWaistDrawingPoint.x, leftWaistDrawingPoint.y);
		torsoShirt.lineTo(rightWaistDrawingPoint.x, rightWaistDrawingPoint.y);
		torsoShirt.lineTo(rightLatTorsoInsertionPoint.x, rightLatTorsoInsertionPoint.y);
		torsoShirt.lineTo(Math.round(rightShoulderSocketPos.x + upperArmWidth/2), rightShoulderSocketPos.y);
		torsoShirt.lineTo(rightTrapNeckIntersectionPoint.x, rightTrapNeckIntersectionPoint.y);
		torsoShirt.lineTo(leftTrapNeckIntersectionPoint.x, leftTrapNeckIntersectionPoint.y);
		torsoShirt.closePath();
		shirtTorsoArea = new Area(torsoShirt);
	
		g2d.setPaint(Constants.ShirtColor);
		g2d.fill(shirtTorsoArea);

	}
}