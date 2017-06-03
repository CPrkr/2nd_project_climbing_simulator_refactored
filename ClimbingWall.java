package com.craigcode.climbing_simulator_refactored;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.Insets;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Area;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Random;

import javax.swing.JPanel;

import com.craigcode.climbing_simulator_refactored.ClimbingWall.SetOfHolds;

class ClimbingWall extends JPanel {
	
	private static final long serialVersionUID = 1L;
	
	private static float Pi = Constants.Pi;
	
	public static int heightOfWall;
	
	static Climber wallClimber;
	
	SetOfHolds<ClimbingHold> setOfIntroductionHolds;
	SetOfHolds<ClimbingHold> setOfUserEnteredHolds;
	
	private boolean displayingIntroduction;
	private boolean requestForAssistanceVisible;
	private boolean startButtonVisible;
	
	private boolean showingInitialInstructions;
	
	private boolean holdsGoodIndividually;
	private boolean holdsGoodAsANetwork;
	private boolean showingWebOfConnections;
	
	private boolean displayingFeedback;
	private boolean tryingToAddHolds;
	private boolean waitingToBeFinalized;
	private boolean wallFinalized;
	private boolean showingClimber;
	private boolean showingAcceptanceOfDefeat;
	private boolean showingFinalMessageOfAppreciation;
	
	ClimbingWall() {
		
		establishWallHeight();

		wallClimber = new Climber();
		
		setOfUserEnteredHolds = new SetOfHolds<ClimbingHold>();
		
		setBackground(Constants.WallColor);
		
		initializeWall();
		
		addMouseListener (new MouseAdapter () {
			
			@Override
			public void mousePressed (MouseEvent e) {

				analyzeMousePress(e);
				
			}
		});
		
		addKeyListener (new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {
			}
			
			@Override
			public void keyReleased(KeyEvent e) {
				
				analyzeKeyRelease(e);
				
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
			}
		});
		
		runIntroduction();
	}
	
	void establishWallHeight() {
		
		Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
			
		Insets insets = Toolkit.getDefaultToolkit().getScreenInsets(GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice().getDefaultConfiguration());
		
		heightOfWall = (int) (screenSize.getHeight() - (50 + insets.bottom));

	}

	void initializeWall() {
		
		holdsGoodIndividually = false;
		holdsGoodAsANetwork = false;
		showingWebOfConnections = false;
		
		displayingFeedback = false;
		waitingToBeFinalized = false;
		wallFinalized = false;
		showingClimber = false;
		showingAcceptanceOfDefeat = false;
		showingFinalMessageOfAppreciation = false;
		
	}
	
	void analyzeMousePress(MouseEvent e) {
		
		if (displayingIntroduction) {
			
			possiblyBeginRouteSetting(e);
			
		} else {
		
			showingInitialInstructions = false;
			
			tryingToAddHolds = true;
			resetWallOrUpdateStateOfHolds(e);
			repaint();
		
		}		
	}
	
	void analyzeKeyRelease(KeyEvent e) {
		
		tryingToAddHolds = false;
		updateStateOfHolds(e);
		updateClimberOnStateOfWall();
		repaint();		
	}
	
	void possiblyBeginRouteSetting(MouseEvent e) {
		
		if (userWantsToBeginRouteSetting(e)) {
		
			displayingIntroduction = false;
			
			showingInitialInstructions = true;
			
			repaint();

		}
	}
		
	boolean userWantsToBeginRouteSetting(MouseEvent e) {
		
		int xValueForMouseClick = e.getX();
		int yValueForMouseClick = e.getY();
		
		if (xValueForMouseClick < Constants.XValueForStartButtonLeftBorder) {
			return false;
		}
		
		if (xValueForMouseClick > Constants.XValueForStartButtonLeftBorder + Constants.WidthOfStartButton) {
			return false;
		}
		
		if (yValueForMouseClick < Constants.YValueForStartButtonUpperBorder) {
			return false;
		}
		
		if (yValueForMouseClick > Constants.YValueForStartButtonUpperBorder + Constants.HeightOfStartButton) {
			return false;
		}
		
		return true;
		
	}
	
	void resetWallOrUpdateStateOfHolds(MouseEvent e) {
		
		if (userIsRequestingAWallReset(e)) {
			
			resetWall();
			
			return;
		}
		
		if (!wallFinalized) {
			
			if(e.getButton() == 1) {
				
				setOfUserEnteredHolds.addHold(e);
				
			} else {
				
				setOfUserEnteredHolds.addRandomSprayOfHolds(e);
			}
		}
		
		if (waitingToBeFinalized) {
			
			waitingToBeFinalized = false;
		}
	}
	
	boolean userIsRequestingAWallReset(MouseEvent e) {
		
		int xValueForMouseClick = e.getX();
		int yValueForMouseClick = e.getY();
		
		if (xValueForMouseClick < Constants.XValueForResetButtonLeftBorder) {
			return false;
		}
		
		if (xValueForMouseClick > Constants.XValueForResetButtonLeftBorder + Constants.WidthOfResetButton) {
			return false;
		}
		
		if (yValueForMouseClick < Constants.YValueForResetButtonUpperBorder) {
			return false;
		}
		
		if (yValueForMouseClick > Constants.YValueForResetButtonUpperBorder + Constants.HeightOfResetButton) {
			return false;
		}
		
		return true;
		
	}
	
	void resetWall() {
		
		wallClimber = new Climber();
		
		setOfUserEnteredHolds = new SetOfHolds<ClimbingHold>();
		
		showingInitialInstructions = true;
		
		try {
			Thread.sleep((1000/Constants.DisplayTickRate) + 1);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		
		initializeWall();
		
	}
	
	void updateStateOfHolds(KeyEvent e) {
		
		setOfUserEnteredHolds.tooClose = false;
		
		if (waitingToBeFinalized  && !wallFinalized) {
			
			lockInFinalStateOfWall();
		}
				
		if (!wallFinalized) {
			
			seeIfHoldsAreSpacedProperly();
		}
	}
		
	void updateClimberOnStateOfWall() {
		
		if (wallFinalized && !wallClimber.isClimberOnTheWall()) {
			
			notifyClimberThatWallIsReady();
		}
	}
	
	void runIntroduction() {
		
		displayingIntroduction = true;
				
		randomlyGenerateSetOfIntroductionHolds();
		
		(new Thread() {public void run() {
			
			int animationLoop = 0;
			
			int indexOfHoldToMakeVisible = 0;
			int numberOfTicksPerHoldVisibilityChange = (int) (Constants.DisplayTickRate/Constants.numberOfHoldsToDisplayPerSecondOnIntroductionScreen);
			
			while(displayingIntroduction) {						
				
				if (animationLoop%numberOfTicksPerHoldVisibilityChange == 0) {
					
					setOfIntroductionHolds.get(indexOfHoldToMakeVisible).isVisibleInIntroductionAnimation = true;
					
					indexOfHoldToMakeVisible++;
					
				}
				
				if (!requestForAssistanceVisible && (animationLoop > (Constants.DisplayTickRate * 3))) {
					
					requestForAssistanceVisible = true;
					
				}
				
				if (!startButtonVisible && (animationLoop > (Constants.DisplayTickRate * 4))) {
					
					startButtonVisible = true;
					
				}
				
				animationLoop++;
				
				try {
					Thread.sleep(1000/Constants.DisplayTickRate);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
				repaint();

			}
			
			repaint();
			
		}}).start();		
	}
	
	void randomlyGenerateSetOfIntroductionHolds() {
		
		setOfIntroductionHolds = new SetOfHolds<ClimbingHold>();
		
		Random rand = new Random();
		
		float radianSlopeOfSegment;
		
		Point currentWholeSegmentStartingPoint = new Point();
		float lengthOfCurrentWholeSegment;
		Point currentWholeSegmentEndingPoint = new Point();
		
		float lengthOfCurrentPartialSegment;
		Point currentPartialSegmentEndingPoint = new Point();
		
		float lengthOfPerpendicularSegment;
		
		currentWholeSegmentStartingPoint.x = Math.round(Constants.WidthOfBufferZoneForRandomHoldGeneration + (rand.nextFloat() * (Constants.WallWidth - (2 * Constants.WidthOfBufferZoneForRandomHoldGeneration)))); 
		currentWholeSegmentStartingPoint.y = heightOfWall;
		
		while (currentWholeSegmentStartingPoint.y > 0) {
			
			if (currentWholeSegmentStartingPoint.x < Constants.WidthOfBufferZoneForRandomHoldGeneration) {
				
				radianSlopeOfSegment = (Pi/2) - (rand.nextFloat() * Constants.RadianLimitOfCentralDirectionforRandomHoldGeneration);
				
			} else if (currentWholeSegmentStartingPoint.x > (Constants.WallWidth - Constants.WidthOfBufferZoneForRandomHoldGeneration)) {
			
				radianSlopeOfSegment = (Pi/2) + (rand.nextFloat() * Constants.RadianLimitOfCentralDirectionforRandomHoldGeneration);
				
			} else {	
			
				radianSlopeOfSegment = (Pi/2 + Constants.RadianLimitOfCentralDirectionforRandomHoldGeneration) - (rand.nextFloat() * (2 * Constants.RadianLimitOfCentralDirectionforRandomHoldGeneration)); 
			
			}	
				
			lengthOfCurrentWholeSegment = Constants.DistanceLimitOfWholeSegmentOfCentralDirectionForRandomHoldGeneration * rand.nextFloat();
			
			currentWholeSegmentEndingPoint = CommonAlgorithms.shiftBySlopeAsRadians(currentWholeSegmentStartingPoint, lengthOfCurrentWholeSegment, radianSlopeOfSegment, true);
			
			lengthOfCurrentPartialSegment = 0.0f;
			
			while (lengthOfCurrentPartialSegment < lengthOfCurrentWholeSegment) {
				
				lengthOfCurrentPartialSegment += rand.nextFloat() * Constants.DistanceLimitOfPartialSegmentOfCentralDirectionForRandomHoldGeneration;
				
				currentPartialSegmentEndingPoint = CommonAlgorithms.shiftBySlopeAsRadians(currentWholeSegmentStartingPoint, lengthOfCurrentPartialSegment, radianSlopeOfSegment, true);
				
				lengthOfPerpendicularSegment = rand.nextFloat() * Constants.DistanceLimitOfSegmentUsedToPlaceHoldPerpendicularToCentralDirectionForRandomHoldGeneration;
											
				float ninetyDegreesInRandomRotationalDirection;
				
				if (rand.nextBoolean()) {
					
					ninetyDegreesInRandomRotationalDirection = (Pi/2);
				
				} else {
					
					ninetyDegreesInRandomRotationalDirection = (Pi/2) * (-1.0f);
					
				}
				
				Point pointPerpendicularToRandomWholeSegment = CommonAlgorithms.shiftPointByShiftedOrientation (currentPartialSegmentEndingPoint, lengthOfPerpendicularSegment, radianSlopeOfSegment, ninetyDegreesInRandomRotationalDirection);
				
				ClimbingHold holdPerpendicularToRandomWholeSegment = new ClimbingHold(pointPerpendicularToRandomWholeSegment.x, pointPerpendicularToRandomWholeSegment.y);
				
				setOfIntroductionHolds.add(holdPerpendicularToRandomWholeSegment);
				
			}
			
			currentWholeSegmentStartingPoint = currentWholeSegmentEndingPoint;
		}
	}
	
	void receiveAndProcessNoticeThatClimberIsBeginningClimbingProcess() {
						
		showClimberMovingOnUserEnteredHolds();
	}
		
	class SetOfHolds<T extends ClimbingHold> extends ArrayList<T> {
		
		private static final long serialVersionUID = 1L;
		
		float interQuadrantGapInDegrees;
		
		boolean someHoldsDoNotHaveEnoughNeighbors;
		
		HashSet<Integer> holdsWithinBottomOfWallZone;
		HashSet<Integer> holdsWithinTopOfWallZone;
		
		boolean thereIsAHoldInTheZoneAtTheBottomOfTheWall;
		boolean thereIsAHoldInTheZoneAtTheTopOfTheWall;
		
		HashSet<Integer> holdsConnectedToBottomOfWall;
		HashSet<Integer> holdsConnectedToTopOfWall;
				
		boolean climberCannotReachTheTopFromTheBottomWithCurrentHoldNetwork;
			
		int latestHold;
		
		boolean tooClose;
		
		SetOfHolds () {
			
			latestHold = 0;
			tooClose = false;
			
			someHoldsDoNotHaveEnoughNeighbors = false;
			
			interQuadrantGapInDegrees = Constants.InterQuadrantGapInDegrees;
			
			thereIsAHoldInTheZoneAtTheBottomOfTheWall = true;
			thereIsAHoldInTheZoneAtTheTopOfTheWall = true;
			
			holdsWithinBottomOfWallZone = new HashSet<Integer>();
			holdsWithinTopOfWallZone = new HashSet<Integer>();
			
			holdsConnectedToBottomOfWall = new HashSet<Integer>();
			holdsConnectedToTopOfWall = new HashSet<Integer>();
		}
			
		@SuppressWarnings("unchecked")
		void addHold(MouseEvent e) {
			
			ClimbingHold hold = new ClimbingHold(e.getX(),e.getY());
			
			if ((!contains(hold)) && (hold.y > 0) && (hold.y < heightOfWall) && (hold.x > 0) && (hold.x < Constants.WallWidth)) {
				
				add(latestHold, (T) hold);
				
				checkWhetherSingleAddedHoldIsTooCloseToOthers(latestHold);
				
				latestHold++;
			}
		}
		
		@SuppressWarnings("unchecked")
		void addRandomSprayOfHolds(MouseEvent e) {
			
			ClimbingHold center = new ClimbingHold(e.getX(), e.getY());
			
			if((!contains(center)) && (center.y > 0) && (center.y < heightOfWall) && (center.x > 0) && (center.x < Constants.WallWidth)) {
				
				add(latestHold, (T) center);
				latestHold++;
				
			} else {
				return;
			}
			
			int numberOfHoldsToAddOtherThanCenter = Constants.NumberOfRandomHoldsInRandomSpray;
			
			Area quadrantI = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(center, Direction.UPANDRIGHT, interQuadrantGapInDegrees);
			Area quadrantII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(center, Direction.UPANDLEFT, interQuadrantGapInDegrees);
			Area quadrantIII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(center, Direction.DOWNANDLEFT, interQuadrantGapInDegrees);
			Area quadrantIV = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(center, Direction.DOWNANDRIGHT, interQuadrantGapInDegrees);
			
			while (true) {
				
				ClimbingHold randomHold = createARandomHoldWithinRandomSprayRadiusConstraintsOfCenterHold(center);
				
				if ((quadrantI.contains(randomHold) || quadrantIII.contains(randomHold)) && (!contains(randomHold)) && (randomHold.y > 0) && (randomHold.y < heightOfWall) && (randomHold.x > 0) && (randomHold.x < Constants.WallWidth)) {
					
					add(latestHold, (T) randomHold);
					latestHold++;
					numberOfHoldsToAddOtherThanCenter--;
					break;
				}
			}
			
			while (true) {
				
				ClimbingHold randomHold = createARandomHoldWithinRandomSprayRadiusConstraintsOfCenterHold(center);
				
				if ((quadrantII.contains(randomHold) || quadrantIV.contains(randomHold)) && (!contains(randomHold)) && (randomHold.y > 0) && (randomHold.y < heightOfWall) && (randomHold.x > 0) && (randomHold.x < Constants.WallWidth)) {
					
					add(latestHold, (T) randomHold);
					latestHold++;
					numberOfHoldsToAddOtherThanCenter--;
					break;
				}
			}
			
			for (int i = 0; i < numberOfHoldsToAddOtherThanCenter; i++) {
				
				ClimbingHold randomHold = createARandomHoldWithinRandomSprayRadiusConstraintsOfCenterHold(center);
				
				if((!contains(randomHold)) && (randomHold.y > 0) && (randomHold.y < heightOfWall) && (randomHold.x > 0) && (randomHold.x < Constants.WallWidth)) {
					add(latestHold, (T) randomHold);
					latestHold++;
				}		
			}			
		}
		
		ClimbingHold createARandomHoldWithinRandomSprayRadiusConstraintsOfCenterHold (ClimbingHold centerHold) {
			
			Random rand = new Random();
			
			float randomRadius = ((rand.nextFloat() * (Constants.RandomHoldSprayRadius - Constants.InterholdDistanceTooClose)) + Constants.InterholdDistanceTooClose) * Constants.SizeFactor;
			float randomDirection = rand.nextFloat() * 2 * Pi;
			
			Point randomPoint = CommonAlgorithms.shiftBySlopeAsRadians(centerHold, randomRadius, randomDirection, true);
			
			return new ClimbingHold(randomPoint.x, randomPoint.y);
		}
		
		void checkWhetherSingleAddedHoldIsTooCloseToOthers (int holdIndex) {	
			
			(new Thread() {public void run() {
				
				for (int i = holdIndex - 1; i >= 0; i--) {
					
					float proximity = CommonAlgorithms.findDistance((Point) get(holdIndex), (Point) get(i));
					
					if (proximity < Constants.InterholdDistanceTooClose * Constants.SizeFactor) {
						
						tooClose = true;
						repaint();
						
						try {
							Thread.sleep(5000);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
						
						tooClose = false;
						
						repaint();
						return;
					}
				}
			}}).start();
		}
		
		void analyzeBottomAndTopZones() {
		
			findHoldsInTheZoneAtTheBottomOfTheWall();
			findHoldsInTheZoneAtTheTopOfTheWall();
			
			checkTheZoneAtTheBottomOfTheWallForAHold();
			checkTheZoneAtTheTopOfTheWallForAHold();
		}
		
		void findHoldsInTheZoneAtTheBottomOfTheWall() {
			
			float bottomOfWall = heightOfWall;
			float bottomOfWallZoneBoundary = bottomOfWall - Constants.HeightOfBottomOfWallZone * Constants.SizeFactor;
			
			for(int i = 0; i < size(); i++) {
				
				if (get(i).y > bottomOfWallZoneBoundary && get(i).y < bottomOfWall) {
					
					get(i).withinBottomOfWallZone = true;
					holdsWithinBottomOfWallZone.add(i);
				}
			}
		}
		
		void findHoldsInTheZoneAtTheTopOfTheWall() {
			
			float topOfWallZoneHeight = Constants.HeightOfTopOfWallZone * Constants.SizeFactor;
			
			for(int i = 0; i < size(); i++) {
				
				if (get(i).y > 0 && (get(i).y < topOfWallZoneHeight)) {
					
					get(i).withinTopOfWallZone = true;
					holdsWithinTopOfWallZone.add(i);
				}
			}
		}
		
		void checkTheZoneAtTheBottomOfTheWallForAHold() {
			
			if (holdsWithinBottomOfWallZone.size() > 0) {
				
				thereIsAHoldInTheZoneAtTheBottomOfTheWall = true;
				
			} else {
				
				thereIsAHoldInTheZoneAtTheBottomOfTheWall = false;
				
				holdsGoodIndividually = false;
			}
		}		
		
		void checkTheZoneAtTheTopOfTheWallForAHold() {
			
			if (holdsWithinTopOfWallZone.size() > 0) {
				
				thereIsAHoldInTheZoneAtTheTopOfTheWall = true;
				
			} else {
				
				thereIsAHoldInTheZoneAtTheTopOfTheWall = false;
				
				holdsGoodIndividually = false;
			}
		}
		
		void analyzeQuadrantContentsForEachHold() {
		
			updateTheHoldsThatAreWithinMaxQuadrantDistance();
			
			updateQuadrantHoldProximityBasedOnReachableRangeForEachHold();
			
			updateQuadrantSufficiencyForEachHoldBasedOnReachableRange();
			
			markAnyHoldThatNeedsANeighborInAnyOfItsQuadrants();
		}
		
		void updateTheHoldsThatAreWithinMaxQuadrantDistance() {
			
			float maximumInterholdDistanceForOneHoldToPossiblyBeInQuadrantOfAnotherHold = Constants.MaximumDistanceForProperSpacingInQuadrant * Constants.SizeFactor;
			
			for (int i = 0; i < size() - 1; i++) {
				
				for (int j = i + 1; j < size(); j++) {
					
					float interholdDistance = CommonAlgorithms.findDistance(get(i), get(j));
					
					if (interholdDistance < maximumInterholdDistanceForOneHoldToPossiblyBeInQuadrantOfAnotherHold) {
						
						get(i).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant.add(j);
						
						get(j).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant.add(i);
					}
				}
			}
		}
				
		void updateQuadrantHoldProximityBasedOnReachableRangeForEachHold() {
			
			for (int i = 0; i < size(); i++) {
				
				Area quadrantI = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(get(i), Direction.UPANDRIGHT, interQuadrantGapInDegrees);
				Area quadrantII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(get(i), Direction.UPANDLEFT, interQuadrantGapInDegrees);
				Area quadrantIII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(get(i), Direction.DOWNANDLEFT, interQuadrantGapInDegrees);
				Area quadrantIV = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(get(i), Direction.DOWNANDRIGHT, interQuadrantGapInDegrees);
				
				for (int nh : get(i).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
					
					if (quadrantI.contains(get(nh))) {
						get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI = true;
					}
					
					if (quadrantII.contains(get(nh))) {
						get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII = true;
					}
					
					if (quadrantIII.contains(get(nh))) {
						get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII = true;
					}

					if (quadrantIV.contains(get(nh))) {
						get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV = true;
					}
				}
			}
		}
		
		void updateQuadrantSufficiencyForEachHoldBasedOnReachableRange() {
			
			for (int i = 0; i < size(); i++) {
				
				get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = false;
				
				if ((get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI && get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII) || (get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII && get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII) || (get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII && get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV) || (get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV && get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI)) {
					
					get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = true;
					
				} else {
					
					get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = false;
				}
			}
		}

		void markAnyHoldThatNeedsANeighborInAnyOfItsQuadrants () {
			
			someHoldsDoNotHaveEnoughNeighbors = false;
			
			omnibusHoldTest:
				
				for (int i = 0; i < size(); i++) {

					get(i).needsAHoldWithinReachableRangeSomewhereAbove = true;
					get(i).needsAHoldWithinReachableRangeSomewhereBelow = true;
					get(i).needsAHoldWithinReachableRangeInIOrIII = true;
					get(i).needsAHoldWithinReachableRangeInIIOrIV = true;
					
					if (get(i).withinTopOfWallZone) {
						
						if (!get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII && !get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV) {
							
							holdsGoodIndividually = false;
							someHoldsDoNotHaveEnoughNeighbors = true;
							
							get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = false;

							get(i).needsAHoldWithinReachableRangeSomewhereAbove = false;
							get(i).needsAHoldWithinReachableRangeInIOrIII = false;
							get(i).needsAHoldWithinReachableRangeInIIOrIV = false;
							
						} else {
							
							get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = true;
							
							get(i).needsAHoldWithinReachableRangeSomewhereBelow = false;
							get(i).needsAHoldWithinReachableRangeSomewhereAbove = false;
							get(i).needsAHoldWithinReachableRangeInIOrIII = false;
							get(i).needsAHoldWithinReachableRangeInIIOrIV = false;
						}
					
						continue omnibusHoldTest;
						
					} else if (get(i).withinBottomOfWallZone) {	
						
						if (!get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI && !get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII) {
							
							holdsGoodIndividually = false;
							someHoldsDoNotHaveEnoughNeighbors = true;
							
							get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = false;

							get(i).needsAHoldWithinReachableRangeSomewhereBelow = false;
							get(i).needsAHoldWithinReachableRangeInIOrIII = false;
							get(i).needsAHoldWithinReachableRangeInIIOrIV = false;
						
						} else {
							
							get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants = true;
							
							get(i).needsAHoldWithinReachableRangeSomewhereBelow = false;
							get(i).needsAHoldWithinReachableRangeSomewhereAbove = false;
							get(i).needsAHoldWithinReachableRangeInIOrIII = false;
							get(i).needsAHoldWithinReachableRangeInIIOrIV = false;
						}
						
						continue omnibusHoldTest;
						
					} else {					
						
						get(i).needsAHoldWithinReachableRangeSomewhereBelow = false;
						get(i).needsAHoldWithinReachableRangeSomewhereAbove = false;
					}
					
					if(!get(i).hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants) {
						
						holdsGoodIndividually = false;
						someHoldsDoNotHaveEnoughNeighbors = true;
						
						if(get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantI || get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIII) {
									
							get(i).needsAHoldWithinReachableRangeInIOrIII = false;
												
						} else if (get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantII || get(i).hasProperlySpacedHoldBasedOnReachableRangeInQuadrantIV) {
							
							get(i).needsAHoldWithinReachableRangeInIIOrIV = false;
							
						}
					}
				}
		}
		
		void analyzeTheVerticalConnectivityOfTheHoldsBasedOnQuadrantLimits() {
			
			connectAndMarkHoldsFromTheBottomOfTheWallUpwardBasedOnQuadrantLimits();
			connectAndMarkHoldsFromTheTopOfTheWallDownwardBasedOnQuadrantLimits();
		}
		
		void connectAndMarkHoldsFromTheBottomOfTheWallUpwardBasedOnQuadrantLimits() {
			
			clearAllConnectionAnalysisStates();
			
			connectHoldsFromTheBottomOfTheWallUpwardBasedOnQuadrantLimits();
			
			updateStatusOfConnectionToTheBottomOfTheWallBasedOnQuadrantLimitsForEachHold();
		}
		
		void clearAllConnectionAnalysisStates() {
			
			for(int i=0; i < size(); i++) {
				
				get(i).connectionAnalysisState = ConnectionAnalysisState.NOTYETCONNECTED;
			}			
		}
		
		void connectHoldsFromTheBottomOfTheWallUpwardBasedOnQuadrantLimits() {
			
			LinkedList<Integer> connectionAnalysisQueue = new LinkedList<Integer>();
	
			for (int i : holdsWithinBottomOfWallZone) {
				
				connectionAnalysisQueue.add(i);
			}
			
			while(!connectionAnalysisQueue.isEmpty()) {
				
				Integer indexOfHoldBeingProcessed = connectionAnalysisQueue.removeFirst();
				
				if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.NOTYETCONNECTED) {
					
					get(indexOfHoldBeingProcessed).connectionAnalysisState = ConnectionAnalysisState.CONNECTING;
				
					for(int nh : get(indexOfHoldBeingProcessed).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
						
						connectionAnalysisQueue.add(nh);
					}
					
				} else if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.CONNECTING) {
					
					get(indexOfHoldBeingProcessed).connectionAnalysisState = ConnectionAnalysisState.CONNECTED;
					
				} else if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.CONNECTED) {
					
					continue;
				}
			}
		}
		
		void updateStatusOfConnectionToTheBottomOfTheWallBasedOnQuadrantLimitsForEachHold() {
			
			for(int i=0; i < size(); i++) {
				
				if (get(i).connectionAnalysisState == ConnectionAnalysisState.NOTYETCONNECTED) {
					
					holdsGoodAsANetwork = false;
					showingWebOfConnections = true;

					get(i).connectedToBottom = false;
					
				} else {
					
					get(i).connectedToBottom = true;
					holdsConnectedToBottomOfWall.add(i);
				}
			}
		}
		
		void connectAndMarkHoldsFromTheTopOfTheWallDownwardBasedOnQuadrantLimits() {
			
			clearAllConnectionAnalysisStates();
			
			connectHoldsFromTheTopOfTheWallDownwardBasedOnQuadrantLimits();
			
			updateStatusOfConnectionToTheTopOfTheWallBasedOnQuadrantLimitsForEachHold();		
		}
		
		void connectHoldsFromTheTopOfTheWallDownwardBasedOnQuadrantLimits() {
			
			LinkedList<Integer> connectionAnalysisQueue = new LinkedList<Integer>();

			for (int i : holdsWithinTopOfWallZone) {
				
				connectionAnalysisQueue.add(i);
			}
			
			while(!connectionAnalysisQueue.isEmpty()) {
				
				Integer indexOfHoldBeingProcessed = connectionAnalysisQueue.removeFirst();
				
				if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.NOTYETCONNECTED) {
					
					get(indexOfHoldBeingProcessed).connectionAnalysisState = ConnectionAnalysisState.CONNECTING;
				
					for(int nh : get(indexOfHoldBeingProcessed).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
						
						connectionAnalysisQueue.add(nh);
					}
					
				} else if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.CONNECTING) {
					
					get(indexOfHoldBeingProcessed).connectionAnalysisState = ConnectionAnalysisState.CONNECTED;
					
				} else if (get(indexOfHoldBeingProcessed).connectionAnalysisState == ConnectionAnalysisState.CONNECTED) {
					
					continue;
				}
			}
		}
		
		void updateStatusOfConnectionToTheTopOfTheWallBasedOnQuadrantLimitsForEachHold() {
			
			for(int i=0; i < size(); i++) {
				
				if (get(i).connectionAnalysisState == ConnectionAnalysisState.NOTYETCONNECTED) {
					
					holdsGoodAsANetwork = false;
					showingWebOfConnections = true;

					get(i).connectedToTop = false;
					
				} else {
					
					get(i).connectedToTop = true;
					holdsConnectedToTopOfWall.add(i);
				}
			}
		}
		
		void assignBorderStatusForEachHold() {
		
			updateQuadrantHoldProximityForDeterminingBorderHolds();
			
			identifyBorderHolds();
		}
				
		void updateQuadrantHoldProximityForDeterminingBorderHolds() {
			
			for (int i = 0; i < size(); i++) {
				
				Area quadrantI = CommonAlgorithms.createQuadrantForDeterminingBorderHolds(get(i), Direction.UPANDRIGHT);
				Area quadrantII = CommonAlgorithms.createQuadrantForDeterminingBorderHolds(get(i), Direction.UPANDLEFT);
				Area quadrantIII = CommonAlgorithms.createQuadrantForDeterminingBorderHolds(get(i), Direction.DOWNANDLEFT);
				Area quadrantIV = CommonAlgorithms.createQuadrantForDeterminingBorderHolds(get(i), Direction.DOWNANDRIGHT);
				
				for (int nh : get(i).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
					
					if (quadrantI.contains(get(nh))) {
						get(i).hasHoldInQuadrantI = true;
					}
					
					if (quadrantII.contains(get(nh))) {
						get(i).hasHoldInQuadrantII = true;
					}
					
					if (quadrantIII.contains(get(nh))) {
						get(i).hasHoldInQuadrantIII = true;
					}

					if (quadrantIV.contains(get(nh))) {
						get(i).hasHoldInQuadrantIV = true;
					}
				}
				
				if (get(i).withinBottomOfWallZone || get(i).withinTopOfWallZone) {
					
					if(get(i).hasHoldInQuadrantI) {
						get(i).hasHoldInQuadrantIV = true;
					}
					
					if(get(i).hasHoldInQuadrantII) {
						get(i).hasHoldInQuadrantIII = true;
					}
					
					if(get(i).hasHoldInQuadrantIII) {
						get(i).hasHoldInQuadrantII = true;
					}
					
					if(get(i).hasHoldInQuadrantIV) {
						get(i).hasHoldInQuadrantI = true;
					}
				}
			}
		}
		
		void identifyBorderHolds() {
			
			for (int i = 0; i < size(); i++) {
				
				ClimbingHold h = get(i);
				
				if (!(h.hasHoldInQuadrantI && h.hasHoldInQuadrantII && h.hasHoldInQuadrantIII && h.hasHoldInQuadrantIV)) {
					
					get(i).isABorderHold = true;
				}
			}
		}		
	}
	
	synchronized void seeIfHoldsAreSpacedProperly() {
		
		holdsGoodIndividually = true;
		holdsGoodAsANetwork = true;
		showingWebOfConnections = false;

		setOfUserEnteredHolds.analyzeBottomAndTopZones();
						
		setOfUserEnteredHolds.analyzeQuadrantContentsForEachHold();
		
		if (holdsGoodIndividually) {
			
			setOfUserEnteredHolds.analyzeTheVerticalConnectivityOfTheHoldsBasedOnQuadrantLimits();
		}

		if (holdsGoodIndividually && holdsGoodAsANetwork) {

			waitingToBeFinalized = true;
			displayingFeedback = false;
			
		} else {

			waitingToBeFinalized = false;
			displayingFeedback = true;
		}
	}
	
	void lockInFinalStateOfWall() {
		
		setOfUserEnteredHolds.assignBorderStatusForEachHold();
		
		wallFinalized = true;
		waitingToBeFinalized = false;
		
	}

	void notifyClimberThatWallIsReady() {
	
		wallClimber.receiveMessageThatTheWallIsReadyForClimber(this); 
		
	}
	
	void showClimberMovingOnUserEnteredHolds() {
		
		showingClimber = true;
		repaint();
		
		(new Thread() {public void run() {
			
			while(wallClimber.isClimberOnTheWall() && wallClimber.isClimberTryingToClimb()) {						
		
				try {
					Thread.sleep(1000/Constants.DisplayTickRate);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
				repaint();
				
				wallClimber.wallHasDrawnTheLatestClimberMoves = true;
			}
			
			if(!wallClimber.isClimberOnTheWall()) {
			
				showingClimber = false;
			
			}
			
			if(!wallClimber.isClimberTryingToClimb()) {
				
				showingAcceptanceOfDefeat = true;
				
			}
			
			showingFinalMessageOfAppreciation = true;
			
			repaint();
			
		}}).start();
	}
	
	protected void paintComponent (Graphics g) {
		
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;
		
		if (displayingIntroduction) {
			
			showIntroductionElements(g2);
			
		}
		
		if (showingInitialInstructions) {
			
			drawInitialInstructions(g2);
			
		}
		
		if (!displayingIntroduction) {
		
			drawUserEnteredHoldsBoundriesAndNetworking(g2);
			
			drawResetButton(g2);
			
		}
		
		if (!wallFinalized) {
			
			showWallPreparationPrompts(g2);
		}
				
		if (showingClimber) {
			
			wallClimber.drawClimber(g2);
		}
		
		
		if (showingAcceptanceOfDefeat) {
			
			showMessageOfAcceptanceOfDefeat(g2);
		}
		
		
		if (showingFinalMessageOfAppreciation) {
			
			showFinalMessageOfAppreciation(g2);
		}
		
		paintOverAreaOutsideOfWall(g2);
		
		g2.dispose();
	}
	
	
	void showIntroductionElements (Graphics2D g2d) {

		drawTitle(g2d);
		
		if (requestForAssistanceVisible) {
			
			drawRequestForAssistance(g2d);	
		
		}
		
		if (startButtonVisible) {
			
			drawButtonUserCanPushToBeginRouteSetting(g2d);	
		
		}
		
		drawIntroductionHolds(g2d);
	}
	
	void drawTitle (Graphics2D g2d) {
		
		g2d.setPaint(Color.BLUE);
		g2d.setFont((new Font("SansSerif",1,36)));
		g2d.drawString("Rock Climbing", 215, 125);
		g2d.drawString("Simulator", 253, 175);

		g2d.setFont((new Font("SansSerif",1,12)));
		g2d.drawString("by C. Parker", 295, 250);

		g2d.setFont((new Font("SansSerif",1,10)));
		g2d.drawString("Copyright 2016. All rights reserved.", 243, heightOfWall - 25);
		
	}
	
	void drawRequestForAssistance (Graphics2D g2d) {
		
		g2d.setPaint(Color.BLUE);
		g2d.setFont((new Font("SansSerif",1,16)));
		g2d.drawString("Will you create a challenging route for me to climb?", 145, 400);
		g2d.drawString("Please click on the button below to begin.", 175, 450);
		
	}
	
	void drawButtonUserCanPushToBeginRouteSetting (Graphics2D g2d) {
		
		g2d.setPaint(Color.black);
		g2d.drawRect(Constants.XValueForStartButtonLeftBorder, Constants.YValueForStartButtonUpperBorder, Constants.WidthOfStartButton, Constants.HeightOfStartButton);
		g2d.setPaint(Color.white);
		g2d.fillRect(Constants.XValueForStartButtonLeftBorder + 1, Constants.YValueForStartButtonUpperBorder + 1, Constants.WidthOfStartButton - 2, Constants.HeightOfStartButton - 2);
		
		g2d.setPaint(Color.black);
		g2d.setFont((new Font("SansSerif",1,12)));
		g2d.drawString("Start Setting a Route", Constants.XValueForStartButtonLeftBorder + 15, Constants.YValueForStartButtonUpperBorder + 25);
		
	}
	
	void drawIntroductionHolds (Graphics2D g2d) {
		
		float hd = Constants.HoldDiameter * Constants.SizeFactor;
		
		for (ClimbingHold h: setOfIntroductionHolds) {
			
			if (h.isVisibleInIntroductionAnimation) {
				
				g2d.setPaint(Color.BLACK);
				g2d.fill(new Ellipse2D.Float(h.x - (hd/2), h.y - (hd/2), hd, hd));
								
			}
		}
	}
	
	void drawInitialInstructions (Graphics2D g2d) {
		
		g2d.setPaint(Color.BLUE);
		g2d.setFont((new Font("SansSerif",1,16)));
		g2d.drawString("Design a route on the wall, and I'll try to figure it out.", 115, 275);
		g2d.drawString("Be as imaginative as you want!", 193, 325);
		
	}
	
	void drawUserEnteredHoldsBoundriesAndNetworking (Graphics2D g2d) {
		
		g2d.setPaint(Color.BLACK);
		g2d.draw(new Rectangle2D.Float(-1, -1, Constants.WallWidth + 2, heightOfWall + 2));
		
		if (!setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheTopOfTheWall) {
			
			g2d.setPaint(Constants.BoundaryDesignationColor);
			g2d.fill(new Rectangle2D.Float(0, Constants.HeightOfTopOfWallZone * Constants.SizeFactor, Constants.WallWidth + 1, Constants.ThicknessOfTheBoundaryMarkerForTheZonesAtTheTopAndTheBottomOfTheWall));
		}
		
		if (!setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheBottomOfTheWall) {

			g2d.setPaint(Constants.BoundaryDesignationColor);
			g2d.fill(new Rectangle2D.Float(0,(heightOfWall - ((Constants.HeightOfBottomOfWallZone * Constants.SizeFactor) + Constants.ThicknessOfTheBoundaryMarkerForTheZonesAtTheTopAndTheBottomOfTheWall)), Constants.WallWidth + 1, Constants.ThicknessOfTheBoundaryMarkerForTheZonesAtTheTopAndTheBottomOfTheWall));
		}
		
		float hd = Constants.HoldDiameter * Constants.SizeFactor;
		
		if (!showingWebOfConnections) {
			
			theDrawingProcessForEachHold:
				
				for (ClimbingHold h: setOfUserEnteredHolds) {
								
					if (h.hasASetOfProperlySpacedHoldsBasedOnReachableRangeInRelevantQuadrants && !h.needsAHoldWithinReachableRangeSomewhereAbove && !h.needsAHoldWithinReachableRangeSomewhereBelow) {
						
						g2d.setPaint(Color.BLACK);
						g2d.fill(new Ellipse2D.Float(h.x - (hd/2), h.y - (hd/2), hd, hd));
						
						continue theDrawingProcessForEachHold;
										
					} else {
						
						displayingFeedback = true;
						
						g2d.setPaint(Color.RED);
						g2d.fill(new Ellipse2D.Float(h.x - (hd/2), h.y - (hd/2), hd, hd));
					}
						
						
					if (h.needsAHoldWithinReachableRangeSomewhereBelow) {
						
						Area quadrantIII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.DOWNANDLEFT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						Area quadrantIV = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.DOWNANDRIGHT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						
						Area lowQuadrants = new Area();
						
						lowQuadrants.add(quadrantIII);
						lowQuadrants.add(quadrantIV);
						
						g2d.setPaint(Constants.TransparentBlue);
						
						g2d.fill(lowQuadrants);
					}
					
					if (h.needsAHoldWithinReachableRangeSomewhereAbove) {
						
						Area quadrantI = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.UPANDRIGHT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						Area quadrantII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.UPANDLEFT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						
						Area highQuadrants = new Area();
						
						highQuadrants.add(quadrantI);
						highQuadrants.add(quadrantII);
						
						g2d.setPaint(Constants.TransparentBlue);
						
						g2d.fill(highQuadrants);
					}
					
					if (h.needsAHoldWithinReachableRangeInIOrIII) {
						
						Area quadrantI = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.UPANDRIGHT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						Area quadrantIII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.DOWNANDLEFT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						
						Area lowLeftToHighRightQuadrants = new Area();
						
						lowLeftToHighRightQuadrants.add(quadrantI);
						lowLeftToHighRightQuadrants.add(quadrantIII);
						
						g2d.setPaint(Constants.TransparentRed);
						
						g2d.fill(lowLeftToHighRightQuadrants);
					}
					
					if (h.needsAHoldWithinReachableRangeInIIOrIV) {
						
						Area quadrantII = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.UPANDLEFT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						Area quadrantIV = CommonAlgorithms.createQuadrantOfProperHoldSpacingBasedOnReachableRange(h, Direction.DOWNANDRIGHT, setOfUserEnteredHolds.interQuadrantGapInDegrees);
						
						Area highLeftToLowRightQuadrants = new Area();
						
						highLeftToLowRightQuadrants.add(quadrantII);
						highLeftToLowRightQuadrants.add(quadrantIV);
						
						g2d.setPaint(Constants.TransparentBlue);
						
						g2d.fill(highLeftToLowRightQuadrants);
					}
				}
		}
		
		if (showingWebOfConnections) {
			
			if (tryingToAddHolds) {
				
				for (ClimbingHold h: setOfUserEnteredHolds) {
					
					g2d.setPaint(Color.BLACK);
					g2d.fill(new Ellipse2D.Float(h.x - (hd/2),h.y - (hd/2),hd,hd));
				}
				
			} else {
				
				for (ClimbingHold h: setOfUserEnteredHolds) {
					
					if (h.connectedToBottom || h.connectedToTop) {
						
						g2d.setPaint(Color.BLACK);
						g2d.fill(new Ellipse2D.Float(h.x - (hd/2),h.y - (hd/2),hd,hd));
										
					} else {
						
						g2d.setPaint(Color.RED);
						g2d.fill(new Ellipse2D.Float(h.x - (hd/2),h.y - (hd/2),hd,hd));
					}
				}
			}
						
			float limitRadius = Constants.MaximumDistanceForProperSpacingInQuadrant * Constants.SizeFactor;
			
			g2d.setPaint(Color.GRAY);
			
			for (ClimbingHold h : setOfUserEnteredHolds) {
				
				g2d.draw(new Ellipse2D.Float(h.x - limitRadius, h.y - limitRadius, 2 * limitRadius, 2 * limitRadius));
			}
			
			g2d.setPaint(Color.DARK_GRAY);
			
			for (Integer i : setOfUserEnteredHolds.holdsConnectedToBottomOfWall) {
				
				for(Integer j : setOfUserEnteredHolds.get(i).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
					
					g2d.draw(new Line2D.Float(setOfUserEnteredHolds.get(i), setOfUserEnteredHolds.get(j)));
				}
			}
			
			for (Integer i : setOfUserEnteredHolds.holdsConnectedToTopOfWall) {
				
				for(Integer j : setOfUserEnteredHolds.get(i).holdsThatAreCloseEnoughToPossiblyBeInAQuadrant) {
					
					g2d.draw(new Line2D.Float(setOfUserEnteredHolds.get(i), setOfUserEnteredHolds.get(j)));
				}
			}
		}
	}
	
	void drawResetButton(Graphics2D g2d) {
		
		g2d.setPaint(Color.black);
		g2d.drawRect(Constants.XValueForResetButtonLeftBorder, Constants.YValueForResetButtonUpperBorder, Constants.WidthOfResetButton, Constants.HeightOfResetButton);
		g2d.setPaint(Color.white);
		g2d.fillRect(Constants.XValueForResetButtonLeftBorder + 1, Constants.YValueForResetButtonUpperBorder + 1, Constants.WidthOfResetButton - 2, Constants.HeightOfResetButton - 2);
		
		g2d.setPaint(Color.black);
		g2d.setFont((new Font("SansSerif",1,12)));
		g2d.drawString("Reset the Wall", Constants.XValueForResetButtonLeftBorder + 5, Constants.YValueForResetButtonUpperBorder + 17);
		
	}
	
	void showWallPreparationPrompts(Graphics2D g2d) {
		
		if (!holdsGoodIndividually || !holdsGoodAsANetwork) {
			promptForACheckOfUserEnteredHolds (g2d);
		}
		
		if (!holdsGoodAsANetwork) {
			explainTheNeedForAContinuousConnection(g2d);
		}
		
		if (setOfUserEnteredHolds.tooClose) {
			requestUserForAChallenge(setOfUserEnteredHolds, g2d);
		}
		
		if (!setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheTopOfTheWall || !setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheBottomOfTheWall) {
			promptForExtremeHolds(g2d);
		}
		
		if (setOfUserEnteredHolds.someHoldsDoNotHaveEnoughNeighbors) {
			promptForCloserHolds(g2d);
		}
		
		if (waitingToBeFinalized) {
			promptForFinalApprovalOfHolds(g2d);
		}
	}
	
	void promptForACheckOfUserEnteredHolds (Graphics2D g2d) {
		
		if (!displayingIntroduction && !wallFinalized && (tryingToAddHolds || !displayingFeedback)) {
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Color.BLUE);
			
			g2d.drawString("Click on the wall to add climbing holds.", 60, heightOfWall - 70);
			g2d.drawString("Press any key to check your placements.", 60, heightOfWall - 50);
			g2d.drawString("Right-click to include a random clustering.", 60, heightOfWall - 30);
		}
	}
	
	void explainTheNeedForAContinuousConnection(Graphics2D g2d) {
		
		int textStartingHeight = 200;
		int textStartingWidth = 20;
		
		int textBoxWidth = 450;
		int textBoxHeight = 300;
		
		if (!wallFinalized && displayingFeedback && !tryingToAddHolds) {
			
			g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
			g2d.fillRect(textStartingWidth - 5, textStartingHeight - 21, textBoxWidth, textBoxHeight);
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Color.BLUE);
			
			g2d.drawString("This is starting to look really fun.", textStartingWidth, textStartingHeight);
			g2d.drawString("Thank you for giving all the individual holds excellent", textStartingWidth, textStartingHeight + 40);
			g2d.drawString("neighbors!", textStartingWidth, textStartingHeight + 60);
			
			g2d.drawString("As things stand, though, I won't be able to move from", textStartingWidth, textStartingHeight + 100);
			g2d.drawString("all the holds that reach the bottom of the wall to all the", textStartingWidth, textStartingHeight + 120);
			g2d.drawString("holds that reach the top of the wall.", textStartingWidth, textStartingHeight + 140);
			
			g2d.drawString("Please address the gap by adding more holds", textStartingWidth, textStartingHeight + 180);
			g2d.drawString("in whatever pattern you'd like.", textStartingWidth, textStartingHeight + 200);
			
			g2d.drawString("The proximity circles and the connection network", textStartingWidth, textStartingHeight + 240);
			g2d.drawString("are shown for your convenience", textStartingWidth, textStartingHeight + 260);
		}
	}
	
	void requestUserForAChallenge (ArrayList<ClimbingHold> al, Graphics2D g2d) {
		
		g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
		g2d.fillRect(45, 169, 300, 60);
		
		g2d.setFont(new Font("SansSerif",1,16));
		g2d.setColor(Color.RED);
		
		g2d.drawString("You're putting them too close together.", 50, 190);
		g2d.drawString("I want a challenge!", 50, 220);
	}
	
	void promptForExtremeHolds(Graphics2D g2d) {
			
		if (!setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheTopOfTheWall) {
			
			g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
			g2d.fillRect(45, 49, 350, 60);
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Constants.BoundaryDesignationColor);
			
			g2d.drawString("I'm not sure I'll be able to reach the top.", 50, 70);
			g2d.drawString("Please add at least one hold above the line.", 50, 100);
		}
		
		if (!setOfUserEnteredHolds.thereIsAHoldInTheZoneAtTheBottomOfTheWall) {
			
			g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
			g2d.fillRect(45, (heightOfWall - 121), 350, 60);
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Constants.BoundaryDesignationColor);
			
			g2d.drawString("I won't be able to start on these high holds.", 50, (heightOfWall - 100));
			g2d.drawString("Please add at least one hold below the line.", 50, (heightOfWall - 70));
		}
	}
	
	void promptForCloserHolds(Graphics2D g2d) {

		if (tryingToAddHolds) {
			
			g2d.setPaint(Constants.ClearestTransparentWallColor);
			g2d.fillRect(15, 239, 550, 330);
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Constants.TransparentNeedCloserHoldsColor);
		
		} else {
			
			g2d.setPaint(Constants.MediumTransparentWallColor);
			g2d.fillRect(15, 239, 550, 330);
			
			g2d.setFont(new Font("SansSerif",1,16));
			g2d.setColor(Constants.OpaqueNeedCloserHoldsColor);
			
		}
			
		g2d.drawString("Unfortunately, I can't climb the wall unless you add more holds.", 20, 260);
		
		g2d.drawString("For each colored area around a hold, add at least one reachable neighboring hold", 20, 300);
		g2d.drawString("within the zone indicated by a specific color.", 20, 320);
		
		g2d.drawString("In other words, a hold surrounded by two differently", 20, 360);
		g2d.drawString("colored areas will need at least two more neighboring holds.", 20, 380);
		
		g2d.drawString("If you can't see the colors, it may be helpful to know that", 20, 420);
		g2d.drawString("holds normally need at least one neighboring hold for each diagonal.", 20, 440);
		g2d.drawString("The exception is for holds at the top and bottom of the wall,", 20, 480);
		g2d.drawString("which need only one neighboring hold for the pair of highlighted areas.", 20, 500);
		
		g2d.drawString("Press any key when you want me to check again.", 20, 540);
	}
	
	void promptForFinalApprovalOfHolds(Graphics2D g2d) {
		
		g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
		g2d.fillRect(95, heightOfWall/2 - 121, 300, 200);
		
		g2d.setFont(new Font("SansSerif",1,16));
		g2d.setColor(Constants.FinalizeHoldsConfirmationTextColor);
		
		g2d.drawString("This looks good to me.", 100, heightOfWall/2 - 100);
		
		g2d.drawString("Can I try to climb it?", 100, heightOfWall/2 - 50);
		
		g2d.drawString("Press any key to finalize your holds.", 100, heightOfWall/2);
		
		g2d.drawString("Otherwise, click to add more.", 100, heightOfWall/2 + 50);
	}
	
	
	void showMessageOfAcceptanceOfDefeat(Graphics2D g2d) {
	
		g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
		g2d.fillRect(35, 79, 410, 85);
		
		g2d.setFont(new Font("SansSerif",1,16));
		g2d.setColor(Color.BLUE);
		
		g2d.drawString("Well, you stumped me!", 40, 100);
		g2d.drawString("Have you ever considered a career in route-setting?", 40, 160);

	}
		
	void showFinalMessageOfAppreciation(Graphics2D g2d) {
		
		g2d.setPaint(Constants.MostOpaqueTransparentWallColor);
		g2d.fillRect(35, (heightOfWall/2) - 21, 400, 175);
				
		g2d.setFont(new Font("SansSerif",1,16));
		g2d.setColor(Color.BLUE);
		
		g2d.drawString("Thank you so much for giving me this opportunity", 40, heightOfWall/2);
		g2d.drawString("to show you how I solve problems.", 40, heightOfWall/2 + 24);
		
		g2d.drawString("I had a BLAST!", 40, heightOfWall/2 + 144);
		
	}
	
	void paintOverAreaOutsideOfWall(Graphics2D g2d) {
		
		g2d.setPaint (Constants.WallColor);
		
		g2d.fillRect (0, heightOfWall + 2, Constants.WallWidth + 1, heightOfWall);
		g2d.fillRect (Constants.WallWidth + 2, 0, Constants.WallWidth, heightOfWall + 1);
		
	}
	
	public Dimension getPreferredSize () {
		return new Dimension(Constants.WallWidth,heightOfWall);
	}
	
	public Dimension getMinimumSize () {
		return new Dimension(Constants.WallWidth,heightOfWall);
	}
}