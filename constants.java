package com.craigcode.climbing_simulator_refactored;

import java.awt.Color;

class Constants {
	
	static final float Pi = (float) Math.PI;
	
	//overall scale adjuster
	static final float SizeFactor = 1f;
	
	//variables for wall dimensions, hold size, and rope thickness
	static final int WallWidth = 650;	
	
	static final float HoldDiameter = 10.0f;
	static final float RopeDiameter = 2.0f;
	
	//introduction screen parameters
	static final float WidthOfBufferZoneForRandomHoldGeneration = 100.0f;
	static final float RadianLimitOfCentralDirectionforRandomHoldGeneration = Pi/4;
	static final float DistanceLimitOfWholeSegmentOfCentralDirectionForRandomHoldGeneration = 100.0f;
	static final float DistanceLimitOfPartialSegmentOfCentralDirectionForRandomHoldGeneration = 30.0f;
	static final float DistanceLimitOfSegmentUsedToPlaceHoldPerpendicularToCentralDirectionForRandomHoldGeneration = 35.0f;
	
	static final int numberOfHoldsToDisplayPerSecondOnIntroductionScreen = 3;
			
	//button parameters
	static final int XValueForStartButtonLeftBorder = 260;
	static final int YValueForStartButtonUpperBorder = 525;
	static final int WidthOfStartButton = 145;
	static final int HeightOfStartButton = 43;	
	
	static final int XValueForResetButtonLeftBorder = 30;
	static final int YValueForResetButtonUpperBorder = 20;
	static final int WidthOfResetButton = 90;
	static final int HeightOfResetButton = 27;	
	
	//timing constants
	//value is cycles per second (hertz)
	static final int DisplayTickRate = 60;
	//value is in milliseconds
	static final int MovementPromptPauseLength = 5;
	
	//movement constants
	static final float LoweringTickDistance = 3.0f;
	static final float ToppingOutDistance = 1.0f;
	static final float LimbMovementIncrement = 1.0f;
	static final float TorsoMovementIncrement = .5f;
	
	static final float DegreesOfLegReachHeightGainedAsFootMovesAwayFromUpperOuterHipBoxPoint = 20.0f;
	static final float DegreesOfLegReachWidthGainedAsFootMovesAwayFromInseam = 7.5f;
	
	static final float MaximumOverallExtension = 0.95f;
	
	//structural positioning dimensions
	static final float ShoulderSocketSpan = 40.0f;
	static final float HumerusLength = 34.0f;
	static final float ForearmLength = 28.0f;
	static final float HandRadius = 6.0f;
	static final float TorsoHeightFromTopOfTrapsToWaist = 56.0f;
	static final float BeltBuckleToCenterOfPelvisBetweenHipSocketsVerticalDistance = 10.0f;
	static final float HipSocketSpan = 20.0f;
	static final float FemurLength = 40f;
	static final float CalfLength = 36f;
	static final float FootLengthFromAnkle = 12.0f;
	static final float FootLength = 17.0f;
	static final float FootHeight = 6.0f;
	
	//dimensions for drawing
	static final float HeadRadius = 15.0f;
	static final float DistanceFromNeckBaseAtShoulderSocketLevelToBottomOfHeadCircle = 10.0f;
	static final float NeckThickness = 16.0f;
	
	static final float UpperArmWidth = 11.0f;
	static final float ElbowWidth = 9.0f;
	static final float WristWidth = 7.0f;
	
	static final float MiddleFingerLength = 18.0f;
	static final float FingerWidth = 1.8f;
	static final float ThumbLength = 11.8f;
	static final float ThumbWidth = 2.6f;

	static final float UpperThighWidth = 16.0f;
	static final float KneeWidth = 9.0f;
	static final float AnkleWidth = 7.0f;
	
	static final float WaistToInseamCentralVerticalDistance = 18.0f;
	static final float HipToOuterPelvisCreaseVerticalDistance = 4.0f;
	static final float InseamToInnerPelvisCreaseHorizontalDistance = 6.0f;
	
	//body dimension ratios
	static final float DistanceFromTopOfTrapsToTorsoCOGComparedToDistanceFromTopOfTrapsToWaistRatio = .8f;
	
	static final float RelativeDistanceFromHeelToBallOfFoot = 0.75f;
	
	static final float FactorForDeterminingHeightOfTrapsAgainstSideOfNeckDeterminedAsAFractionOfTheVerticalDistanceFromShoulderLevelToTheBottomOfTheHead = 0.6f;
	static final float FactorForDeterminingAdditionalVerticalDistanceOfNeckDrawingAreaDeterminedAsAFractionOfHeadRadius = 0.5f;
	static final float LatHorizontalCutInWidthAsRatioOfOverallShoulderSocket = 0.13f;
	static final float LatInsertionPointRatioOfOverallTorsoHeightStartingFromWaist = 0.44f;
	
	static final Color WallColor = new Color (220,220,220);
	static final Color ClearestTransparentWallColor = new Color (.859f, .859f, .859f, .2f);
	static final Color MediumTransparentWallColor = new Color (.859f, .859f, .859f, .5f);
	static final Color MostOpaqueTransparentWallColor = new Color (.859f, .859f, .859f, .75f);
	static final Color ShirtColor = new Color(18,150,165);
	static final Color PantsColor = new Color(150,93,80);
	static final Color SkinTone = new Color(229,174,93);
	static final Color HairColor = new Color(240,103,60);
	static final Color RopeColor = new Color(225,15,240);
	
	static final Color FinalizeHoldsConfirmationTextColor = new Color (26,120,150);
	static final Color OpaqueNeedCloserHoldsColor = new Color (0.95f, 0.15f, 0.35f, 1.0f);
	static final Color TransparentNeedCloserHoldsColor = new Color (0.95f, 0.15f, 0.35f, .35f);
	static final Color BoundaryDesignationColor = new Color(50,170,110);
	
	static final Color TransparentBlue = new Color(0.1f, 0.5f, 0.95f, 0.35f);
	static final Color TransparentRed = new Color(0.95f, 0.15f, 0.1f, 0.35f);
	
	static final float InterholdDistanceTooClose = 20.0f;
	
	static final float MinimumDistanceForProperSpacingInQuadrant = 25.0f;		
	static final float MaximumDistanceForProperSpacingInQuadrant = 75.0f;
	
	static final float InterQuadrantGapInDegrees = 10.0f;
	
	static final float HeightOfBottomOfWallZone = 40.0f;
	static final float HeightOfTopOfWallZone = 20.0f;
			
	static final int NumberOfRandomHoldsInRandomSpray = 5;
	static final float RandomHoldSprayRadius = 65.0f;
	
	static final float ThicknessOfTheBoundaryMarkerForTheZonesAtTheTopAndTheBottomOfTheWall = 4.0f;
}