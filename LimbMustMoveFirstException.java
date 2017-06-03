package com.craigcode.climbing_simulator_refactored;

class LimbMustMoveFirstException extends Exception {

		private static final long serialVersionUID = 1L;
	
	LimbIdentifier limbIdentifier;
	
	public LimbMustMoveFirstException(LimbIdentifier limbIdentifier) {
		
		this.limbIdentifier = limbIdentifier;
	}

	public LimbIdentifier getLimbIdentifier() {
		
		return this.limbIdentifier;
	}
}