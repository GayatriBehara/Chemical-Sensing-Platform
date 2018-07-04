
#define DESIRED_PHASE x /* x is the ADC_COUNT corresponding to input voltage representing 45deg/desired phase angle
#define PHASE_RESOLUTION y /* y is the allowed margin to stop the PLL loop

main()
{
	Initializations();

	while(1) {
		awaitWakeup();
		getCalcium();
	}
}

getCalcium ()
{
	phaseMatch = 0;
	phaseAcqErr=0;
	dacOut = 0;
	*dacPort = dacOut;
	while(!phaseMatch) {
		currentPhase = getPhase();
		if(abs(currentPhase-DESIRED_PHASE)<PHASE_RESOLUTION) phaseMatch=1;
		else {
			*dacPort=dacOut++;
			if(dacOut == MAX_DAC_COUNT) {
				phaseAcqErr = 1;
				break;
			}
		}
	}
	if(!phaseAcqErr) {
		currentFreq = getFrequency();
		calcCalcium(currentFreq);
		compTemp();
	}
	else {
	/*handle error here */
	}
}
/* Reads ADC "Phase" channel input */
getPhase()
{

}
/* Use pulse counter to measure the frequency */
getFrequency()
{
}
/* Use LUT or expression to convert frequency to Calcium */
calcCalcium(int frequency)
{

}
