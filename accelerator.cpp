// Posted to the T41 group, 5/11/2022.
/*
 *	Accelerator parameters. "ACCELERATE" turns the feature on or off.
 *
 *	If "ACC_FACTOR" is zero, TJ's original dynamic accelerate/decelerate math is used
 *	(it's really cool). If "ACC_FACTOR" is a number other than zero, a linear
 *	acceleration formula is used that just multiplies the "bandData.incr" value by the
 *	"ACC_FACTOR" when it kicks in.
 *
 *	"V_TH" is the number of encoder interrupts that must be seen in one read of the
 *	encoder for the accelerator to kick in.
 */

#define	ACCELERATE	  true		// Accelerator enabled = "true"; Off = "false"
#define	ACC_FACTOR	  0			// Frequency increment multiplier with accelerator on
#define	V_TH		  2			// Encoder count where the accelerator kinks in


/*
 *	Some global variables needed for managing the rotary encoders; note the "volatile"
 *	designation as these are used in the encoder ISRs. We also create the rotary encoder
 *	object:
 */

volatile	int16_t	freqCount   = 0;			// Encoder cumulative interrupt counter
volatile	int16_t	encoderDir  = 0;			// Copy of last encoder direction
volatile	bool	freqPulse   = false;		// True when we get an encoder interrupt


	Rotary	freqEncdr = Rotary ( FREQ_ENCDR_A, FREQ_ENCDR_B );


/*
 *	In setup:
 */
 
 	attachInterrupt ( digitalPinToInterrupt ( FREQ_ENCDR_A ), FrequencyISR, CHANGE );
	attachInterrupt ( digitalPinToInterrupt ( FREQ_ENCDR_B ), FrequencyISR, CHANGE );


/*
 *	Frequency encoder ISR. It's pretty simple. If the encoder moves, we set a
 *	flag indicating so and make a copy of the direction it moved. Depending on
 *	which way it moved, we increment or decrement the click counter.
 *
 *	If the radio is transmitting (requires the "PTT_PIN" pin to be connected and
 *	the "PTT_LINE" indicator set to '1') we don't allow the frequency to be
 *	changed.
 */

IRAM_ATTR void FrequencyISR ()
{
uint8_t	Result;								// Direction read from encoder

	if ( PTT_LINE  && xmitStatus )			// If PTT connected and transmitter is on
		return;								// Frequency can't be changed

	Result = freqEncdr.process ();			// Otherwise, read the encoder

	if ( Result == DIR_NONE )				// Encoder didn't move (don't think this
		return;								// Is actually possible)

	freqPulse  = true;						// Got a new pulse
	freqCount += Result;					// Adjust the counter
}


/*
 *	This is in the loop() (could be a separate function but inline saves a
 *	function call):
 *
 *	First part of the math overhaul. Originally, he was allowing "count" to be
 *	negative in the "afstp" computation which was causing nonsense numbers. So,
 *	"count" is now going to always be positive in the computation.
 *
 *	But, we need to take into consideration the setting of "F_REV" which tells
 *	us which way the dial rotates when the frequency increases and decreases
 *	and we need to consider which way the encoder actually moved (an option
 *	in the VFO).
 *
 *	"freqDir" will end up being either plus or minus '1' and we will multiply the
 *	result of the "afstp" calculation by that to determine whether to increase
 *	or decrease the operating frequency.
 */

		if ( freqPulse )						// New interrupt?
			freqPulse = false;					// Clear the indicator
 
 
 /*
 *	The "ENCDR_FCTR" is used to reduce the number of virtual interrupts from the
 *	high-speed encoders. If using a mechanical encoder, it should probably be set
 *	to '1' (in "config.h").
 */

		count = freqCount / ENCDR_FCTR;			// Adjusted interrupt count
		freqCount %= ENCDR_FCTR;				// Leave the remainder for next time

		if ( count != 0 )						// If we have a pulse to process
		{
			freqDir = -1;						// Direction will be plus or minus 1

			if ( count > 0 )					// Encoder counter is positive
				freqDir = 1;					// So frequency is increasing

			if ( F_REV == 1 )					// Dial is in reverse (?) mode
				freqDir = -freqDir;				// Reverse frequency direction

			count = abs ( count );				// Always positive now

			incrFactor = lclIncr * count;		// Unaccelerated increment

			if ( ACCELERATE )					// Accelerator on?
			{
				if ( count >= V_TH )			// At or above the threshold?
				{
					if ( ACC_FACTOR )							// Accelerator factor non-zero?
						incrFactor = lclIncr * ACC_FACTOR;		// Yes - use linear acceleration

					else										// If factor is zero, use dynamic mode
					{
						L += 1.0 * (float) ( abs (count) - V_TH );
						incrFactor = lclIncr + ( Racc * L * L );
					}
				}
			}

			afstp = count * incrFactor;							// Positive "afstp"

			afstp = freqDir * ( afstp / lclIncr ) * lclIncr;	// Set direction and eliminate
																// insignificant digits

			if ( L > MaxL )										// Range check
				L = MaxL;
		}										// End of if ( count != 0 )

		else									// The encoder didn't move since last check
		{
			L -= Rdec;							// Subtract deceleration constant

			if ( L < 0 )						// Range check
				L = 0;
		}										// End of if ( count != 0 )

		if ( afstp != 0 )						// Need to update the frequency?
		{


/*
 *	The encoder only changes the receiver frequency which is always in VFO-A regardless
 *	of whether split mode is turned on or not.
 */

			changed.Disp = true;				// The display needs to be updated

			rxFreq = bandData[activeBand].vfoA;	// Get the old receive frequency
			rxFreq += afstp;					// And add the new increment


/*
 *	Another part of the modifications. Instead of making sure the frequency is within
 *	the operating limits of the Si5351, we now limit it to the band edge limits stored
 *	in the "bandData" array.
 */

			if ( rxFreq > bandData[activeBand].topLimit )		// Range checks
				 rxFreq = bandData[activeBand].topLimit;		// Without clarifier factor

			if ( rxFreq < bandData[activeBand].lowLimit )
				 rxFreq = bandData[activeBand].lowLimit;


/*
 *	By dividing the resulting frequency by the "incr" for the current band, the unwanted
 *	low order digits should fall of the edge of the world (it is flat, no?). Then we multiply
 *	by the "incr" which should force the low order digits to be zero.
 */

			rxFreq = rxFreq / lclIncr;
			rxFreq = rxFreq * lclIncr;

			bandData[activeBand].vfoA = rxFreq;		// Update the VFO-A frequency in "bandData"
			CAT.SetFA ( rxFreq );					// And in the CAT control module

			afstp = 0;								// Clear the increment

			if ( CLAR_FA_RESET )					// Reset clarifier on frequency change?
				clarCount = 0;						// Yes
		}											// End of if ( afstp != 0 )	Need to update the frequency
