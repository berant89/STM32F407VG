	AREA KALMANCODE, CODE ;Define area to be code
	EXPORT kalman ;Declare symbol to be exported
	
kalman
	;r0 contains the address to the Kalman Filter struct and s0 contains the float measurement.
	VPUSH {s1-s2} ;Push two scratch registers.
	
	;PART 1: p = p + q
	vldr s1, [r0] ;Load float q
	vldr s2, [r0, #12] ;Load float p
	vadd.F32 s2, s1 ;p = p + q we need to do .F32 to indicate single precision addition
	
	;PART 2: k = p/(p + r)
	vldr s1, [r0, #4] ;Load float r since q is not used again.
	vadd.F32 s1, s2 ;p+r store the result in s1
	vdiv.F32 s1, s2, s1 ;k = p/(p+r) store the result in s1 which is now k
	
	;PART 3: p = (1 - k) * p
	;At this point s1 = k and s2 = p
	vmls.F32 s2, s2, s1; p * (1 - k) = p - p*k store into s2
	vstr s2, [r0, #12] ;Store p since we will not need it anymore
	
	;PART 4: x = x + k * (measurement - x)
	;At this point s1 = k and s2 is scratch
	vldr s2, [r0, #8] ;Load x into s2
	vsub.F32 s0, s2; measurement - x store it into s0
	vmul.F32 s0, s1 ; k * (measurement - x) store into s0
	vadd.F32 s2, s0; x + k * (measurement - x) store into s2
	
	;Now store s1 = k and s2 = x
	vstr s1, [r0, #16]
	vstr s2, [r0, #8]
	
	vmov.F32 s0, s2 ;Move the result of s2 which is x into the return register s0.
	
	VPOP {s1-s2}
	BX R14
	END
	