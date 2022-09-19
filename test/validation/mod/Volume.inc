NEURON {
	GLOBAL Vi, Vcell, Vup, Vrel, S
}

PARAMETER {
	Vcell 	= 0.0157  	(mm3)  	: ok
	Vi 		= 0.014137	(mm3)	: Intracellular Volume
	Ve		= 0.00157	(mm3)	: ok

:	Vc 		= 	(cm3)		: cleft space
	Vup 	= 0.0007069 	(mm3)		: SR uptake Cptmt Vol
	Vrel 	= 0.0002827		(mm3)		: SR release Cptmt Vol

:	S = 0.6283			(um2)		: Surface pour Vi
	S = 0.063 			(cm2)
}
