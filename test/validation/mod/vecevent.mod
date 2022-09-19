NEURON {
	THREADSAFE
	ARTIFICIAL_CELL VecStim
}

PARAMETER {
    xpos = 0
}
 
PROCEDURE position(a) { 
       xpos = a
}
