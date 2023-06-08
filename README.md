MOD2C was part of https://github.com/neuronsimulator/nrn but is now deprecated for https://github.com/BlueBrain/nmodl

# MOD2C
> NMODL to C converter for CoreNEURON

[![Build Status](https://travis-ci.org/BlueBrain/mod2c.svg?branch=master)](https://travis-ci.org/BlueBrain/mod2c) [![MOD2C CI](https://github.com/BlueBrain/mod2c/workflows/MOD2C%20CI/badge.svg)](https://github.com/BlueBrain/mod2c/actions?query=workflow%3A%22MOD2C+CI%22)

MOD2C is NMODL to C adapted for [CoreNEURON simulator](https://github.com/BlueBrain/CoreNEURON).
More information about NMODL can be found [here](https://www.neuron.yale.edu/neuron/static/docs/refman/nocmodl.html).

This version supports GPU code generation using OpenACC. Use latest
version of CoreNeuron.

# Requirements
* [CMake 2.8.9+](https://cmake.org/)
* [Flex](http://flex.sourceforge.net)
* [Bison](https://www.gnu.org/software/bison/)

# Installation

Once you clone the repository, you can build mod2c using CMake as follows:

```bash
cd mod2c
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/path/to/install/directory
make
make install
```

Note that mod2c is used only during the compilation phase of the CoreNEURON simulator
and hence should be built only for the front-end/login nodes in cross-compile environments
like BG-Q, Cray, Intel MIC etc.


# Funding & Acknowledgment
 
The development of this software was supported by funding to the Blue Brain Project, a research center of the École polytechnique fédérale de Lausanne (EPFL), from the Swiss government's ETH Board of the Swiss Federal Institutes of Technology.
 
Copyright © 2015-2022 Blue Brain Project/EPFL


## License
* See LICENSE.txt
* See [NEURON](https://www.neuron.yale.edu/neuron/)
* The mechanisms and test datasets appearing in the CoreNeuron repository are subject to separate licenses.
  More information is available on the NMC portal website [NMC portal](https://bbp.epfl.ch/nmc-portal/copyright),
  the specific licenses are described in the ME-type model packages downloadable from that website.
