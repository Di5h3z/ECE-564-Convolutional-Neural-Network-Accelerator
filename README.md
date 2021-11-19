# ECE-564-Convolutional-Neural-Network-Accelerator
A Verilog implementation of a CNN accelerator.

Abstract:
This piece of hardware is a single stage binary convolution accelerator that can accept 3
different sized input matrixes. The input, output and weights are all stored in separate
SRAMs and the outputs are left zero padded. Convolution is achieved by calculating the
XNOR of the weight and input and bit counting the result to determine the sign of the
desired output bit. Doing this in parallel allows the module to pipeline inputs and operate
with a new memory input every clock cycle. A FSM controls the inputs and outputs and
allows the module to process multiple input matrixes until it encounters a matrix
terminated with 0x00FF. Each matrix is preempted with an xdim and ydim indicating
how many columns and rows the matrix has respectively. The weights are stored in the
second address of the weight SRAM and are a 3x3 matrix. Using this approach allowed
memory bandwidth to be well utilized.
