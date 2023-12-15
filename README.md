# Link Layer Transfer Protocol

This project implements a transfer protocol that operates at the link layer. 

## Overview

The link layer transfer protocol is responsible for moving frames from one node to another, and handling errors that occur along the way.
This protocol can handle noise, lost frames, and frames that are received out of order.
The application layer implemented on top of the link layer is responsible for handling the reassembly of the frames into the original file.

## Files

- `include/link_layer.h`: This file contains the header definitions for the link layer.
- `src/link_layer.c`: This file contains the implementation of the link layer.
- `src/frame.c`: Contains helper functions to handle frames.

## Building the Project

You can build the project using the provided Makefile.

## To run the project locally

1. Run `make` to build the project.
2. Run `./bin/cable` to simulate the cable between two nodes.
3. Run `make run_rx` to start listening for frames on one node.
4. Run `make run_tx` to start transmitting frames on the other node.
5. The penguin.gif file should transmit from one node to the other.
