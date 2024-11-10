# Data Link Layer Protocol for File Transmission

## Overview

This project implements a Data Link Layer protocol aimed at ensuring reliable communication between two systems connected via an RS-232 serial cable. The primary objective is to transfer a file from a transmitter computer to a receiver computer, emulating real-world data link protocol functionalities.

## Goals

- **Reliable File Transfer**: Develop a protocol with transmitter and receiver functionality for reliable file transmission across a serial connection.
- **Protocol Functions**: Implement key data link layer functionalities such as framing, error control, and flow control.
- **Layered Architecture**: Maintain a clean separation between the protocol and application layers by designing an API for communication.

## Features

- **Connection Setup and Termination**: Use specific control frames (e.g., SET and DISC) to establish and terminate connections.
- **Stop-and-Wait Protocol**: Implemented for flow control, ensuring only one frame is in transit at a time.
- **Error Detection and Handling**: Utilize frame numbering, acknowledgment frames, and retransmission to ensure reliable delivery.
- **Byte Stuffing**: A byte-stuffing mechanism is used to avoid misinterpreting frame delimiters within the payload.
  
## Protocol Phases

1. **Connection Establishment**: Initiated by the transmitter, which sends a SET frame. The receiver responds with a UA frame to confirm the connection.
2. **Data Transfer**: Files are segmented into packets and transmitted sequentially. The Stop-and-Wait protocol ensures each packet is acknowledged by the receiver.
3. **Connection Termination**: After file transfer completion, a DISC frame is exchanged to gracefully close the connection.

## Protocol Interface

The following functions expose the protocol to the application layer:

- `llopen()`: Initializes the connection parameters (e.g., serial port, baud rate) and establishes the connection.
- `llwrite()`: Transmits data packets by encapsulating them within frames.
- `llread()`: Receives frames, verifies integrity, and extracts data.
- `llclose()`: Closes the connection and outputs transmission statistics (e.g., frame counts, retransmissions).

## Code Structure

- **main.c**: Handles the application layer and controls the file transfer process by invoking protocol functions.
- **serial_port.c**: Manages the serial port configuration, setting parameters like baud rate, character size, and parity.
- **link_layer.c**: Implements the llopen, llread, llwrite and llclose functions to be used by the application layer. Also, implements auxiliary functions to handle the execution of the protocol phases and the features established.
- **application_layer.c**: Uses the interface provided to write and read packets into the selected file.

## Development Environment

- **Operating System**: Linux
- **Programming Language**: C
- **Hardware**: RS-232 serial cable connection

## Testing and Evaluation

Testing was conducted over five milestones, including basic string exchange, control frames, Stop-and-Wait, timer and retransmission, and application layer integration. Performance metrics, such as frame error rate (FER) and propagation delay, were analyzed to evaluate protocol efficiency.

## References

- **RS-232 Serial Communication**: For reliable communication between connected systems.
- **UNIX Serial Port Drivers**: To interface with hardware for low-level serial communication.

## Contributors

- Leonardo Garcia (up202200041@up.pt)
- Maria Eduarda Rabelo (up202000130@up.pt)


