# FriendlessFoosball-Build18-2021

Commands are sent over USB Serial.  The first byte of a command determines which motor and what command is being sent.  The 2 least significant bits of this byte are used to determine which motor (0,1,2,3 corresponds to X,Y,Z,A).  The 6 most significant bits are used to determine which command is used as follows:

 - 000000: move motor to 0 position
 - 000001: set current position of motor to be 0 position
 - 000010: set speed
 - 000011: set position
 - 000100: get position

If set speed or set position is sent, the Arduino expects the next 2 bytes to be the speed/position in 8-bit signed integer format.  The least significant byte should be sent first.

If get position is received by the Arduino, it will reply with 2 bytes corresponding to the position as an 8-bit signed integer value.  The least significant byte is sent first.

Any transmission must be terminated with a newline (0x0A) to tell the Arduino that the transmission is complete.
