#ifndef BOARDS_H
#define BOARDS_H

#define BOARD_UNKNOWN -1

#define BOARD_EINSY_RAMBO        304  

#define MB(board) (MOTHERBOARD==BOARD_##board)

#endif //__BOARDS_H
