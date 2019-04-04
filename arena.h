/*
 * arena.h
 *
 *  Created on: 4 Apr 2019
 *      Author: astridhochart
 */

#ifndef ARENA_H_
#define ARENA_H_

void gotoarenacenter(void);
void searchwaste(void);
void gotowall(void);
int16_t aligntothewall(int16_t angle_min);
void goforward(bool pid_or_not, uint16_t distance);
void gotoedge(void);
void walltoright(void);
void turnleft(int16_t angle);
void findwall(void);


#endif /* ARENA_H_ */
