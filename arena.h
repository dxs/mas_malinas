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
void aligntothewall(int16_t angle_min);
void goforward(bool pid_or_not, float distance);
void gotoedge(void);
void walltoright(void);
void turnleft(int16_t angle);
int16_t  findwall(void);
bool wasteinsight(int16_t angle);
void pickupwaste(void);
void goback(float distance);
void shoveldown(void);
void shovelup(void);
void throwwaste(void);

#endif /* ARENA_H_ */
