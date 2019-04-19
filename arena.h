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
void goforward(uint8_t pid_or_not, float distance, uint8_t speed, uint8_t stop_dist);
void gotoedge(void);
void walltoright(void);
void turnleft(int16_t angle);
int16_t  findwall(void);
uint16_t wasteinsight(int16_t angle);
void pickupwaste(uint16_t angle_waste);
void goback(uint16_t frequence, uint8_t speed);
void shoveldown(void);
void shovelup(void);
void throwwaste(void);
void init_arena(void);
uint16_t function_distance_arena(uint16_t angle_robot);
void go_to_another_edge(void);

#endif /* ARENA_H_ */
