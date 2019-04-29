/*
 * arena.h
 *
 *  Created on: 4 Apr 2019
 *      Author: Astrid Hochart & Sven Borden
 *      Version 0.8
 */

#ifndef ARENA_H_
#define ARENA_H_


void init_arena(void);
/*- INIT POSITION OF ROBOT TO ARENA CENTER -*/
void gotoarenacenter(void);

/*- INFINITE LOOP SEARCH WASTE AND CLEAN ARENA -*/
void search_waste(void);


#endif /* ARENA_H_ */
