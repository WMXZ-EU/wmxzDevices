// CS5361.h
#ifndef CS53661_H
#define CS5361_H

//extern unsigned int CS5361_overflow1;
//extern unsigned int CS5361_overflow2;


#ifdef __cplusplus
extern "C"{
#endif

extern unsigned int CS5361_overflow1;
extern unsigned int CS5361_overflow2;

void CS5361_setup(int speed);
void CS5361_start(void);

#ifdef __cplusplus
}
#endif

#endif