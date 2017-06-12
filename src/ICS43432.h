//Copyright 2017 by Walter Zimmer
// Version 18-05-17
//
#ifndef ICS42432_H
#define ICS43432_H

class c_ICS43432
{
  public:
  void init(int32_t *buffer, uint32_t nbuf);
  void start(void);
  void stop(void);
};

#endif
