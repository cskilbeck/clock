#pragma once

//////////////////////////////////////////////////////////////////////

// 16 bit random number
int rand16();
byte rand8();

// set the seed
void seed_rand(int);

//////////////////////////////////////////////////////////////////////

struct Random
{
    uint32 seed;

    Random(int start_seed);
    int next();
    byte next_byte();
    bool boolean();
};
