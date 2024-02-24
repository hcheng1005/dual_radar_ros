#pragma once

#define MAX_RADAR_NUMS          (40000)
#define RADAR_FEATURES_NUMS     (5)

struct radarPoint
{
    float x;
    float y;
    float z;
    float rcs;
    float doppler;
};

struct Bndbox {
    float x; 
    float y;
    float z;
    float l;
    float w;
    float h;
    float vx;
    float vy;
    float rt;
    int id;
    float score;
    int type;
    Bndbox(){};
    Bndbox(float x_, float y_, float z_, float l_, float w_, float h_, float vx_, float vy_, float rt_, int id_, float score_, int type_)
        : x(x_), y(y_), z(z_), l(l_), w(w_), h(h_), vx(vx_), vy(vy_), rt(rt_), id(id_), score(score_), type(type_) {}
};
