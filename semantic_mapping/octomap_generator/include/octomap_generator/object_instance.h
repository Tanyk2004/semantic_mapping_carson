#ifndef OBJECT_INSTANCE_H
#define OBJECT_INSTANCE_H

#include <octomap/octomap.h>

struct ObjectInstance {

    int label;
    octomap::point3d position;
    octomap::point3d size;

    ObjectInstance(label, position, size) : label(label), position(position), size(size) {}

};