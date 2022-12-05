#include "Axis.h"

Axis::Axis() {
  setXYZ(0, 0, 0);
}

Axis::Axis(float x, float y, float z) {
  setXYZ(x, y, z);
}

float Axis::getX() {
  return _x;
}

float Axis::getY() {
  return _y;
}

float Axis::getZ() {
  return _z;
}

void Axis::setXYZ(float x, float y, float z) {
  _x = x;
  _y = y;
  _z = z;
}

void Axis::setX(float x) {
  _x = x;
}

void Axis::setY(float y) {
  _y = y;
}

void Axis::setZ(float z) {
  _z = z;
}