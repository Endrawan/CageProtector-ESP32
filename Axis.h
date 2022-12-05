#ifndef Axis_h
#define Axis_h

class Axis {
  private:
    float _x, _y, _z;
  public:
    Axis();
    Axis(float x, float y, float z);
    float getX();
    float getY();
    float getZ();
    void setXYZ(float x, float y, float z);
    void setX(float x);
    void setY(float y);
    void setZ(float z);
};

#endif