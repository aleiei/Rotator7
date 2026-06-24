//vec.cpp - Vector library.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//For more information please visit http://www.sarcnet.org
//Released under the GNU General Public License.
//Provides vector functions using cartesian coordinates 
#include "vec.h"
#include <Arduino.h>

//Public methods

Vec::Vec() {
  //Constructor
  i = 0;
  j = 0;
  k = 0;
}

Vec::Vec(float I, float J, float K) {
  //Constructor
  i = I;
  j = J;
  k = K;
}

Vec Vec::cross(const Vec &b) const {
  //Returns the vector cross product of two vectors.
  return Vec(j * b.k - k * b.j, k * b.i - i * b.k, i * b.j - j * b.i);
}

float Vec::dot(const Vec &b) const {
  //Returns the scalar dot product of two vectors.
  return i * b.i + j * b.j + k * b.k;
}

float Vec::mod() const {
  //Returns the scalar modulus of a vector.
  return sqrt(i * i + j * j + k * k);
}

Vec Vec::unit() const {
  //Returns the unit vector of a vector.
  float modulus = mod();
  if (modulus <= 0.000001f) {
    return Vec(0, 0, 0);
  }
  return Vec(i / modulus, j / modulus, k / modulus);
}

Vec Vec::neg() const {
  //Returns the vector negative of a vector.
  return Vec( -i, -j, -k);
}
