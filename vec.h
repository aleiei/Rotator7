//vec.h - Vector library.
//Copyright (c) 2015-2018 Julie VK3FOWL and Joe VK3YSP
//For more information please visit http://www.sarcnet.org
//Released under the GNU General Public License.
//Provides vector functions using cartesian coordinates 
#ifndef VEC_H
#define VEC_H

class Vec {
  public:
    Vec();
    Vec(float I, float J, float K);
    Vec cross(const Vec &b) const;
    float dot(const Vec &b) const;
    float mod() const;
    Vec unit() const;
    Vec neg() const;
    float i, j, k; //Vector type comprising three orthogonal components.
  private:
};
#endif
