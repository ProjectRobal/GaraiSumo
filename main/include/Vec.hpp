#pragma once

#include <cstdint>



template<typename T>
class Vec4D;

template<typename T>
class Vec2D
{
    public:
    T x;
    T y;

    Vec2D()
    {}

    Vec2D(T x)
    : x(x),y(x)
    {}

    Vec2D(T x,T y)
    : x(x),
    y(y)
    {}

    Vec2D operator+(const Vec2D& vec)
    {
        return Vec2D(this->x+vec.x,this->y+vec.y);
    }

    Vec2D operator-(const Vec2D& vec)
    {
        return Vec2D(this->x-vec.x,this->y-vec.y);
    }

    Vec2D operator*(const Vec2D& vec)
    {
        return Vec2D(this->x*vec.x,this->y*vec.y);
    }

    Vec2D operator*(const T& num)
    {
        return Vec2D(this->x*num,this->y*num);
    }

    Vec2D operator/(const Vec2D& vec)
    {
        return Vec2D(this->x/vec.x,this->y/vec.y);
    }

    Vec2D operator/(const T& num)
    {
        return Vec2D(this->x/num,this->y/num);
    }

    Vec2D& operator+=(const Vec2D& vec)
    {
        this->x+=vec.x;
        this->y+=vec.y;

        return *this;
    }

    Vec2D& operator-=(const Vec2D& vec)
    {
        this->x-=vec.x;
        this->y-=vec.y;  

        return *this;
    }

    Vec2D& operator*=(const Vec2D& vec)
    {
        this->x*=vec.x;
        this->y*=vec.y;

        return *this;
    }

    Vec2D& operator*=(const T& num)
    {
        this->x*=num;
        this->y*=num;

        return *this;
    }

    Vec2D& operator/=(const Vec2D& vec)
    {
        this->x/=vec.x;
        this->y/=vec.y;

        return *this;
    }

    Vec2D& operator/=(const T& num)
    {
        this->x/=num;
        this->y/=num;

        return *this;
    }
};

typedef Vec2D<float> Vec2Df;

typedef Vec2D<double> Vec2Dd;

typedef Vec2D<uint32_t> Vec2Du;

typedef Vec2D<int32_t> Vec2Di;

typedef Vec2D<uint16_t> Vec2D16u;

typedef Vec2D<int16_t> Vec2D16i;


template<typename T>
class Vec3D
{
    public:
    T x;
    T y;
    T z;

    Vec3D()
    {}

    Vec3D(T x)
    : x(x),y(x),z(x)
    {}

    Vec3D(T x,T y,T z)
    : x(x),
    y(y),
    z(z)
    {}

    Vec3D operator+(const Vec3D& vec)
    {
        return Vec3D(this->x+vec.x,this->y+vec.y,this->z+vec.z);
    }

    Vec3D operator-(const Vec3D& vec)
    {
        return Vec3D(this->x-vec.x,this->y-vec.y,this->z-vec.z);
    }

    Vec3D operator*(const Vec3D& vec)
    {
        return Vec3D(this->x*vec.x,this->y*vec.y,this->z*vec.z);
    }

    Vec3D operator*(const T& num)
    {
        return Vec3D(this->x*num,this->y*num,this->z*num);
    }

    Vec3D operator/(const Vec3D& vec)
    {
        return Vec3D(this->x/vec.x,this->y/vec.y,this->z/vec.z);
    }

    Vec3D operator/(const T& num)
    {
        return Vec3D(this->x/num,this->y/num,this->z/num);
    }

    Vec3D& operator+=(const Vec3D& vec)
    {
        this->x=this->x+vec.x;
        this->y=this->y+vec.y;
        this->z=this->z+vec.z;

        return *this;
    }

    Vec3D& operator-=(const Vec3D& vec)
    {
        this->x=this->x-vec.x;
        this->y=this->y-vec.y;   
        this->z=this->z-vec.z;

        return *this;
    }

    Vec3D& operator*=(const Vec3D& vec)
    {
        this->x=this->x*vec.x;
        this->y=this->y*vec.y;
        this->z=this->z*vec.z;

        return *this;
    }

    Vec3D& operator*=(const T& num)
    {
        this->x=this->x*num;
        this->y=this->y*num;
        this->z=this->z*num;

        return *this;
    }

    Vec3D& operator/=(const Vec3D& vec)
    {
        this->x=this->x/vec.x;
        this->y=this->y/vec.y;
        this->z=this->z/vec.z;

        return *this;
    }

    Vec3D& operator/=(const T& num)
    {
        this->x=this->x/num;
        this->y=this->y/num;
        this->z=this->z/num;

        return *this;
    }

    void rotate(const Vec4D<T>& q)
    {
        Vec4D<T> p(0,q.x,q.y,q.z);

        p=q.getProduct(p);

        p=p.getProduct(q.getConjugate());

        this->x=p.x;
        this->y=p.y;
        this->z=p.z;
    }
};

typedef Vec3D<float> Vec3Df;

typedef Vec3D<double> Vec3Dd;

typedef Vec3D<uint32_t> Vec3Du;

typedef Vec3D<int32_t> Vec3Di;

typedef Vec3D<uint16_t> Vec3D16u;

typedef Vec3D<int16_t> Vec3D16i;


template<typename T>
class Vec4D
{
    public:
    T x;
    T y;
    T z;
    T w;

    Vec4D()
    {}

    Vec4D(T x)
    : x(x),y(x),z(x),w(x)
    {}

    Vec4D(T x,T y,T z,T w)
    : x(x),
    y(y),
    z(z),
    w(w)
    {}

    Vec4D operator+(const Vec4D& vec)
    {
        return Vec4D(this->x+vec.x,this->y+vec.y,this->z+vec.z,this->w+vec.w);
    }

    Vec4D operator-(const Vec4D& vec)
    {
        return Vec4D(this->x-vec.x,this->y-vec.y,this->z-vec.z,this->w-vec.w);
    }

    Vec4D operator*(const Vec4D& vec)
    {
        return Vec4D(this->x*vec.x,this->y*vec.y,this->z*vec.z,this->w*vec.w);
    }

    Vec4D operator*(const T& num)
    {
        return Vec4D(this->x*num,this->y*num,this->z*num,this->w*num);
    }

    Vec4D operator/(const Vec4D& vec)
    {
        return Vec4D(this->x/vec.x,this->y/vec.y,this->z/vec.z,this->w/vec.w);
    }

    Vec4D operator/(const T& num)
    {
        return Vec4D(this->x/num,this->y/num,this->z/num,this->w/num);
    }

    Vec4D& operator+=(const Vec4D& vec)
    {
        this->x+=vec.x;
        this->y+=vec.y;
        this->z+=vec.z;
        this->w+=vec.w;

        return *this;
    }

    Vec4D& operator-=(const Vec4D& vec)
    {
        this->x-=vec.x;
        this->y-=vec.y;   
        this->z-=vec.z;
        this->w-=vec.w;

        return *this;
    }

    Vec4D& operator*=(const Vec4D& vec)
    {
        this->x*=vec.x;
        this->y*=vec.y;
        this->z*=vec.z;
        this->w*=vec.w;

        return *this;
    }

    Vec4D& operator*=(const T& num)
    {
        this->x*=num;
        this->y*=num;
        this->z*=num;
        this->w*=num;

        return *this;
    }

    Vec4D& operator/=(const Vec4D& vec)
    {
        this->x/=vec.x;
        this->y/=vec.y;
        this->z/=vec.z;
        this->w/=vec.w;

        return *this;
    }

    Vec4D& operator/=(const T& num)
    {
        this->x/=num;
        this->y/=num;
        this->z/=num;
        this->w/=num;

        return *this;
    }

    Vec4D getConjugate() const
    {
        return Vec4D(this->w,-this->x,-this->y,-this->z);
    }

    Vec4D getProduct(const Vec4D& vec) const
    {
        return Vec4D(
            vec.w*this->w - vec.x*this->x - vec.y*this->y - vec.z*this->z,
            vec.x*this->w + vec.w*this->x + vec.z*this->y - vec.y*this->z,
            vec.y*this->w - vec.z*this->x + vec.w*this->y + vec.x*this->z,
            vec.z*this->w + vec.y*this->x - vec.x*this->y + vec.w*this->z
        );
    }
};

typedef Vec4D<float> Vec4Df;

typedef Vec4D<double> Vec4Dd;

typedef Vec4D<uint32_t> Vec4Du;

typedef Vec4D<int32_t> Vec4Di;

typedef Vec4D<uint16_t> Vec4D16u;

typedef Vec4D<int16_t> Vec4D16i;
