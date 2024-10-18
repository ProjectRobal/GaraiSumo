#pragma once



template<typename T>
class PID
{
    private:

    T _p;
    T _i;
    T _d;

    T last_output;
    T integral;
    T max_integral;

    T max;
    T min;

    bool filtr;

    double _dt;

    public:

    PID()
    {
        filtr=false;
        _p=0;
        _i=0;
        _d=0;
        last_output=0;
        _dt=0.f;
        integral=0;
        max_integral=25;
    }

    PID(T p,T i,T d)
    : PID()
    {
        _p = p;
        _i = i;
        _d = d;
    }

    void reset()
    {
        this->last_output = 0;
        this->integral = 0;
    }   

    void setParams(T p,T i=0,T d=0)
    {
        _p=p;
        if(i)
        {
            setI(i);
        }
        if(d)
        {
            setD(d);
        }

        this->reset();
    }

    void setMax(T _max)
    {
        filtr=true;
        max=_max;
    }

    void setMin(T _min)
    {
        filtr=true;
        min=_min;
    }

    void setP(T p)
    {
        _p=p;

        this->reset();
    }

    void setI(T i)
    {
        _i=i;

        this->reset();
    }

    void setD(T d)
    {
        _d=d;

        this->reset();
    }

    T P() const
    {
        return _p;
    }

    T I() const
    {
        return _i;
    }

    T D() const
    {
        return _d;
    }

    void setTimeStep(double dt)
    {
        _dt=dt;

        this->reset();
    }

    const double& TimeStep() const
    {
        return _dt;
    }

    T step(T x,double dt)
    {
        integral += _i*x*dt;

        if( integral > max_integral )
        {
            integral = max_integral;
        }
        else if( integral < -max_integral )
        {
            integral = -max_integral;
        }

        last_output=filter(_p*x + integral + _d*((x-last_output)/dt));

        return last_output;
    }

    T step(T x)
    {
        return step(x,_dt);
    }

    T filter(T x)
    {   
        if(!filtr)
        {
            return x;
        }

        if(x>max)
        {
            return max;
        }

        if(x<min)
        {
            return min;
        }

        return x;
    }

};

