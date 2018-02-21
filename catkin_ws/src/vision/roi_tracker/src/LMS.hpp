#ifndef LMS_HPP_INCLUDED
#define LMS_HPP_INCLUDED

//#include "opencv2/nonfree/features2d.hpp"
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

/** LMS FILTER   **/
    class LMS
    {
    public:
        LMS();
        LMS(int SizeW,float pEpsilon,float mu);

        void UpdateW(float xn);
        float Stimate();
        float GetError();
        float Get_ye();

    private:
        int Nw;
        int n;
        float W[10];
        float xn_Last[10];

        float ye;
        float u;
        float un;
        float Error;
        float Epsilon;
    };
/********/


#endif // LMS_HPP_INCLUDED
