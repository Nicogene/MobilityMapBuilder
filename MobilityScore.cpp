//
// Created by nick on 19/08/15.
//

#include "MobilityScore.h"
#include <iostream>
#include <stdio.h>

double MobilityScore(double r, double s){
    double ms=1;
    if(s>40 || r>2.0){
        ms=0;
        return ms;
    }
    else if(s<1 && r<1.2){
        ms=1;
        return  ms;}

    if(s<40 && s>25) {
        ms= ms-0.75;

        //std::cout<<"ciao "<<ms<<std::endl;
    }
    if(r<2.0 && r>1.8){
        ms= ms-0.75;
        //std::cout<<"ciao "<<ms<<std::endl;

    }
    if(s<25 && s>15) {
        ms= ms-0.5;
        //std::cout<<"ciao "<<ms<<std::endl;
    }
    if(r<1.8 && r>1.6){
        ms= ms-0.5;
    }
    if(s<15 && s>1) {
        ms= ms-0.25;
    }
    if(r<1.6 && r>1.4){
        ms= ms-0.25;
    }

    if (ms<0)
        ms=0;

    return ms;

}
