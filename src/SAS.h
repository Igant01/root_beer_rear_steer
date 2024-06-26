/**
 * @file SAS.h
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef SAS_H
#define SAS_H

class SAS {

    private:
        int topMin,topMax,bottomMin,bottomMax; //far extremes(left&right) for both poteniometers
        bool calibratedLeft,calibratedRight;
        int pinA,pinB;
        
    public:
        SAS(int pinA,int pinB);
        void setLeft();
        void setRight();
        bool error();
        int positionNonredundant();

};

#endif