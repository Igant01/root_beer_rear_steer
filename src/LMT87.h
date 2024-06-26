/**
 * @file LMT87.h
 * @author Ian Gant (Igant01)
 * @brief Temperature header file
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef LMT87_H
#define LMT87_H
#include <LMT87_lookup.h>
#define tempPin A5   ///onboard temperature

enum unit{
    CELCIUS,
    FAHRENHEIT,
};

class LMT87 {

    private:
        unit temp;
        int binarySearch(int target);
        
    public:
        LMT87();
        void setUnit(unit temp);
        int read();


};

#endif