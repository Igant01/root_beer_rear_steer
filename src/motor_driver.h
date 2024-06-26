/**
 * @file motor_driver.h
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#define DirA 2    //out direction a
#define DirB 3    //out direction b
#define EN 4      //active low

class motor_driver {

    private:
        bool ENABLE;

    public:
        motor_driver();
        void enable();
        void disable();
        void set(int input);

};

#endif