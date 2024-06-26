/**
 * @file current_sense.h
 * @author Ian Gant (Igant01)
 * @brief 
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef CURRENTSENSE_H
#define CURRENTSENSE_H

class current_sense {

    private:
        int pin;

    public:
        current_sense(int definePin);
        float read();

};

#endif