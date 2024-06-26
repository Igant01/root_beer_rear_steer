/**
 * @file LMT87.cpp
 * 
 * This class uses a binary search to find the nearest
 * temperature value based upon millivolt input.
 * Binary search O(n)= log2(201) ~ 8 search max
 * @author Ian Gant (Igant01)
 * @brief Class helper functions
 * @version 0.1
 * @date 2024-06-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <LMT87.h>

LMT87::LMT87(){
    temp = FAHRENHEIT;
}

void LMT87::setUnit(unit temp){
    if(temp == FAHRENHEIT){
        temp = FAHRENHEIT;
    }
    if(temp == CELCIUS){
        temp = CELCIUS;
    }
}

int LMT87::read(){
    int temp = analogRead(tempPin);
    int mV = temp*3.223;
    if(temp==CELCIUS){
        return binarySearch(mV);
    }
    else{
        int x = binarySearch(mV);
        x = x * (9/5);
        x = x + 32;
        return x;
    }
}




// someone should optimize the part that looks for nearest element
// remove when optimized

int LMT87::binarySearch(int target){    
    int low = 0;
    int mid = 0;
    int high = 202;

    while (low <= high) {
        mid = low + (high - low) / 2;

        if (lookup[mid][1] == target)
            return lookup[mid][0];

        if (lookup[mid][1] > target)
            low = mid + 1;

        else
            high = mid - 1;
    }

    int top = abs(target-lookup[mid-1][1]);    ///compare target to previous 
    int center = abs(target-lookup[mid][1]);   ///compare target to current
    int bottom = abs(target-lookup[mid+1][1]); ///compare target to next

    int min = top;  ///find minimum distance
    if(center<min){
        min = center;
    }
    if(bottom<min){
        min = bottom;
    }
    
    if(min==top){   ///return value at nearest value
        return lookup[mid-1][0];
    }
    if(min==center){
        return lookup[mid][0];
    }
    else{
        return lookup[mid+1][0];
    }
}
