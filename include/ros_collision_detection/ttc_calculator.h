/**
 * @file ttc_calculator.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-15
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_CALCULATOR_H_
#define _TTC_CALCULATOR_H_


#include <ros/ros.h>


class TTCCalculator
{
private:

public:
    TTCCalculator();
    ~TTCCalculator();
    void calculateAllTTCs();

};

#endif // _TTC_CALCULATOR_H_