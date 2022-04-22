/**
 * @file ttc_algorithm.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _TTC_ALGORITM_H_
#define _TTC_ALGORITM_H_

class TTCAlgorithm
{
    public:
        virtual bool calculateTTC(
            float x_i, float y_i, 
            float length_xi, 
            float length_yi, 
            float heading_i, 
            float speed_i, 
            float acceleration_i, 
            float x_bj, float y_bj, 
            float length_xj, 
            float length_yj, 
            float heading_j, 
            float speed_j, 
            float acceleration_j,
            double *ttc_output
        ) = 0; 

};

#endif // _TTC_ALGORITM_H_