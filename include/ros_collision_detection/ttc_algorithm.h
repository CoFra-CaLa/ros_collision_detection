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

typedef struct {
    float center_pos_x;
    float center_pos_y; 
    float length_x; 
    float length_y; 
    float heading;
    float speed; 
    float acceleration;
} object_motion_t;


class TTCAlgorithm
{
    public:
        virtual float calculateTTC(
            const object_motion_t &subject_object_motion,
            const object_motion_t &perceived_object_motion
        ) = 0; 
        
};

#endif // _TTC_ALGORITM_H_