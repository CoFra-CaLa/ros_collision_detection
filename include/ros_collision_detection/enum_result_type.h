/**
 * @file enum_result_type.h
 * @author Michael Wittmann (miw2006@thi.de)
 * @brief Header for the Enumeration of the collision warning Result Type. 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _ENUM_RESULT_TYPE_H_
#define _ENUM_RESULT_TYPE_H_


/**
 * @brief Enumeration that defines different collision warning levels.
 * 
 */
enum ResultType
{
    RESULT_IGNORE = 0,
    RESULT_CLOSE_MONITORING = 1,
    RESULT_WARNING = 2,
    RESULT_ALERT = 3,
};

#endif  // _ENUM_RESULT_TYPE_H_

