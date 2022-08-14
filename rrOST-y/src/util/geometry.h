/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             rrOST 0-036_noair                              */
/*                                                                            */
/*                  (C) Copyright 2021 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* geometry.h / 0-036_noair                                                   */
/*----------------------------------------------------------------------------*/
//
// Geometric utilities and routines.
//
/*----------------------------------------------------------------------------*/


#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <math.h>

#include "result.h"


using namespace rrOST;


/*----------------------------------------------------------------------------*/

namespace rrOST
{



    
/*----------------------------------------------------------------------------*/
// Various geometric definitions
    
#define sDEG_2_RAD(deg)        ((M_PI * (deg)) / 180.0)
#define sRAD_2_DEG(rad)        ((180.0 * (rad)) / M_PI)



    
/*============================================================================*/
// Global functions

    

/*----------------------------------------------------------------------------*/

} // namespace rrOST

#endif /* __GEOMETRY_H__ */
