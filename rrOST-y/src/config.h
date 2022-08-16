/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             rrOST 0-045_noair                              */
/*                                                                            */
/*                  (C) Copyright 2021 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* config.h / 0-045_noair                                                     */
/*----------------------------------------------------------------------------*/
//
// Configuration file for auRIx package - global settings.
//
/*----------------------------------------------------------------------------*/

#ifndef __CONFIG_H__
#define __CONFIG_H__


/*----------------------------------------------------------------------------*/

namespace rrOST
{


/*----------------------------------------------------------------------------*/

#define s__STANDARD_INDENT                "    "
    
#define s__DEFAULT_N_PARALLEL_THREADS          4
#define s__DEFAULT_RANDOM_WALK_LENGTH    1048576

#define s__DEFAULT_WRAP_LINE_LENGTH           80
#define s__CONVERSION_BUFFER_SIZE            128

#define s__VERIFICATION_SIMULATION_STEP    0.001

#define s__MAX_OPTIMIZATION_ITERATIONS      1024
#define s__MAX_OPTIMIZATION_RESTARTS          16
    


/*----------------------------------------------------------------------------*/

} // namespace rrOST

#endif /* __CONFIG_H__ */
