/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             rrOST 0-050_noair                              */
/*                                                                            */
/*                  (C) Copyright 2021 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* motion_plan_main.h / 0-050_noair                                           */
/*----------------------------------------------------------------------------*/
//
// Robot Motion Planning - main program.
//
// Motion planning for the RR1 robotic arm and related robots.
//
/*----------------------------------------------------------------------------*/


#ifndef __MOTION_PLAN_MAIN_H__
#define __MOTION_PLAN_MAIN_H__

#include "config.h"
#include "compile.h"
#include "defs.h"
#include "version.h"

using namespace std;


/*----------------------------------------------------------------------------*/

namespace rrOST
{


/*----------------------------------------------------------------------------*/

    struct sCommandParameters
    {
	sCommandParameters();
        /*--------------------------------*/

	sInt_32 m_seed;
    };


/*----------------------------------------------------------------------------*/

    void print_IntroductoryMessage(void);
    void print_ConcludingMessage(void);
    void print_Help(void);
    
    sResult parse_CommandLineParameter(const sString &parameter, sCommandParameters &parameters);
    sResult plan_Motion(const sCommandParameters &parameters);    


/*----------------------------------------------------------------------------*/

} // namespace rrOST


#endif /* __MOTION_PLAN_MAIN_H__ */
