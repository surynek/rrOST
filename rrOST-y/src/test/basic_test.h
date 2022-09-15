/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             rrOST 0-058_noair                              */
/*                                                                            */
/*                  (C) Copyright 2021 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* basic_test.h / 0-058_noair                                                 */
/*----------------------------------------------------------------------------*/
//
// Basic initial test.
//
/*----------------------------------------------------------------------------*/


#ifndef __BASIC_TEST_H__
#define __BASIC_TEST_H__

#include "defs.h"
#include "result.h"


/*----------------------------------------------------------------------------*/

namespace rrOST
{

void print_Introduction(void);
int test_basic_1(void);
int test_basic_2(void);
int test_basic_3(void);
int test_basic_4(void);
int test_basic_5(void);
int test_basic_5_RR1_rev1(void);

void print_RR1_rev1_steps(const s3DRobot &robot);



/*----------------------------------------------------------------------------*/

} // namespace rrOST

#endif /* __BASIC_TEST_H__ */

