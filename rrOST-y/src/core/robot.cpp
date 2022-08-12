/*============================================================================*/
/*                                                                            */
/*                                                                            */
/*                             rrOST 0-030_noair                              */
/*                                                                            */
/*                  (C) Copyright 2021 - 2022 Pavel Surynek                   */
/*                                                                            */
/*                http://www.surynek.net | <pavel@surynek.net>                */
/*       http://users.fit.cvut.cz/surynek | <pavel.surynek@fit.cvut.cz>       */
/*                                                                            */
/*============================================================================*/
/* robot.cpp / 0-030_noair                                                    */
/*----------------------------------------------------------------------------*/
//
// Robot (model) related data structures and functions.
//
/*----------------------------------------------------------------------------*/


#include <stdio.h>
#include <math.h>

#include "defs.h"
#include "result.h"

#include "common/types.h"
#include "core/robot.h"


using namespace rrOST;




/*----------------------------------------------------------------------------*/

namespace rrOST
{



    
/*============================================================================*/
// s2D class
/*----------------------------------------------------------------------------*/
    
    void s2D::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void s2D::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(%.3f,%.3f)\n", indent.c_str(), x, y);
    }


    
    
/*============================================================================*/
// s3D class
/*----------------------------------------------------------------------------*/
    
    void s3D::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void s3D::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s(%.3f,%.3f,%.3f)\n", indent.c_str(), x, y, z);
    }


    


/*============================================================================*/
// s2DRobot class
/*----------------------------------------------------------------------------*/

    void s2DRobot::Link::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void s2DRobot::Link::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sLink: (end.x = %.3f, end.y = %.3f)\n", indent.c_str(), end.x, end.y);
    }


/*----------------------------------------------------------------------------*/
    
    void s2DRobot::Joint::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);
    }

    
    void s2DRobot::Joint::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sJoint: (rotation = %.3f)\n", indent.c_str(), rotation);
    }


/*----------------------------------------------------------------------------*/
    
    void s2DRobot::Configuration::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);	
    }

    
    void s2DRobot::Configuration::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%sConfiguration: (|dimension| = %ld)\n", indent.c_str(), Rotations.size());

	for (Rotations_vector::const_iterator rotation = Rotations.begin(); rotation != Rotations.end(); ++rotation)
	{
	    fprintf(fw, "%s%srotation: %.f\n", indent.c_str(), s_INDENT.c_str(), *rotation);	    
	}
    }
    

/*============================================================================*/
    
    s2DRobot::~s2DRobot()
    {
	for (Joints_vector::const_iterator joint = Joints.begin(); joint != Joints.end(); ++joint)
	{
	    delete *joint;
	}
	for (Links_vector::const_iterator link = Links.begin(); link != Links.end(); ++link)
	{
	    delete *link;
	}		
    }
    
    
/*----------------------------------------------------------------------------*/

    void s2DRobot::add_Joint(sDouble rotation)
    {
	sASSERT(Joints.size() == Links.size());

	Joint *joint = new Joint(rotation);
	Joints.push_back(joint);

	if (Joints.size() == 1)
	{
	    base_joint = Joints[0];
	    base_joint->prev = NULL;
	    base_joint->next = NULL;
	}
	else
	{
	    sInt_32 last = Joints.size() - 1;
	    Joints[last]->prev = Joints[last - 1];
	    Joints[last - 1]->next = Joints[last];
	}	
    }

    
    void s2DRobot::add_Link(s2D end)
    {
	sASSERT(!Joints.empty());
	    
	s2DRobot::Link *link = new s2DRobot::Link(end);
	Links.push_back(link);

	sInt_32 last = Joints.size() - 1;
	Joints[last]->child = link;
    }

    
    void s2DRobot::add_Link(sDouble length)
    {
	sASSERT(!Joints.empty());
	
	s2DRobot::Link *link = new s2DRobot::Link(length);
	Links.push_back(link);
	
	sInt_32 last = Joints.size() - 1;
	Joints[last]->child = link;	
    }


    void s2DRobot::attach_Constraint(Joint *joint, const Constraint &constraint)
    {
	joint->constraint = new Constraint(constraint);
    }

    
/*----------------------------------------------------------------------------*/
    
    void s2DRobot::save_RobotConfiguration(Configuration &configuration) const
    {
	sASSERT(base_joint != NULL);
	
	configuration.Rotations.clear();
	const Joint *joint = base_joint;	

	while (joint != NULL)
	{
	    configuration.Rotations.push_back(joint->rotation);
	    joint = joint->next;	    
	}	
    }

    
    void s2DRobot::restore_RobotConfiguration(const Configuration &configuration)
    {
	sASSERT(base_joint != NULL);
	
	Rotations_vector::const_iterator rotation = configuration.Rotations.begin();
	Joint *joint = base_joint;

	while (joint != NULL)
	{
	    sASSERT(rotation != configuration.Rotations.end());
	    
	    joint->rotation = *rotation++;
	    joint = joint->next;	    
	}		
    }


    void s2DRobot::randomize_RobotConfiguration(void)
    {
	Joint *joint = base_joint;

	while (joint != NULL)
	{
	    joint->rotation = 2 * M_PI * (rand() / (sDouble)RAND_MAX);
	    joint = joint->next;
	}
    }
	

/*----------------------------------------------------------------------------*/
    

    void s2DRobot::calc_EndPosition(const s2D &origin, s2D &end) const
    {
	sASSERT(base_joint != NULL);
	
	calc_EndPosition(base_joint, origin, end);
    }

    
    void s2DRobot::calc_EndPosition(const Joint *_base_joint, const s2D &origin, s2D &end) const	
    {
	end.x = origin.x;
	end.y = origin.y;

	const Joint *joint = _base_joint;
	sDouble rotation = 0.0;
	
	while (joint != NULL)
	{
	    Link *link = joint->child;
	    rotation += joint->rotation;

	    sDouble cosr = cos(rotation);
	    sDouble sinr = sin(rotation);	    
	    
	    end.x += cosr * link->end.x - sinr * link->end.y;	
	    end.y += sinr * link->end.x + cosr * link->end.y;

	    joint = joint->next;
	}
    }


    sDouble s2DRobot::calc_PositionDifference(const s2D &origin, const s2D &position) const
    {
	s2D end;
	calc_EndPosition(origin, end);

	sDouble dx = end.x - position.x;
	sDouble dy = end.y - position.y;

	return (dx * dx + dy * dy);
    }

    
    sDouble s2DRobot::calc_JointRotationDerivative(Joint *joint, const s2D &origin, const s2D &position)
    {
	sDouble rotation_save = joint->rotation;
	
	sDouble pos_diff1 = calc_PositionDifference(origin, position);
	joint->rotation += s_DELTION;
	sDouble pos_diff2 = calc_PositionDifference(origin, position);
	joint->rotation = rotation_save;

	//printf("pos diffs: %f, %f\n", pos_diff1, pos_diff2);

	return ((pos_diff2 - pos_diff1) / s_DELTION);
    }

    
    sDouble s2DRobot::calc_JointRotation2Derivative(Joint *joint, const s2D &origin, const s2D &position)
    {
	sDouble rotation_save = joint->rotation;
	
	sDouble pos_diff_derivative1 = calc_JointRotationDerivative(joint, origin, position);
	joint->rotation += s_DELTION;
	sDouble pos_diff_derivative2 = calc_JointRotationDerivative(joint, origin, position);
	joint->rotation = rotation_save;

	//printf("pos diffs 2: %f, %f\n", pos_diff_derivative1, pos_diff_derivative2);

	return ((pos_diff_derivative2 - pos_diff_derivative1) / s_DELTION);
    }


    bool s2DRobot::optimize_JointRotation(Joint *joint, const s2D &origin, const s2D &position)
    {
	sDouble drot, ddrot;
	
	for (sInt_32 i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
	{
	    drot = calc_JointRotationDerivative(joint, origin, position);
	    ddrot = calc_JointRotation2Derivative(joint, origin, position);

	    printf("%.10f,%.3f,%d <-- %f\n", drot, ddrot, i, joint->rotation);
	    
	    if (sABS(drot) > s_PRECISION)
	    {
		joint->rotation = joint->rotation - (drot / ddrot);		
	    }
	    else
	    {
		break;
	    }
	}
	joint->rotation -= sSGN(joint->rotation) * 2 * M_PI * floor(joint->rotation / (2 * M_PI));	

	if (sABS(drot) <= s_PRECISION)
	{
	    s2D end;
	    calc_EndPosition(origin, end);
	    printf("Optimum found: %.3f\n", joint->rotation);
	    end.to_Screen();
	    return true;
	}
	else
	{
	    printf("Failed to find optimum (joint rot).\n");	    
	    return false;
	}	
    }

    
    bool s2DRobot::optimize_JointRotation(Joint *joint, const s2D &origin, const s2D &position, sDouble &rotation)
    {
	sDouble rotation_save = joint->rotation;
	sDouble drot, ddrot;
	
	for (sInt_32 i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
	{
	    drot = calc_JointRotationDerivative(joint, origin, position);
	    ddrot = calc_JointRotation2Derivative(joint, origin, position);

	    printf("%.3f,%.3f,%d <-- %f\n", drot, ddrot, i, joint->rotation);
	    
	    if (sABS(drot) > s_PRECISION)
	    {
		joint->rotation = joint->rotation - (drot / ddrot);		
	    }
	    else
	    {
		break;
	    }
	}
	joint->rotation -= sSGN(joint->rotation) * 2 * M_PI * floor(joint->rotation / (2 * M_PI));
	rotation = joint->rotation;
	
	joint->rotation = rotation_save;

	if (sABS(drot) <= s_PRECISION)
	{
	    printf("Optimum found: %.3f\n", rotation);
	    return true;
	}
	else
	{
	    printf("Failed to find optimum.\n");
	    return false;
	}
    }


    bool s2DRobot::optimize_JointConstraint(Joint *joint, const s2D &origin, const s2D &position)
    {
	sDouble rotation;
	
	if (optimize_JointConstraint(joint, origin, position, rotation))
	{
	    joint->rotation = rotation;
	    return true;
	}
	else
	{
	    return false;
	}
    }

    
    bool s2DRobot::optimize_JointConstraint(Joint *joint, const s2D &sUNUSED(origin), const s2D &sUNUSED(position), sDouble &rotation)
    {
	switch (joint->constraint->type)
	{
	case Constraint::TYPE_ANGULAR:
	{
	    Joint *_joint = base_joint;
	    sDouble _rotation = 0.0;
	
	    while (_joint != NULL)
	    {
		if (_joint == joint)
		{
		    rotation = joint->constraint->angle - _rotation;
		    rotation -= sSGN(rotation) * 2 * M_PI * floor(rotation / (2 * M_PI));
		    printf("constraint rotation:%.3f\n", rotation);
		    break;
		}
		_rotation += _joint->rotation;	    
		_joint = _joint->next;
	    }
	    return true;
	}
	default:
	{
	    break;
	}
	}
	return false;
    }


    bool s2DRobot::optimize_RobotConfiguration(Joint *base_joint, const s2D &origin, const s2D &position)
    {
	Configuration configuration;
	save_RobotConfiguration(configuration);
	
	for (sInt_32 r = 0; r < MAX_OPTIMIZATION_RESTARTS; ++r)
	{
	    randomize_RobotConfiguration();
	
	    for (sInt_32 i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
	    {
		Joint *joint = base_joint;
		
		while (joint != NULL)
		{
		    if (!optimize_JointRotation(joint, origin, position))
		    {
			restore_RobotConfiguration(configuration);
			return false;
		    }
		    joint = joint->next;
		}
		sDouble diff = calc_PositionDifference(origin, position);
		printf("Diff:%.10f\n", diff);
	    
		if (diff < s_PRECISION)
		{
		    printf("Optimal configuration found.\n");
		    return true;
		}
	    }
	}
	restore_RobotConfiguration(configuration);
	printf("Failed to optimize configuration.\n");	
	return false;
    }


    bool s2DRobot::optimize_ConstrainedRobotConfiguration(Joint *base_joint, const s2D &origin, const s2D &position)
    {
	Configuration configuration;
	save_RobotConfiguration(configuration);
	    
	for (sInt_32 r = 0; r < MAX_OPTIMIZATION_RESTARTS; ++r)
	{
	    randomize_RobotConfiguration();
		
	    for (sInt_32 i = 0; i < MAX_OPTIMIZATION_ITERATIONS; ++i)
	    {
		Joint *joint = base_joint;

		while (joint != NULL)
		{
		    if (joint->constraint != NULL)
		    {
			if (!optimize_JointConstraint(joint, origin, position))
			{
			    restore_RobotConfiguration(configuration);
			    return false;
			}		    
		    }
		    else
		    {
			if (!optimize_JointRotation(joint, origin, position))
			{
			    restore_RobotConfiguration(configuration);			    
			    return false;
			}
		    }
		    joint = joint->next;		
		}
		sDouble diff = calc_PositionDifference(origin, position);
		printf("Diff:%.10f,%.10f\n", diff, s_EPSILON);
	    
		if (diff < s_PRECISION)
		{
		    printf("Optimal configuration found.\n");
		    return true;
		}
	    }
	}
	restore_RobotConfiguration(configuration);	
	printf("Failed to optimize configuration.\n");	
	return false;
    }    


/*----------------------------------------------------------------------------*/
    
    void s2DRobot::to_Screen(const sString &indent) const
    {
	to_Stream(stdout, indent);	
    }

    
    void s2DRobot::to_Stream(FILE *fw, const sString &indent) const
    {
	fprintf(fw, "%s2DRobot: (|joints| = %ld, |links| = %ld)\n", indent.c_str(), Joints.size(), Links.size());
	
	fprintf(fw, "%s%sJoints:\n", indent.c_str(), s_INDENT.c_str());
	for (Joints_vector::const_iterator joint = Joints.begin(); joint != Joints.end(); ++joint)
	{
	    (*joint)->to_Screen(indent + s2_INDENT);
	}

	fprintf(fw, "%s%sLinks:\n", indent.c_str(), s_INDENT.c_str());
	for (Links_vector::const_iterator link = Links.begin(); link != Links.end(); ++link)
	{
	    (*link)->to_Screen(indent + s2_INDENT);
	}	
    }
    
    
/*----------------------------------------------------------------------------*/

} // namespace rrOST
