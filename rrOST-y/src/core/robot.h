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
/* robot.h / 0-030_noair                                                      */
/*----------------------------------------------------------------------------*/
//
// Robot (model) related data structures and functions.
//
/*----------------------------------------------------------------------------*/


#ifndef __ROBOT_H__
#define __ROBOT_H__


#include <vector>

#include "common/types.h"


using namespace std;
using namespace rrOST;


/*----------------------------------------------------------------------------*/

namespace rrOST
{



    
/*----------------------------------------------------------------------------*/
// s2D
    
    class s2D
    {
    public:
	s2D() { /* nothing */ }
	s2D(sDouble _x, sDouble _y)
	    : x(_x), y(_y)
	{
	    // nothing
	}

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	sDouble x, y;
	
    };


/*----------------------------------------------------------------------------*/
// s3D
    
    class s3D
    {
    public:
	s3D() { /* nothing */ }
	s3D(sDouble _x, sDouble _y, sDouble _z)
	    : x(_x), y(_y), z(_z)
	{
	    // nothing
	}

    public:
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;	

    public:
	sDouble x, y, z;
    };
    


/*----------------------------------------------------------------------------*/
// sRobot
    
    class sRobot
    {
    public:	    
    public:
    };


/*----------------------------------------------------------------------------*/
// s2DRobot
    
    class s2DRobot
	: public sRobot
    {
    public:	
	const sInt_32 MAX_OPTIMIZATION_ITERATIONS = s__MAX_OPTIMIZATION_ITERATIONS;
	const sInt_32 MAX_OPTIMIZATION_RESTARTS   = s__MAX_OPTIMIZATION_RESTARTS;
	
        /*----------------------------------------------------------------*/
	
	struct Link
	{
	    Link(sDouble length)
	        : end(length, 0)
	    {
		// nothing
	    }
	    
	    Link(const s2D _end)
		: end(_end)
	    {
		// nothing
	    }
	    
	    s2D end;

	public:
	    virtual void to_Screen(const sString &indent = "") const;
	    virtual void to_Stream(FILE *fw, const sString &indent = "") const;		    
	};

	struct Constraint
	{
	    enum Type
	    {
		TYPE_UNDEFINED,
		TYPE_ANGULAR
	    };

	    Constraint(sDouble _angle)
	        : type(TYPE_ANGULAR)
	        , angle(_angle)
	    {
		// nothing
	    }

	    Type type;
	    sDouble angle;
	};

	struct Joint
	{
	    Joint(sDouble _rotation)
	        : child(NULL)
		, rotation(_rotation)
		, constraint(NULL)
	    {
		// nothing
	    }
	    
	    Joint(Link *_child, sDouble _rotation)
	        : child(_child)
		, rotation(_rotation)
		, constraint(NULL)		
	    {
		// nothing
	    }

	    Link *child;

	    Joint *prev;
	    Joint *next;

	    sDouble rotation;
	    Constraint *constraint;
	    
	public:	    
	    virtual void to_Screen(const sString &indent = "") const;
	    virtual void to_Stream(FILE *fw, const sString &indent = "") const;		    
	};

	typedef vector<Link*> Links_vector;	
	typedef vector<Joint*> Joints_vector;

	typedef vector<sDouble> Rotations_vector;

	
	struct Configuration
	{
	    Rotations_vector Rotations;

	public:	    
	    virtual void to_Screen(const sString &indent = "") const;
	    virtual void to_Stream(FILE *fw, const sString &indent = "") const;		    	    
	};
	
    public:
	s2DRobot()
	    : base_joint(NULL)
	{
	    // nothing
	}

	virtual ~s2DRobot();
        /*----------------------------------------------------------------*/

	void add_Joint(sDouble rotation);
	void add_Link(s2D end);
	void add_Link(sDouble length);

	void attach_Constraint(Joint *joint, const Constraint &constraint);
        /*----------------------------------------------------------------*/

	void save_RobotConfiguration(Configuration &configuration) const;
	void restore_RobotConfiguration(const Configuration &configuration);
	void randomize_RobotConfiguration(void);
        /*----------------------------------------------------------------*/	

	void calc_EndPosition(const s2D &origin, s2D &end) const;
	void calc_EndPosition(const Joint *_base_joint, const s2D &origin, s2D &end) const;

	sDouble calc_PositionDifference(const s2D &origin, const s2D &position) const;
	sDouble calc_JointRotationDerivative(Joint *joint, const s2D &origin, const s2D &position);
	sDouble calc_JointRotation2Derivative(Joint *joint, const s2D &origin, const s2D &position);

	bool optimize_JointRotation(Joint *joint, const s2D &origin, const s2D &position);
	bool optimize_JointRotation(Joint *joint, const s2D &origin, const s2D &position, sDouble &rotation);

	bool optimize_JointConstraint(Joint *joint, const s2D &origin, const s2D &position);
	bool optimize_JointConstraint(Joint *joint, const s2D &origin, const s2D &position, sDouble &rotation);	
	
	bool optimize_RobotConfiguration(Joint *base_joint, const s2D &origin, const s2D &position);
	bool optimize_ConstrainedRobotConfiguration(Joint *base_joint, const s2D &origin, const s2D &position);	
        /*----------------------------------------------------------------*/
	
	virtual void to_Screen(const sString &indent = "") const;
	virtual void to_Stream(FILE *fw, const sString &indent = "") const;

    public:
	Joint *base_joint;
	
	Links_vector Links;
	Joints_vector Joints;
    };


/*----------------------------------------------------------------------------*/
// s3DRobot
    
    
  

/*----------------------------------------------------------------------------*/

} // namespace rrOST

#endif /* __ROBOT_H__ */
