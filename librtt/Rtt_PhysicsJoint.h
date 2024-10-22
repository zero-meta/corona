//////////////////////////////////////////////////////////////////////////////
//
// This file is part of the Corona game engine.
// For overview and more information on licensing please refer to README.md 
// Home page: https://github.com/coronalabs/corona
// Contact: support@coronalabs.com
//
//////////////////////////////////////////////////////////////////////////////

#ifndef _Rtt_PhysicsJoint_H__
#define _Rtt_PhysicsJoint_H__

#ifdef Rtt_PHYSICS

#include "box2d/box2d.h"
#include "Rtt_LuaProxyVTable.h"

// ----------------------------------------------------------------------------

// class b2Joint;
// struct b2Vec2;

struct lua_State;

namespace Rtt
{

//class Runtime;
	
// ----------------------------------------------------------------------------

class PhysicsJoint
{
	public:
		typedef PhysicsJoint Self;

	public:
		static const char kMetatableName[];

	public:
		static b2JointId GetJoint( lua_State *L, int index );

	public:
		static void Initialize( lua_State *L );
	
	public:
		static int getAnchorA( lua_State *L );
		static int getAnchorB( lua_State *L );
		static bool HasLocalAnchor( b2JointId joint );
		static b2Vec2 GetLocalAnchorA( b2JointId joint );
		static b2Vec2 GetLocalAnchorB( b2JointId joint );
		static int getLocalAnchorA( lua_State *L );
		static int getLocalAnchorB( lua_State *L );
		static int getLocalAxis( lua_State *L );
		static int getReactionForce( lua_State *L );
		static int setRotationLimits( lua_State *L );
		static int getRotationLimits( lua_State *L );
		static int setLimits( lua_State *L );
		static int getLimits( lua_State *L );
		static int setLinearOffset( lua_State *L );
		static int getLinearOffset( lua_State *L );
		static int getGroundAnchorA( lua_State *L );
		static int getGroundAnchorB( lua_State *L );
		static int removeSelf( lua_State *L );
		static int getTarget( lua_State *L );
		static int setTarget( lua_State *L );
				
	// Metatable methods
	public:
		static int ValueForKey( lua_State *L );
		static int SetValueForKey( lua_State *L );
		static int Finalizer( lua_State *L );
};

// ----------------------------------------------------------------------------
	
} // namespace Rtt

// ----------------------------------------------------------------------------

#endif // Rtt_PHYSICS	

#endif // _Rtt_PhysicsJoint_H__
