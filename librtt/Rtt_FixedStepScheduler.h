// Internal fixed-step scheduling. Definitions live in Rtt_PhysicsWorld.cpp.
#ifndef _Rtt_FixedStepScheduler_H__
#define _Rtt_FixedStepScheduler_H__
struct lua_State;
namespace Rtt
{
class PhysicsWorld;
class FixedStepScheduler
{
public:
    static int Configure( lua_State *L );
    static int SetSpeed( lua_State *L );
    static int SetListener( lua_State *L );
    static int GetState( lua_State *L );
    static bool Step( PhysicsWorld& physics, float frameInterval );
    static void Reset( PhysicsWorld& physics );
    static void Interrupt( PhysicsWorld& physics );
    static void Forget( PhysicsWorld& physics );
private:
    static void SyncDisplay( PhysicsWorld& physics );
    static bool DispatchEvents( PhysicsWorld& physics, unsigned long long generation );
};
}
#endif
