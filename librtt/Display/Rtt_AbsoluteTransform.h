//////////////////////////////////////////////////////////////////////////////
// Internal, call-scoped absolute transform assignment. No display object ABI changes.
//////////////////////////////////////////////////////////////////////////////
#ifndef _Rtt_AbsoluteTransform_H__
#define _Rtt_AbsoluteTransform_H__

#include "Core/Rtt_Geometry.h"
#include "Core/Rtt_Real.h"

namespace Rtt
{
class DisplayObject;

class AbsoluteTransformScope
{
public:
    AbsoluteTransformScope( const DisplayObject *object, GeometricProperty property, Real value )
    : fPrevious( sCurrent ), fObject( object ), fProperty( property ), fValue( value ), fConsumed( false )
    { sCurrent = this; }

    ~AbsoluteTransformScope() { sCurrent = fPrevious; }

    // Only the top request is eligible, and only once. Never expose an outer
    // assignment to an unrelated or reentrant relative operation.
    static bool Take( const DisplayObject *object, bool rotation, GeometricProperty& property, Real& value )
    {
        AbsoluteTransformScope *scope = sCurrent;
        if ( ! scope || scope->fConsumed || scope->fObject != object ||
             ( rotation ? scope->fProperty != kRotation :
               ( scope->fProperty != kOriginX && scope->fProperty != kOriginY ) ) )
        { return false; }
        scope->fConsumed = true;
        property = scope->fProperty;
        value = scope->fValue;
        return true;
    }

    // Callbacks may perform independent relative operations or nested setters.
    // Hide the pending request for their duration; nested setters install their
    // own scope.
    class Suspend
    {
    public:
        Suspend() : fPrevious( sCurrent ) { sCurrent = NULL; }
        ~Suspend() { sCurrent = fPrevious; }
    private:
        Suspend( const Suspend& );
        Suspend& operator=( const Suspend& );
        AbsoluteTransformScope *fPrevious;
    };

private:
    AbsoluteTransformScope( const AbsoluteTransformScope& );
    AbsoluteTransformScope& operator=( const AbsoluteTransformScope& );
    static thread_local AbsoluteTransformScope *sCurrent;
    AbsoluteTransformScope *fPrevious;
    const DisplayObject *fObject;
    GeometricProperty fProperty;
    Real fValue;
    bool fConsumed;
};
}
#endif
