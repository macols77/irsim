#ifndef PROGRAMMED_ARENA
#define PROGRAMMED_ARENA

/******************************************************************************/
/******************************************************************************/

#include "arena.h"

#include <math.h>
#include <vector>
#include <list>

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CProgrammedArena : public CArena
{
public:
    CProgrammedArena(const char* pch_name, unsigned int un_res_x, 
                     unsigned int un_res_y, float f_size_x, float f_size_y);
    virtual ~CProgrammedArena();

    virtual eArenaHeight GetHeight(double x, double y);
    virtual void         SetHeightPixel(unsigned int x, 
                                        unsigned int y, 
                                        eArenaHeight height);

    virtual eArenaHeight GetHeightPixel(unsigned int x, unsigned int y);
    //Build the arena using the given char map    
    virtual void SetHeightPixelsFromChars(const char* pch_arena_as_char, 
                                          char c_hole, 
                                          char c_normal, 
                                          char c_obstacle);

    virtual CCompoundCollisionObject* GetHorizontalCollisionObject(double y, double x1, double x2);
    virtual CCompoundCollisionObject* GetVerticalCollisionObject(double x, double y1, double y2);

    virtual int GetArenaType();

protected:
    eArenaHeight*     m_pArenaPixels;    
};


/******************************************************************************/
/******************************************************************************/

#endif
