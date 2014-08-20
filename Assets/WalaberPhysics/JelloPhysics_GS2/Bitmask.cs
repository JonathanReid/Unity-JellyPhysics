using System;
using System.Collections.Generic;
using System.Text;

namespace JelloPhysics
{
    public struct Bitmask
    {
        public int mask;

        public void clear() { mask = 0x00; }
        public void setOn(int bit)
        {
            mask |= (0x01 << ((bit > 0) ? (bit - 1) : 0));
        }
        public void setOff(int bit)
        {
            mask &= ~(0x01 << ((bit > 0) ? (bit - 1) : 0));
        }
    }
}
