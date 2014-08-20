/*
Copyright (c) 2007 Walaber

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace JelloPhysics
{
    /// <summary>
    /// class represents a 2D axis-aligned bounding box, for collision detection, etc.
    /// </summary>
    public class AABB
    {
        #region PRIVATE VARIABLES
        public enum PointValidity { Invalid, Valid };

        /// <summary>
        /// Minimum point of this bounding box.
        /// </summary>
        public Vector2 Min;   

        /// <summary>
        /// Maximum point of this bounding box.
        /// </summary>
        public Vector2 Max;

        /// <summary>
        /// Property that indicated whether or not this bounding box is valid.
        /// </summary>
        public PointValidity Validity;
        #endregion


        #region CONSTRUCTORS
        /// <summary>
        /// Basic constructor.  creates a bounding box that is invalid (describes no space)
        /// </summary>
        public AABB()
        {
            Min = Max = Vector2.zero;
            Validity = PointValidity.Invalid;
        }

        /// <summary>
        /// create a boundingbox with the given min and max points.
        /// </summary>
        /// <param name="minPt">min point</param>
        /// <param name="maxPt">max point</param>
        public AABB(ref Vector2 minPt, ref Vector2 maxPt)
        {
            Min = minPt;
            Max = maxPt;
            Validity = PointValidity.Valid;
        }
        #endregion

        #region CLEAR
        /// <summary>
        /// Resets a bounding box to invalid.
        /// </summary>
        public void clear()
        {
            Min.x = Max.x = Min.y = Max.y = 0;
            Validity = PointValidity.Invalid;
        }
        #endregion

        #region EXPANSION
        public void expandToInclude(ref Vector2 pt)
        {
            if (Validity == PointValidity.Valid)
            {
                if (pt.x < Min.x) { Min.x = pt.x; }
                else if (pt.x > Max.x) { Max.x = pt.x; }

                if (pt.y < Min.y) { Min.y = pt.y; }
                else if (pt.y > Max.y) { Max.y = pt.y; }
            }
            else
            {
                Min = Max = pt;
                Validity = PointValidity.Valid;
            }
        }
        #endregion

        #region COLLISION / OVERLAP
        public bool contains( ref Vector2 pt )
        {
            if (Validity == PointValidity.Invalid) { return false; }

            return ((pt.x >= Min.x) && (pt.x <= Max.x) && (pt.y >= Min.y) && (pt.y <= Max.y));
        }

        public bool intersects(ref AABB box)
        {
            // X overlap check.
            bool overlapX = ((Min.x <= box.Max.x) && (Max.x >= box.Min.x));
            bool overlapY = ((Min.y <= box.Max.y) && (Max.y >= box.Min.y));

            return (overlapX && overlapY);
        }

        #endregion
    }
}
