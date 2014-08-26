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
using UnityEngine;

namespace JelloPhysics
{
    /// <summary>
    /// class that represents a single polygonal closed shape (can be concave)
    /// </summary>
    public class ClosedShape : MonoBehaviour
    {
        #region PRIVATE VARIABLES
        ////////////////////////////////////////////////////////////////
        // Vertices that make up this collision geometry.  shape connects vertices in order, closing the last vertex to the first.
        private List<Vector2> mLocalVertices;
        #endregion

        #region CONSTRUCTORS
        ////////////////////////////////////////////////////////////////
        // default constructor.
        public ClosedShape()
        {
            mLocalVertices = new List<Vector2>();
        }

        // construct from an existing list of vertices.
        public ClosedShape(List<Vector2> verts)
        {
            mLocalVertices = new List<Vector2>(verts);
            finish();
        }
        #endregion

        #region SETUP - ADDING VERTS
        ////////////////////////////////////////////////////////////////
        // start adding vertices to this collision.  will erase any existing verts.
        public void begin()
        {
            mLocalVertices.Clear();
        }

        ////////////////////////////////////////////////////////////////
        // add a vertex to this collision.
        public int addVertex(Vector2 vert)
        {
            mLocalVertices.Add(vert);
            return mLocalVertices.Count;
        }

        ////////////////////////////////////////////////////////////////
        // finish adding vertices to this collision, and convert them into local space (be default).
        public void finish(bool recenter)
        {
            if (recenter)
            {
                // find the average location of all of the vertices, this is our geometrical center.
                Vector2 center = Vector2.zero;

                for (int i = 0; i < mLocalVertices.Count; i++)
                    center += mLocalVertices[i];

                center /= mLocalVertices.Count;

                // now subtract this from each element, to get proper "local" coordinates.
                for (int i = 0; i < mLocalVertices.Count; i++)
                    mLocalVertices[i] -= center;
            }
        }

        public void finish()
        {
            finish(true);
        }
        #endregion

        #region PUBLIC PROPERTIES
        ////////////////////////////////////////////////////////////////
        // access to the vertice list.
        public List<Vector2> Vertices
        {
            get { return mLocalVertices; }
        }
        #endregion

        #region HELPER FUNCTIONS
        /// <summary>
        /// Get a new list of vertices, transformed by the given position, angle, and scale.
        /// transformation is applied in the following order:  scale -> rotation -> position.
        /// </summary>
        /// <param name="worldPos">position</param>
        /// <param name="angleInRadians">rotation (in radians)</param>
        /// <param name="localScale">scale</param>
        /// <returns>new list of transformed points.</returns>
        public List<Vector2> transformVertices(Vector2 worldPos, float angleInRadians, Vector2 localScale)
        {
            List<Vector2> ret = new List<Vector2>(mLocalVertices);

            Vector2 v = new Vector2();
            for (int i = 0; i < ret.Count; i++)
            {
                // rotate the point, and then translate.
                v.x = ret[i].x * localScale.x;
                v.y = ret[i].y * localScale.y;
                JelloPhysics.VectorTools.rotateVector(ref v, angleInRadians, ref v);

                v.x += worldPos.x;
                v.y += worldPos.y;
                ret[i] = v;
            }

            return ret;
        }

        /// <summary>
        /// Get a new list of vertices, transformed by the given position, angle, and scale.
        /// transformation is applied in the following order:  scale -> rotation -> position.
        /// </summary>
        /// <param name="worldPos">position</param>
        /// <param name="angleInRadians">rotation (in radians)</param>
        /// <param name="localScale">scale</param>
        /// <param name="outList">new list of transformed points.</param>
        public void transformVertices(Vector2 worldPos, float angleInRadians, ref Vector2 localScale, ref Vector2[] outList)
        {
            for (int i = 0; i < mLocalVertices.Count; i++)
            {
                // rotate the point, and then translate.
                Vector2 v = new Vector2();
                v.x = mLocalVertices[i].x * localScale.x;
                v.y = mLocalVertices[i].y * localScale.y;
                JelloPhysics.VectorTools.rotateVector(ref v, angleInRadians);
                v.x += worldPos.x;
                v.y += worldPos.y;
                //outList[i] = JelloPhysics.VectorTools.rotateVector(mLocalVertices[i] * localScale, angleInRadians);
                outList[i].x = v.x;
                outList[i].y = v.y;
            }
        }
        #endregion

//		void OnDrawGizmos()
//		{
//			for (int i = 1; i < mLocalVertices.Count; i++) 
//			{
//				Gizmos.DrawLine(mLocalVertices[i-1],mLocalVertices[i]);
//			}
//		}
    }
}
