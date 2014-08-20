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

----------------------------------------------------------------------------
Portions of this Pressure class are based on the excellent tutorial
"How to Implement Pressure Soft Body Model" by Maciej Matyka
http://panoramix.ift.uni.wroc.pl/~maq/eng/index.php

*/

using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace JelloPhysics
{
    /// <summary>
    /// a subclass of SpringBody, with the added element of pressurized gas inside the body.  The amount
    /// of pressure can be adjusted at will to inflate / deflate the object.  The object will not deflate
    /// much smaller than the original size of the Shape if shape matching is enabled.
    /// </summary>
    public class PressureBody : SpringBody
    {
        #region PRIVATE VARIABLES
        private float mVolume;
        private float mGasAmount;
        private Vector2[] mNormalList;
        private float[] mEdgeLengthList;
        #endregion

        #region CONSTRUCTORS
        /// <summary>
        /// Default constructor, with shape-matching ON.
        /// </summary>
        /// <param name="w">World object to add this body to</param>
        /// <param name="s">ClosedShape for this body</param>
        /// <param name="massPerPoint">mass per PointMass</param>
        /// <param name="gasPressure">amount of gas inside the body</param>
        /// <param name="shapeSpringK">shape-matching spring constant</param>
        /// <param name="shapeSpringDamp">shape-matching spring damping</param>
        /// <param name="edgeSpringK">spring constant for edges</param>
        /// <param name="edgeSpringDamp">spring damping for edges</param>
        /// <param name="pos">global position</param>
        /// <param name="angleInRadians">global angle</param>
        /// <param name="scale">scale</param>
        /// <param name="kinematic">kinematic control boolean</param>
        public void Setup(World w, ClosedShape s, float massPerPoint, float gasPressure, float shapeSpringK, float shapeSpringDamp, float edgeSpringK, float edgeSpringDamp, Vector2 pos, float angleInRadians, Vector2 scale, bool kinematic)
        {
			base.Setup (w, s, massPerPoint, shapeSpringK, shapeSpringDamp, edgeSpringK, edgeSpringDamp, pos, angleInRadians, scale, kinematic);
            mGasAmount = gasPressure;
            mNormalList = new Vector2[mPointMasses.Count];
            mEdgeLengthList = new float[mPointMasses.Count];
        }
        #endregion

        #region PRESSURE
        /// <summary>
        /// Amount of gas inside the body.
        /// </summary>
        public float GasPressure
        {
            set { mGasAmount = value; }
            get { return mGasAmount; }
        }
        #endregion

        #region VOLUME
        /// <summary>
        /// Gets the last calculated volume for the body.
        /// </summary>
        public float Volume
        {
            get { return mVolume; }
        }
        #endregion

        #region ACCUMULATING FORCES
        public override void accumulateInternalForces()
        {
            base.accumulateInternalForces();

            // internal forces based on pressure equations.  we need 2 loops to do this.  one to find the overall volume of the
            // body, and 1 to apply forces.  we will need the normals for the edges in both loops, so we will cache them and remember them.
            mVolume = 0f;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                int prev = (i > 0) ? i-1 : mPointMasses.Count-1;
                int next = (i < mPointMasses.Count - 1) ? i + 1 : 0;

                // currently we are talking about the edge from i --> j.
                // first calculate the volume of the body, and cache normals as we go.
                Vector2 edge1N = new Vector2();
                edge1N.x = mPointMasses[i].Position.x - mPointMasses[prev].Position.x;
                edge1N.y = mPointMasses[i].Position.y - mPointMasses[prev].Position.y;
                VectorTools.makePerpendicular(ref edge1N);

                Vector2 edge2N = new Vector2();
                edge2N.x = mPointMasses[next].Position.x - mPointMasses[i].Position.x;
                edge2N.y = mPointMasses[next].Position.y - mPointMasses[i].Position.y;
                VectorTools.makePerpendicular(ref edge2N);

                Vector2 norm = new Vector2();
                norm.x = edge1N.x + edge2N.x;
                norm.y = edge1N.y + edge2N.y;

                float nL = (float)Math.Sqrt((norm.x*norm.x)+(norm.y*norm.y));
                if (nL > 0.001f)
                {
                    norm.x /= nL;
                    norm.y /= nL;
                }

                float edgeL = (float)Math.Sqrt((edge2N.x * edge2N.x) + (edge2N.y * edge2N.y));

                // cache normal and edge length
                mNormalList[i] = norm;
                mEdgeLengthList[i] = edgeL;

                float xdist = Math.Abs(mPointMasses[i].Position.x - mPointMasses[next].Position.x);

                float volumeProduct = xdist * Math.Abs(norm.x) * edgeL;

                // add to volume
                mVolume += 0.5f * volumeProduct;
            }

            // now loop through, adding forces!
            float invVolume = 1f / mVolume;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                int j = (i < mPointMasses.Count - 1) ? i + 1 : 0;

                float pressureV = (invVolume * mEdgeLengthList[i] * mGasAmount);
                mPointMasses[i].Force.x += mNormalList[i].x * pressureV;
                mPointMasses[i].Force.y += mNormalList[i].y * pressureV;

                mPointMasses[j].Force.x += mNormalList[j].x * pressureV;
                mPointMasses[j].Force.y += mNormalList[j].y * pressureV;
            }
        }
        #endregion
		/*
        #region DEBUG VISUALIZATION
        public override void debugDrawMe(GraphicsDevice device, Effect effect)
        {
            base.debugDrawMe(device, effect);

            // draw edge normals!
            VertexPositionColor[] normals = new VertexPositionColor[mPointMasses.Count*2];

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                int prev = (i > 0) ? i - 1 : mPointMasses.Count - 1;
                int next = (i < mPointMasses.Count - 1) ? i + 1 : 0;

                // currently we are talking about the edge from i --> j.
                // first calculate the volume of the body, and cache normals as we go.
                Vector2 edge1N = VectorTools.getPerpendicular(mPointMasses[i].Position - mPointMasses[prev].Position);
                edge1N.Normalize();

                Vector2 edge2N = VectorTools.getPerpendicular(mPointMasses[next].Position - mPointMasses[i].Position);
                edge2N.Normalize();

                Vector2 norm = edge1N + edge2N;
                float nL = norm.Length();
                if (nL > 0.001f)
                    norm.Normalize();

                normals[(i * 2) + 0].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position);
                normals[(i * 2) + 0].Color = Color.yellow;

                normals[(i * 2) + 1].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position + norm);
                normals[(i * 2) + 1].Color = Color.Honeydew;
            }

            effect.Begin();
            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Begin();
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, normals, 0, mPointMasses.Count);
                pass.End();
            }
            effect.End();
        }
        #endregion
		*/
    }
}
