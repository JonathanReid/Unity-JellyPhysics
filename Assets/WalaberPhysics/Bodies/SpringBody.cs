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
    /// The simplest type of Body, that tries to maintain its shape through shape-matching (global springs that
    /// try to keep the original shape), and internal springs for support.  Shape matching forces can be
    /// enabled / disabled at will.
    /// </summary>
    public class SpringBody : Body
    {
        #region PRIVATE VARIABLES
        protected List<InternalSpring> mSprings;

        // shape-matching spring constants.
        protected bool mShapeMatchingOn = true;
        protected float mEdgeSpringK;
        protected float mEdgeSpringDamp;
        protected float mShapeSpringK;
        protected float mShapeSpringDamp;
                
        //// debug visualization variables
//        VertexDeclaration mVertexDecl = null;
        #endregion

        #region CONSTRUCTORS
        /// <summary>
        /// Create a Springbody with shape matching set to OFF.
        /// </summary>
        /// <param name="w">World to add this body to.</param>
        /// <param name="shape">ClosedShape shape for this body</param>
        /// <param name="massPerPoint">mass per PointMass.</param>
        /// <param name="edgeSpringK">spring constant for edges.</param>
        /// <param name="edgeSpringDamp">spring damping for edges</param>
        /// <param name="pos">global position of the body</param>
        /// <param name="angleinRadians">global angle of the body</param>
        /// <param name="scale">scale</param>
        /// <param name="kinematic">kinematic control boolean</param>
        public void Setup(World w, ClosedShape shape, float massPerPoint, float edgeSpringK, float edgeSpringDamp, Vector2 pos, float angleinRadians, Vector2 scale, bool kinematic)
        {
			base.Setup (w, shape, massPerPoint, pos, angleinRadians, scale, kinematic);

            mShapeMatchingOn = false;
            mSprings = new List<InternalSpring>();

            base.setPositionAngle(pos, angleinRadians, scale);

            mEdgeSpringK = edgeSpringK;
            mEdgeSpringDamp = edgeSpringDamp;
            mShapeSpringK = 0.0f;
            mShapeSpringDamp = 0.0f;

            // build default springs.
            _buildDefaultSprings();
        }

        /// <summary>
        /// Create a SpringBody with shape matching turned ON.
        /// </summary>
        /// <param name="w"></param>
        /// <param name="shape">ClosedShape shape for this body</param>
        /// <param name="massPerPoint">mass per PointMass.</param>
        /// <param name="shapeSpringK">shape-matching spring constant</param>
        /// <param name="shapeSpringDamp">shape-matching spring damping</param>
        /// <param name="edgeSpringK">spring constant for edges.</param>
        /// <param name="edgeSpringDamp">spring damping for edges</param>
        /// <param name="pos">global position</param>
        /// <param name="angleinRadians">global angle</param>
        /// <param name="scale">scale</param>
        /// <param name="kinematic">kinematic control boolean</param>
        public void Setup(World w, ClosedShape shape, float massPerPoint, float shapeSpringK, float shapeSpringDamp, float edgeSpringK, float edgeSpringDamp, Vector2 pos, float angleinRadians, Vector2 scale, bool kinematic)
        {
			base.Setup (w, shape, massPerPoint, pos, angleinRadians, scale, kinematic);
            mSprings = new List<InternalSpring>();

            base.setPositionAngle(pos, angleinRadians, scale);

            
            mShapeMatchingOn = true;
            mShapeSpringK = shapeSpringK;
            mShapeSpringDamp = shapeSpringDamp;
            mEdgeSpringK = edgeSpringK;
            mEdgeSpringDamp = edgeSpringDamp;

            // build default springs.
            _buildDefaultSprings();
        }
        #endregion

        #region SPRINGS
        /// <summary>
        /// Add an internal spring to this body.
        /// </summary>
        /// <param name="pointA">point mass on 1st end of the spring</param>
        /// <param name="pointB">point mass on 2nd end of the spring</param>
        /// <param name="springK">spring constant</param>
        /// <param name="damping">spring damping</param>
        public void addInternalSpring(int pointA, int pointB, float springK, float damping)
        {
            float dist = (mPointMasses[pointB].Position - mPointMasses[pointA].Position).magnitude;
            InternalSpring s = new InternalSpring(pointA, pointB, dist, springK, damping);

            mSprings.Add(s);
        }

        /// <summary>
        /// Clear all springs from the body.
        /// </summary>
        /// <param name="k"></param>
        /// <param name="damp"></param>
        public void clearAllSprings()
        {
            mSprings.Clear();
            _buildDefaultSprings();
        }

        private void _buildDefaultSprings()
        {
            for (int i = 0; i < mPointMasses.Count; i++)
            {
                if (i < (mPointMasses.Count - 1))
                    addInternalSpring(i, i + 1, mEdgeSpringK, mEdgeSpringDamp);
                else
                    addInternalSpring(i, 0, mEdgeSpringK, mEdgeSpringDamp);
            }
        }
        #endregion

        #region SHAPE MATCHING
        /// <summary>
        /// Set shape-matching on/off.
        /// </summary>
        /// <param name="onoff">boolean</param>
        public void setShapeMatching(bool onoff) { mShapeMatchingOn = onoff; }

        /// <summary>
        /// Set shape-matching spring constants.
        /// </summary>
        /// <param name="springK">spring constant</param>
        /// <param name="damping">spring damping</param>
        public void setShapeMatchingConstants(float springK, float damping) { mShapeSpringK = springK; mShapeSpringDamp = damping; }
        #endregion

        #region ADJUSTING EDGE VALUES
        /// <summary>
        /// Change the spring constants for the springs around the shape itself (edge springs)
        /// </summary>
        /// <param name="edgeSpringK">spring constant</param>
        /// <param name="edgeSpringDamp">spring damping</param>
        public void setEdgeSpringConstants(float edgeSpringK, float edgeSpringDamp)
        {
            // we know that the first n springs in the list are the edge springs.
            for (int i = 0; i < mPointMasses.Count; i++)
            {
                mSprings[i].springK = edgeSpringK;
                mSprings[i].damping = edgeSpringDamp;
            }
        }
        #endregion

        #region ADJUSTING SPRING VALUES
        public void setSpringConstants(int springID, float springK, float springDamp)
        {
            // index is for all internal springs, AFTER the default internal springs.
            int index = mPointMasses.Count + springID;
            mSprings[index].springK = springK;
            mSprings[index].damping = springDamp;
        }

        public float getSpringK(int springID)
        {
            int index = mPointMasses.Count + springID;
            return mSprings[index].springK;
        }

        public float getSpringDamping(int springID)
        {
            int index = mPointMasses.Count + springID;
            return mSprings[index].damping;
        }
        #endregion

        #region ACCUMULATING FORCES
        public override void accumulateInternalForces()
        {
            base.accumulateInternalForces();

            // internal spring forces.
            Vector2 force = new Vector2();
            for (int i = 0; i < mSprings.Count; i++)
            {
                InternalSpring s = mSprings[i];
                VectorTools.calculateSpringForce(ref mPointMasses[s.pointMassA].Position, ref mPointMasses[s.pointMassA].Velocity,
                    ref mPointMasses[s.pointMassB].Position, ref mPointMasses[s.pointMassB].Velocity, 
                    s.springD, s.springK, s.damping,
                    ref force);

                mPointMasses[s.pointMassA].Force.x += force.x;
                mPointMasses[s.pointMassA].Force.y += force.y;

                mPointMasses[s.pointMassB].Force.x -= force.x;
                mPointMasses[s.pointMassB].Force.y -= force.y;
            }

            // shape matching forces.
            if (mShapeMatchingOn)
            {
                mBaseShape.transformVertices(DerivedPos, DerivedAngle, ref mScale, ref mGlobalShape);
                for (int i = 0; i < mPointMasses.Count; i++)
                {
                    if (mShapeSpringK > 0)
                    {
                        if (!mKinematic)
                        {
                            VectorTools.calculateSpringForce(ref mPointMasses[i].Position, ref mPointMasses[i].Velocity,
                                ref mGlobalShape[i], ref mPointMasses[i].Velocity, 0.0f, mShapeSpringK, mShapeSpringDamp,
                                ref force);
                        }
                        else
                        {
                            Vector2 kinVel = Vector2.zero;
                            VectorTools.calculateSpringForce(ref mPointMasses[i].Position, ref mPointMasses[i].Velocity,
                                ref mGlobalShape[i], ref kinVel, 0.0f, mShapeSpringK, mShapeSpringDamp,
                                ref force);
                        }

                        mPointMasses[i].Force.x += force.x;
                        mPointMasses[i].Force.y += force.y;
                    }
                }
            }
        }
        #endregion

		void OnDrawGizmos()
		{
			mBaseShape.transformVertices(DerivedPos, DerivedAngle, ref mScale, ref mGlobalShape);
			
			VertexPositionColor[] shape = new VertexPositionColor[mPointMasses.Count * 2];
			VertexPositionColor[] springs = new VertexPositionColor[mSprings.Count * 2];
			
			mBaseShape.transformVertices(DerivedPos, DerivedAngle, ref mScale, ref mGlobalShape);
			for (int i = 0; i < mPointMasses.Count; i++)
			{
				shape[(i * 2) + 0] = new VertexPositionColor();
				shape[(i * 2) + 0].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position);
				shape[(i * 2) + 0].Color = Color.green;

				shape[(i * 2) + 1] = new VertexPositionColor();
				shape[(i * 2) + 1].Position = VectorTools.vec3FromVec2(mGlobalShape[i]);
				shape[(i * 2) + 1].Color = Color.red;
//				if(i != 0)
//				{
//					//					Gizmos.DrawLine(springs[(i * 2) + 0].Position,springs[(i * 2) + 1].Position);
//					Gizmos.DrawLine(springs[i].Position,springs[i+1].Position);
//					Gizmos.DrawLine(springs[i+1].Position,springs[i-1].Position);
//				}
//				Gizmos.DrawLine(springs[mSprings.Count-1].Position,springs[0].Position);
			}
			
			for (int i = 0; i < mSprings.Count; i++)
			{
				springs[(i * 2) + 0] = new VertexPositionColor();
				springs[(i * 2) + 0].Position = VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassA].Position);
				springs[(i * 2) + 0].Color = Color.gray;

				springs[(i * 2) + 1] = new VertexPositionColor();
				springs[(i * 2) + 1].Position = VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassB].Position);
				springs[(i * 2) + 1].Color = Color.yellow;
				Gizmos.color = Color.red;
				if(i != 0)
				{
//					Gizmos.DrawLine(springs[(i * 2) + 0].Position,springs[(i * 2) + 1].Position);
					Gizmos.DrawLine(springs[i*2].Position,springs[(i*2)+1].Position);
//					Gizmos.DrawLine(springs[(i*2)+1].Position,springs[i-1].Position);
				}
			}
			Gizmos.DrawLine(springs[mSprings.Count-1].Position,springs[0].Position);
			
		}
		/*
        #region DEBUG VISUALIZATION
        public override void debugDrawMe(GraphicsDevice device, Effect effect)
        {
            if (mVertexDecl == null)
            {
                mVertexDecl = new VertexDeclaration(device, VertexPositionColor.VertexElements);
            }
            

            // now draw the goal positions.
            VertexPositionColor[] shape = new VertexPositionColor[mPointMasses.Count * 2];
            VertexPositionColor[] springs = new VertexPositionColor[mSprings.Count * 2];

            mBaseShape.transformVertices(ref mDerivedPos, mDerivedAngle, ref mScale, ref mGlobalShape);
            for (int i = 0; i < mPointMasses.Count; i++)
            {
                shape[(i * 2) + 0].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position);
                shape[(i * 2) + 0].Color = Color.LawnGreen;

                shape[(i * 2) + 1].Position = VectorTools.vec3FromVec2(mGlobalShape[i]);
                shape[(i * 2) + 1].Color = Color.LightSeaGreen;
            }

            for (int i = 0; i < mSprings.Count; i++)
            {
                springs[(i * 2) + 0].Position = VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassA].Position);
                springs[(i * 2) + 0].Color = Color.LawnGreen;
                springs[(i * 2) + 1].Position = VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassB].Position);
                springs[(i * 2) + 1].Color = Color.LightSeaGreen;
            }

            device.VertexDeclaration = mVertexDecl;
            effect.Begin();
            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Begin();
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, shape, 0, mPointMasses.Count);
                device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, springs, 0, mSprings.Count);
                pass.End();
            }
            effect.End();

            base.debugDrawMe(device, effect);
        }
        #endregion
		*/
    }
}
