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
using System.Linq;
using UnityEngine;

namespace JelloPhysics
{

    /// <summary>
    /// contains base functionality for all bodies in the JelloPhysics world.  all bodies are
    /// made up of a ClosedShape geometry, and a list of PointMass objects equal to the number of vertices in the
    /// ClosedShape geometry.  The vertices are considered to be connected by lines in order, which creates the collision
    /// volume for this body.  Individual implementations of Body handle forcing the body to keep it's shape through
    /// various methods.
    /// </summary>
    public class Body : MonoBehaviour
    {
        #region EVENTS
        public delegate void CollisionEvent(Body otherBody);
        public event CollisionEvent OnCollisionEnter;
        public event CollisionEvent OnCollisionStay;
        public event CollisionEvent OnCollisionExit;
        #endregion

        #region PRIVATE VARIABLES
        internal ClosedShape mBaseShape;
        internal Vector2[] mGlobalShape;
        internal List<PointMass> mPointMasses;
        internal Vector2 mScale;
        internal Vector2 mDerivedPos;
        internal Vector2 mDerivedVel;
        internal float mDerivedAngle;
        internal float Gravity;
        protected float mDerivedOmega;
        protected float mLastAngle;
        protected AABB mAABB;
        protected int mMaterial;
        protected bool mIsStatic;
        protected bool mKinematic;
        protected object mObjectTag;
        protected float mVelDamping = 0.999f;

        //// debug visualization variables
//        VertexDeclaration mVertexDecl = null;
        #endregion

        #region INTERNAL VARIABLES
        internal Bitmask mBitMaskX = new Bitmask();
        internal Bitmask mBitMaskY = new Bitmask();
        #endregion

        #region CONSTRUCTORS

        /// <summary>
        /// default constructor.
        /// </summary>
        /// <param name="w">world to add this body to (done automatically)</param>
        public void Setup(World w)
        {
            mAABB = new AABB();
            mBaseShape = null;
            mGlobalShape = null;
            mPointMasses = new List<PointMass>();
            mScale = Vector2.one;
            mIsStatic = false;
            mKinematic = false;

            mMaterial = 0;

            w.addBody(this);
        }

        /// <summary>
        /// create a body, and set its shape and position immediately
        /// </summary>
        /// <param name="w">world to add this body to (done automatically)</param>
        /// <param name="shape">closed shape for this body</param>
        /// <param name="massPerPoint">mass for each PointMass to be created</param>
        /// <param name="position">global position of the body</param>
        /// <param name="angleInRadians">global angle of the body</param>
        /// <param name="scale">local scale of the body</param>
        /// <param name="kinematic">whether this body is kinematically controlled</param>
        public void Setup(World w, ClosedShape shape, float massPerPoint, Vector2 position, float angleInRadians, Vector2 scale, bool kinematic)
        {
            mAABB = new AABB();
            mDerivedPos = position;
            mDerivedAngle = angleInRadians;
            mLastAngle = mDerivedAngle;
            mScale = scale;
            mMaterial = 0;
            mIsStatic = float.IsPositiveInfinity(massPerPoint);
            mKinematic = kinematic;

            mPointMasses = new List<PointMass>();
            setShape(shape);
            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses [i].Mass = massPerPoint;

            updateAABB(0f, true);

            w.addBody(this);
            List<Vector2> points = new List<Vector2>();
            foreach (PointMass m in mPointMasses)
            {
                points.Add(m.Position);
            }
        }

        /// <summary>
        /// create a body, and set its shape and position immediately - with individual masses for each PointMass.
        /// </summary>
        /// <param name="w">world to add this body to (done automatically)</param>
        /// <param name="shape">closed shape for this body</param>
        /// <param name="pointMasses">list of masses for each PointMass</param>
        /// <param name="position">global position of the body</param>
        /// <param name="angleInRadians">global angle of the body</param>
        /// <param name="scale">local scale of the body</param>
        /// <param name="kinematic">whether this body is kinematically controlled.</param>
        public void Setup(World w, ClosedShape shape, List<float> pointMasses, Vector2 position, float angleInRadians, Vector2 scale, bool kinematic)
        {
            mAABB = new AABB();
            mDerivedPos = position;
            mDerivedAngle = angleInRadians;
            mLastAngle = mDerivedAngle;
            mScale = scale;
            mMaterial = 0;
            mIsStatic = false;
            mKinematic = kinematic;

            mPointMasses = new List<PointMass>();
            setShape(shape);
            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses [i].Mass = pointMasses [i];

            updateAABB(0f, true);

            w.addBody(this);
        }
        #endregion

        #region SETTING SHAPE
        /// <summary>
        /// set the shape of this body to a new ClosedShape object.  This function 
        /// will remove any existing PointMass objects, and replace them with new ones IF
        /// the new shape has a different vertex count than the previous one.  In this case
        /// the mass for each newly added point mass will be set zero.  Otherwise the shape is just
        /// updated, not affecting the existing PointMasses.
        /// </summary>
        /// <param name="shape">new closed shape</param>
        public void setShape(ClosedShape shape)
        {
            mBaseShape = shape;

            if (mBaseShape.Vertices.Count != mPointMasses.Count)
            {
                mPointMasses.Clear();
                mGlobalShape = new Vector2[mBaseShape.Vertices.Count];
                
                mBaseShape.transformVertices(ref mDerivedPos, mDerivedAngle, ref mScale, ref mGlobalShape);

                for (int i = 0; i < mBaseShape.Vertices.Count; i++)
                    mPointMasses.Add(new PointMass(0.0f, mGlobalShape [i]));               
            }
        }
        #endregion

        #region SETTING MASS
        /// <summary>
        /// set the mass for each PointMass in this body.
        /// </summary>
        /// <param name="mass">new mass</param>
        public void setMassAll(float mass)
        {
            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses [i].Mass = mass;

            if (float.IsPositiveInfinity(mass))
            {
                mIsStatic = true;
            }
        }

        /// <summary>
        /// set the mass for each PointMass individually.
        /// </summary>
        /// <param name="index">index of the PointMass</param>
        /// <param name="mass">new mass</param>
        public void setMassIndividual(int index, float mass)
        {
            if ((index >= 0) && (index < mPointMasses.Count))
                mPointMasses [index].Mass = mass;
        }

        /// <summary>
        /// set the mass for all point masses from a list.
        /// </summary>
        /// <param name="masses">list of masses (count MUSE equal PointMasses.Count)</param>
        public void setMassFromList(List<float> masses)
        {
            if (masses.Count == mPointMasses.Count)
            {
                for (int i = 0; i < mPointMasses.Count; i++)
                    mPointMasses [i].Mass = masses [i];
            }
        }
        #endregion

        #region MATERIAL
        /// <summary>
        /// Material for this body.  Used for physical interaction and collision notification.
        /// </summary>
        public int Material
        {
            get { return mMaterial; }
            set { mMaterial = value; }
        }
        #endregion

        #region SETTING POSITION AND ANGLE MANUALLY
        /// <summary>
        /// Set the position and angle of the body manually.
        /// </summary>
        /// <param name="pos">global position</param>
        /// <param name="angleInRadians">global angle</param>
        public virtual void setPositionAngle(Vector2 pos, float angleInRadians, Vector2 scale)
        {
            mBaseShape.transformVertices(ref pos, angleInRadians, ref scale, ref mGlobalShape);
            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses [i].Position = mGlobalShape [i];

            mDerivedPos = pos;
            mDerivedAngle = angleInRadians;
        }

        /// <summary>
        /// For moving a body kinematically.  sets the position in global space.  via shape-matching, the
        /// body will eventually move to this location.
        /// </summary>
        /// <param name="pos">position in global space.</param>
        public virtual void setKinematicPosition(ref Vector2 pos)
        {
            mDerivedPos = pos;
        }

        /// <summary>
        /// For moving a body kinematically.  sets the angle in global space.  via shape-matching, the
        /// body will eventually rotate to this angle.
        /// </summary>
        /// <param name="angleInRadians"></param>
        public virtual void setKinematicAngle(float angleInRadians)
        {
            mDerivedAngle = angleInRadians;
        }

        /// <summary>
        /// For changing a body kinematically.  via shape matching, the body will eventually
        /// change to the given scale.
        /// </summary>
        /// <param name="scale"></param>
        public virtual void setKinematicScale(ref Vector2 scale)
        {
            mScale = scale;
        }
        #endregion

        #region DERIVING POSITION AND VELOCITY
        /// <summary>
        /// Derive the global position and angle of this body, based on the average of all the points.
        /// This updates the DerivedPosision, DerivedAngle, and DerivedVelocity properties.
        /// This is called by the World object each Update(), so usually a user does not need to call this.  Instead
        /// you can juse access the DerivedPosition, DerivedAngle, DerivedVelocity, and DerivedOmega properties.
        /// </summary>
        public void derivePositionAndAngle(float elaspsed)
        {
            // no need it this is a static body, or kinematically controlled.
            if (mIsStatic || mKinematic)
                return;

            // find the geometric center.
            Vector2 center = new Vector2();
            center.x = 0;
            center.y = 0;

            Vector2 vel = new Vector2();
            vel.x = 0f;
            vel.y = 0f;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                center.x += mPointMasses [i].Position.x;
                center.y += mPointMasses [i].Position.y;

                vel.x += mPointMasses [i].Velocity.x;
                vel.y += mPointMasses [i].Velocity.y;
            }

            center.x /= mPointMasses.Count;
            center.y /= mPointMasses.Count;

            vel.x /= mPointMasses.Count;
            vel.y /= mPointMasses.Count;

            mDerivedPos = center;
            mDerivedVel = vel;

            // find the average angle of all of the masses.
            float angle = 0;
            int originalSign = 1;
            float originalAngle = 0;
            for (int i = 0; i < mPointMasses.Count; i++)
            {
                Vector2 baseNorm = new Vector2();
                baseNorm.x = mBaseShape.Vertices [i].x;
                baseNorm.y = mBaseShape.Vertices [i].y;
                baseNorm.Normalize();
//                Vector2.Normalize(ref baseNorm, out baseNorm);

                Vector2 curNorm = new Vector2();
                curNorm.x = mPointMasses [i].Position.x - mDerivedPos.x;
                curNorm.y = mPointMasses [i].Position.y - mDerivedPos.y;
                curNorm.Normalize();
//                Vector2.Normalize(ref curNorm, out curNorm);

                float dot;
                dot = Vector2.Dot(baseNorm, curNorm);
//                Vector2.Dot(ref baseNorm, ref curNorm, out dot);
                if (dot > 1.0f)
                {
                    dot = 1.0f;
                }
                if (dot < -1.0f)
                {
                    dot = -1.0f;
                }

                float thisAngle = (float)Math.Acos(dot);
                if (!JelloPhysics.VectorTools.isCCW(ref baseNorm, ref curNorm))
                {
                    thisAngle = -thisAngle;
                }

                if (i == 0)
                {
                    originalSign = (thisAngle >= 0.0f) ? 1 : -1;
                    originalAngle = thisAngle;
                } else
                {
                    float diff = (thisAngle - originalAngle);
                    int thisSign = (thisAngle >= 0.0f) ? 1 : -1;

                    if ((Math.Abs(diff) > Math.PI) && (thisSign != originalSign))
                    {
                        thisAngle = (thisSign == -1) ? ((float)Math.PI + ((float)Math.PI + thisAngle)) : (((float)Math.PI - thisAngle) - (float)Math.PI);
                    }
                }

                angle += thisAngle;
            }

            angle /= mPointMasses.Count;
            mDerivedAngle = angle;

            // now calculate the derived Omega, based on change in angle over time.
            float angleChange = (mDerivedAngle - mLastAngle);
            if (Math.Abs(angleChange) >= Math.PI)
            {
                if (angleChange < 0f)
                    angleChange = angleChange + (float)(Math.PI * 2);
                else
                    angleChange = angleChange - (float)(Math.PI * 2);
            }

            mDerivedOmega = angleChange / elaspsed;

            mLastAngle = mDerivedAngle;
        }

        /// <summary>
        /// Derived position of the body in global space, based on location of all PointMasses.
        /// </summary>
        public Vector2 DerivedPosition
        {
            get { return mDerivedPos; }
        }

        /// <summary>
        /// Derived global angle of the body in global space, based on location of all PointMasses.
        /// </summary>
        public float DerivedAngle
        {
            get { return mDerivedAngle; }
        }

        /// <summary>
        /// Derived global velocity of the body in global space, based on velocity of all PointMasses.
        /// </summary>
        public Vector2 DerivedVelocity
        {
            get { return mDerivedVel; }
        }

        /// <summary>
        /// Derived rotational velocity of the body in global space, based on changes in DerivedAngle.
        /// </summary>
        public float DerivedOmega
        {
            get { return mDerivedOmega; }
        }
        #endregion

        #region ACCUMULATING FORCES - TO BE INHERITED!
        /// <summary>
        /// this function should add all internal forces to the Force member variable of each PointMass in the body.
        /// these should be forces that try to maintain the shape of the body.
        /// </summary>
        public virtual void accumulateInternalForces()
        {
            CheckCollision();
        }

        /// <summary>
        /// this function should add all external forces to the Force member variable of each PointMass in the body.
        /// these are external forces acting on the PointMasses, such as gravity, etc.
        /// </summary>
        public virtual void accumulateExternalForces()
        {
        }
        #endregion

        #region INTEGRATION
        internal void integrate(float elapsed)
        {
            if (mIsStatic)
            {
                return;
            }

            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses [i].integrateForce(elapsed);
        }

        internal void dampenVelocity()
        {
            if (mIsStatic)
            {
                return;
            }

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                mPointMasses [i].Velocity.x *= mVelDamping;
                mPointMasses [i].Velocity.y *= mVelDamping;
            }
        }
        #endregion

        #region HELPER FUNCTIONS
        /// <summary>
        /// update the AABB for this body, including padding for velocity given a timestep.
        /// This function is called by the World object on Update(), so the user should not need this in most cases.
        /// </summary>
        /// <param name="elapsed">elapsed time in seconds</param>
        public void updateAABB(float elapsed, bool forceUpdate)
        {
            if ((!IsStatic) || (forceUpdate))
            {
                mAABB.clear();
                for (int i = 0; i < mPointMasses.Count; i++)
                {
                    Vector2 p = mPointMasses [i].Position;
                    mAABB.expandToInclude(ref p);

                    // expanding for velocity only makes sense for dynamic objects.
                    if (!IsStatic)
                    {
                        p.x += (mPointMasses [i].Velocity.x * elapsed);
                        p.y += (mPointMasses [i].Velocity.y * elapsed);
                        mAABB.expandToInclude(ref p);
                    }
                }
            }
        }

        /// <summary>
        /// get the Axis-aligned bounding box for this body.  used for broad-phase collision checks.
        /// </summary>
        /// <returns>AABB for this body</returns>
        public AABB getAABB()
        {
            return mAABB;
        }

        /// <summary>
        /// collision detection.  detect if a global point is inside this body.
        /// </summary>
        /// <param name="pt">point in global space</param>
        /// <returns>true = point is inside body, false = it is not.</returns>
        public bool contains(ref Vector2 pt)
        {
            // basic idea: draw a line from the point to a point known to be outside the body.  count the number of
            // lines in the polygon it intersects.  if that number is odd, we are inside.  if it's even, we are outside.
            // in this implementation we will always use a line that moves off in the positive X direction from the point
            // to simplify things.
            Vector2 endPt = new Vector2();
            endPt.x = mAABB.Max.x + 0.1f;
            endPt.y = pt.y;

            // line we are testing against goes from pt -> endPt.
            bool inside = false;
            Vector2 edgeSt = mPointMasses [0].Position;
            Vector2 edgeEnd = new Vector2();
            int c = mPointMasses.Count;
            for (int i = 0; i < c; i++)
            {
                // the current edge is defined as the line from edgeSt -> edgeEnd.
                if (i < (c - 1))
                    edgeEnd = mPointMasses [i + 1].Position;
                else
                    edgeEnd = mPointMasses [0].Position;

                // perform check now...
                if (((edgeSt.y <= pt.y) && (edgeEnd.y > pt.y)) || ((edgeSt.y > pt.y) && (edgeEnd.y <= pt.y)))
                {
                    // this line crosses the test line at some point... does it do so within our test range?
                    float slope = (edgeEnd.x - edgeSt.x) / (edgeEnd.y - edgeSt.y);
                    float hitX = edgeSt.x + ((pt.y - edgeSt.y) * slope);

                    if ((hitX >= pt.x) && (hitX <= endPt.x))
                        inside = !inside;
                }
                edgeSt = edgeEnd;
            }

            return inside;
        }

        private List<Body> _collisionBodies = new List<Body>();
        private void CheckCollision()
        {
            Body[] otherPressureBodies = FindObjectsOfType<Body>();
            if (otherPressureBodies.Length > 1)
            {
                otherPressureBodies = otherPressureBodies.OrderBy(n => Vector2.Distance(n.DerivedPosition, this.DerivedPosition)).ToArray();
                otherPressureBodies = otherPressureBodies.Where(n => n!=this).ToArray();
                int i = 0, l = otherPressureBodies.Length;
                for (; i<l; ++i)
                {
                    Body b = otherPressureBodies [i];
                    int j = 0, k = b.mPointMasses.Count;
                    bool hit = false;
                    for (; j<k; ++j)
                    {
                        if (contains(ref b.mPointMasses[j].Position))
                        {
                            hit = true;
                            break;
                        }
                    }
                
                    if (hit)
                    {
                        if(_collisionBodies.Contains(b))
                        {
                            CollisionStay(b);
                        }
                        else
                        {
                            CollisionEnter(b);
                            _collisionBodies.Add(b);
                        }
                    }
                    else
                    {
                        if(_collisionBodies.Contains(b))
                        {
                            CollisionExit(b);
                            _collisionBodies.Remove(b);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Collision Entered. For override.
        /// </summary>
        /// <param name="otherBody">Other body.</param>
        public virtual void CollisionEnter(Body otherBody) 
        {
            if (OnCollisionEnter != null)
            {
                OnCollisionEnter(otherBody);
            }
        }

        /// <summary>
        /// Collision Stay. For override.
        /// </summary>
        /// <param name="otherBody">Other body.</param>
        public virtual void CollisionStay(Body otherBody) 
        {
            if (OnCollisionStay != null)
            {
                OnCollisionStay(otherBody);
            }
        }

        /// <summary>
        /// Collision Exit. For override.
        /// </summary>
        /// <param name="otherBody">Other body.</param>
        public virtual void CollisionExit(Body otherBody)
        {
            if (OnCollisionExit != null)
            {
                OnCollisionExit(otherBody);
            }
        }

        /// <summary>
        /// collision detection - given a global point, find the point on this body that is closest to the global point,
        /// and if it is an edge, information about the edge it resides on.
        /// </summary>
        /// <param name="pt">global point</param>
        /// <param name="hitPt">returned point on the body in global space</param>
        /// <param name="normal">returned normal on the body in global space</param>
        /// <param name="pointA">returned ptA on the edge</param>
        /// <param name="pointB">returned ptB on the edge</param>
        /// <param name="edgeD">scalar distance between ptA and ptB [0,1]</param>
        /// <returns>distance</returns>
        public float getClosestPoint(Vector2 pt, out Vector2 hitPt, out Vector2 normal, out int pointA, out int pointB, out float edgeD)
        {
            hitPt = Vector2.zero;
            pointA = -1;
            pointB = -1;
            edgeD = 0f;
            normal = Vector2.zero;

            float closestD = 1000.0f;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                Vector2 tempHit;
                Vector2 tempNorm;
                float tempEdgeD;

                float dist = getClosestPointOnEdge(pt, i, out tempHit, out tempNorm, out tempEdgeD);
                if (dist < closestD)
                {
                    closestD = dist;
                    pointA = i;
                    if (i < (mPointMasses.Count - 1))
                        pointB = i + 1;
                    else
                        pointB = 0;
                    edgeD = tempEdgeD;
                    normal = tempNorm;
                    hitPt = tempHit;
                }
            }


            // return.
            return closestD;
        }

        /// <summary>
        /// find the distance from a global point in space, to the closest point on a given edge of the body.
        /// </summary>
        /// <param name="pt">global point</param>
        /// <param name="edgeNum">edge to check against.  0 = edge from pt[0] to pt[1], etc.</param>
        /// <param name="hitPt">returned point on edge in global space</param>
        /// <param name="normal">returned normal on edge in global space</param>
        /// <param name="edgeD">returned distance along edge from ptA to ptB [0,1]</param>
        /// <returns>distance</returns>
        public float getClosestPointOnEdge(Vector2 pt, int edgeNum, out Vector2 hitPt, out Vector2 normal, out float edgeD)
        {
            hitPt = new Vector2();
            hitPt.x = 0f;
            hitPt.y = 0f;

            normal = new Vector2();
            normal.x = 0f;
            normal.y = 0f;

            edgeD = 0f;
            float dist = 0f;

            Vector2 ptA = mPointMasses [edgeNum].Position;
            Vector2 ptB = new Vector2();

            if (edgeNum < (mPointMasses.Count - 1))
                ptB = mPointMasses [edgeNum + 1].Position;
            else
                ptB = mPointMasses [0].Position;

            Vector2 toP = new Vector2();
            toP.x = pt.x - ptA.x;
            toP.y = pt.y - ptA.y;

            Vector2 E = new Vector2();
            E.x = ptB.x - ptA.x;
            E.y = ptB.y - ptA.y;

            // get the length of the edge, and use that to normalize the vector.
            float edgeLength = (float)Math.Sqrt((E.x * E.x) + (E.y * E.y));
            if (edgeLength > 0.00001f)
            {
                E.x /= edgeLength;
                E.y /= edgeLength;
            }

            // normal
            Vector2 n = new Vector2();
            VectorTools.getPerpendicular(ref E, ref n);

            // calculate the distance!
            float x;
//            Vector2.Dot(ref toP, ref E, out x);
            x = Vector2.Dot(toP, E);
            if (x <= 0.0f)
            {
                // x is outside the line segment, distance is from pt to ptA.
                //dist = (pt - ptA).Length();
//                Vector2.Distance(ref pt, ref ptA, out dist);
                dist = Vector2.Distance(pt, ptA);
                hitPt = ptA;
                edgeD = 0f;
                normal = n;
            } else if (x >= edgeLength)
            {
                // x is outside of the line segment, distance is from pt to ptB.
                //dist = (pt - ptB).Length();
//                Vector2.Distance(ref pt, ref ptB, out dist);
                dist = Vector2.Distance(pt, ptB);
                hitPt = ptB;
                edgeD = 1f;
                normal = n;
            } else
            {
                // point lies somewhere on the line segment.
                Vector3 toP3 = new Vector3();
                toP3.x = toP.x;
                toP3.y = toP.y;

                Vector3 E3 = new Vector3();
                E3.x = E.x;
                E3.y = E.y;

                //dist = Math.Abs(Vector3.Cross(toP3, E3).z);
//                Vector3.Cross(ref toP3, ref E3, out E3);
                E3 = Vector3.Cross(toP3, E3);
                dist = Mathf.Abs(E3.z);
                hitPt.x = ptA.x + (E.x * x);
                hitPt.y = ptA.y + (E.y * x);
                edgeD = x / edgeLength;
                normal = n;
            }

            return dist;
        }

        /// <summary>
        /// find the squared distance from a global point in space, to the closest point on a given edge of the body.
        /// </summary>
        /// <param name="pt">global point</param>
        /// <param name="edgeNum">edge to check against.  0 = edge from pt[0] to pt[1], etc.</param>
        /// <param name="hitPt">returned point on edge in global space</param>
        /// <param name="normal">returned normal on edge in global space</param>
        /// <param name="edgeD">returned distance along edge from ptA to ptB [0,1]</param>
        /// <returns>distance</returns>
        public float getClosestPointOnEdgeSquared(Vector2 pt, int edgeNum, out Vector2 hitPt, out Vector2 normal, out float edgeD)
        {
            hitPt = new Vector2();
            hitPt.x = 0f;
            hitPt.y = 0f;

            normal = new Vector2();
            normal.x = 0f;
            normal.y = 0f;

            edgeD = 0f;
            float dist = 0f;

            Vector2 ptA = mPointMasses [edgeNum].Position;
            Vector2 ptB = new Vector2();

            if (edgeNum < (mPointMasses.Count - 1))
                ptB = mPointMasses [edgeNum + 1].Position;
            else
                ptB = mPointMasses [0].Position;

            Vector2 toP = new Vector2();
            toP.x = pt.x - ptA.x;
            toP.y = pt.y - ptA.y;

            Vector2 E = new Vector2();
            E.x = ptB.x - ptA.x;
            E.y = ptB.y - ptA.y;

            // get the length of the edge, and use that to normalize the vector.
            float edgeLength = (float)Math.Sqrt((E.x * E.x) + (E.y * E.y));
            if (edgeLength > 0.00001f)
            {
                E.x /= edgeLength;
                E.y /= edgeLength;
            }

            // normal
            Vector2 n = new Vector2();
            VectorTools.getPerpendicular(ref E, ref n);

            // calculate the distance!
            float x;
//            Vector2.Dot(ref toP, ref E, out x);
            x = Vector2.Dot(toP, E);
            if (x <= 0.0f)
            {
                // x is outside the line segment, distance is from pt to ptA.
                //dist = (pt - ptA).Length();
//                Vector2.DistanceSquared(ref pt, ref ptA, out dist);
                dist = Vector2.Distance(pt, ptA);
                dist = dist * dist;
                hitPt = ptA;
                edgeD = 0f;
                normal = n;
            } else if (x >= edgeLength)
            {
                // x is outside of the line segment, distance is from pt to ptB.
                //dist = (pt - ptB).Length();
//                Vector2.DistanceSquared(ref pt, ref ptB, out dist);
                dist = Vector2.Distance(pt, ptB);
                dist = dist * dist;
                hitPt = ptB;
                edgeD = 1f;
                normal = n;
            } else
            {
                // point lies somewhere on the line segment.
                Vector3 toP3 = new Vector3();
                toP3.x = toP.x;
                toP3.y = toP.y;

                Vector3 E3 = new Vector3();
                E3.x = E.x;
                E3.y = E.y;

                //dist = Math.Abs(Vector3.Cross(toP3, E3).z);
//                Vector3.Cross(ref toP3, ref E3, out E3);
                E3 = Vector3.Cross(toP3, E3);
                dist = Mathf.Abs(E3.z * E3.z);
                hitPt.x = ptA.x + (E.x * x);
                hitPt.y = ptA.y + (E.y * x);
                edgeD = x / edgeLength;
                normal = n;
            }

            return dist;
        }


        /// <summary>
        /// Find the closest PointMass in this body, givena global point.
        /// </summary>
        /// <param name="pos">global point</param>
        /// <param name="dist">returned dist</param>
        /// <returns>index of the PointMass</returns>
        public int getClosestPointMass(Vector2 pos, out float dist)
        {
            float closestSQD = 100000.0f;
            int closest = -1;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                float thisD = (pos - mPointMasses [i].Position).SqrMagnitude();
                if (thisD < closestSQD)
                {
                    closestSQD = thisD;
                    closest = i;
                }
            }

            dist = (float)Math.Sqrt(closestSQD);
            return closest;
        }

        /// <summary>
        /// Number of PointMasses in the body
        /// </summary>
        public int PointMassCount
        {
            get { return mPointMasses.Count; }
        }

        /// <summary>
        /// Get a specific PointMass from this body.
        /// </summary>
        /// <param name="index">index</param>
        /// <returns>PointMass</returns>
        public PointMass getPointMass(int index)
        {
            return mPointMasses [index];
        }

        /// <summary>
        /// Helper function to add a global force acting on this body as a whole.
        /// </summary>
        /// <param name="pt">location of force, in global space</param>
        /// <param name="force">direction and intensity of force, in global space</param>
        public void addGlobalForce(ref Vector2 pt, ref Vector2 force)
        {
            Vector2 R = (mDerivedPos - pt);
            
            float torqueF = Vector3.Cross(JelloPhysics.VectorTools.vec3FromVec2(R), JelloPhysics.VectorTools.vec3FromVec2(force)).z;

            for (int i = 0; i < mPointMasses.Count; i++)
            {
                Vector2 toPt = (mPointMasses [i].Position - mDerivedPos);
                Vector2 torque = JelloPhysics.VectorTools.rotateVector(toPt, -(float)(Math.PI) / 2f);

                mPointMasses [i].Force += torque * torqueF;

                mPointMasses [i].Force += force;
            }
        }
        #endregion

        void OnDrawGizmos()
        {
            mBaseShape.transformVertices(ref mDerivedPos, mDerivedAngle, ref mScale, ref mGlobalShape);
            
            VertexPositionColor[] debugVerts = new VertexPositionColor[mGlobalShape.Length + 1];
            for (int i = 0; i < mGlobalShape.Length; i++)
            {
                debugVerts [i] = new VertexPositionColor();
                debugVerts [i].Position = VectorTools.vec3FromVec2(mGlobalShape [i]);
                debugVerts [i].Color = Color.red;
                Gizmos.color = Color.red;
                if (i != 0)
                {
                    Gizmos.DrawLine(debugVerts [i - 1].Position, debugVerts [i].Position);
                }
            }
            debugVerts [debugVerts.Length - 1] = new VertexPositionColor();
            debugVerts [debugVerts.Length - 1].Position = VectorTools.vec3FromVec2(mGlobalShape [0]);
            debugVerts [debugVerts.Length - 1].Color = Color.gray;
            Gizmos.DrawLine(debugVerts [debugVerts.Length - 1].Position, debugVerts [0].Position);
            
        }

        #region PUBLIC PROPERTIES
        /// <summary>
        /// Gets / Sets whether this is a static body.  setting static greatly improves performance on static bodies.
        /// </summary>
        public bool IsStatic
        {
            get { return mIsStatic; }
            set { mIsStatic = value; }
        }

        /// <summary>
        /// Sets whether this body is kinematically controlled.  kinematic control requires shape-matching forces to work properly.
        /// </summary>
        public bool IsKinematic
        {
            get { return mKinematic; }
            set { mKinematic = value; }
        }

        public float VelocityDamping
        {
            get { return mVelDamping; }
            set { mVelDamping = value; }
        }

        public object ObjectTag
        {
            get { return mObjectTag; }
            set { mObjectTag = value; }
        }
        #endregion
    }
}
