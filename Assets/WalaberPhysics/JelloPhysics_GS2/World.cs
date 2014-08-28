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
    /// World - this class represents the physics world, and keeps track of all Bodies in it, coordinating integration, collision, etc.
    /// </summary>
    public class World
    {
        #region PUBLIC VARIABLES

        /// <summary>
        /// Collision Filter type. return TRUE to allow collision, FALSE to ignore collision.
        /// </summary>
        /// <param name="bodyA">The colliding body</param>
        /// <param name="bodyApm">Point mass that has collided</param>
        /// <param name="bodyB">Body that bodyA collided with</param>
        /// <param name="bodyBpm1">PointMass 1 on the edge that was collided with</param>
        /// <param name="bodyBpm2">PointMass 2 on the edge that was collided with</param>
        /// <param name="hitPt">Location of collision in global space</param>
        /// <param name="normalVel">Velocity along normal of collision.</param>
        /// <returns>TRUE = accept collision, FALSE = ignore collision</returns>
        public delegate bool collisionFilter( Body bodyA, int bodyApm, Body bodyB, int bodyBpm1, int bodyBpm2, Vector2 hitPt, float normalVel );

        /// <summary>
        /// represents data about collision between 2 materials.
        /// </summary>
        public struct MaterialPair
        {
            /// <summary>
            /// Whether these 2 materials should collide with each other or not.
            /// </summary>
            public bool Collide;

            /// <summary>
            /// Amount of "bounce" when collision occurs. value range [0,1]. 0 == no bounce, 1 == 100% bounce
            /// </summary>
            public float Elasticity;

            /// <summary>
            /// Amount of friction.  Value range [0,1].  0 == no friction, 1 == 100% friction, will stop on contact.
            /// </summary>
            public float Friction;

            /// <summary>
            /// Collision filter function.
            /// </summary>
            public collisionFilter CollisionFilter;
        }

        /// <summary>
        /// number of materials created.
        /// </summary>
        public int MaterialCount
        {
            get { return mMaterialCount; }
        }
        #endregion

        #region PRIVATE VARIABLES
        List<Body> mBodies;
        AABB mWorldLimits;
        Vector2 mWorldSize;
        Vector2 mWorldGridStep;

        float mPenetrationThreshold;
        int mPenetrationCount;

        // material chart.
        MaterialPair[,] mMaterialPairs;
        MaterialPair mDefaultMatPair;
        int mMaterialCount;

        // collision information
        private struct BodyCollisionInfo
        {
            public void Clear() { bodyA = bodyB = null; bodyApm = bodyBpmA = bodyBpmB = -1; hitPt = Vector2.zero; edgeD = 0f; normal = Vector2.zero; penetration = 0f; }
            public Body bodyA;
            public int bodyApm;
            public Body bodyB;
            public int bodyBpmA;
            public int bodyBpmB;
            public Vector2 hitPt;
            public float edgeD;
            public Vector2 normal;
            public float penetration;
        };

        List<BodyCollisionInfo> mCollisionList;

        //// debug visualization variables
//        VertexDeclaration mVertexDecl = null;
        #endregion

        #region CONSTRUCTOR
        /// <summary>
        /// Creates the World object, and sets world limits to default (-20,-20) to (20,20).
        /// </summary>
        public World()
        {
            mBodies = new List<Body>();
            mCollisionList = new List<BodyCollisionInfo>();

            // initialize materials.
            mMaterialCount = 1;
            mMaterialPairs = new MaterialPair[1,1];
            mDefaultMatPair.Friction = 0.3f;
            mDefaultMatPair.Elasticity = 0.8f;
            mDefaultMatPair.Collide = true;
            mDefaultMatPair.CollisionFilter = new collisionFilter(this.defaultCollisionFilter);
            
            mMaterialPairs[0, 0] = mDefaultMatPair;

            Vector2 min = new Vector2(-20.0f, -20.0f);
            Vector2 max = new Vector2(20.0f, 20.0f);
            setWorldLimits(min, max);

            mPenetrationThreshold = 0.3f;
        }
        #endregion

        #region WORLD SIZE
        public void setWorldLimits(Vector2 min, Vector2 max)
        {
            mWorldLimits = new AABB(ref min, ref max);
            mWorldSize = max - min;
            mWorldGridStep = mWorldSize / 32;
        }
        #endregion

        #region MATERIALS
        /// <summary>
        /// Add a new material to the world.  all previous material data is kept intact.
        /// </summary>
        /// <returns>int ID of the newly created material</returns>
        public int addMaterial()
        {
            MaterialPair[,] old = mMaterialPairs;
            mMaterialCount++;

            mMaterialPairs = new MaterialPair[mMaterialCount, mMaterialCount];

            // replace old data.
            for (int i = 0; i < mMaterialCount; i++)
            {
                for (int j = 0; j < mMaterialCount; j++)
                {
                    if ((i < (mMaterialCount-1)) && (j < (mMaterialCount-1)))
                        mMaterialPairs[i, j] = old[i, j];
                    else
                        mMaterialPairs[i, j] = mDefaultMatPair;
                }
            }

            return mMaterialCount - 1;
        }

        /// <summary>
        /// Enable or Disable collision between 2 materials.
        /// </summary>
        /// <param name="a">material ID A</param>
        /// <param name="b">material ID B</param>
        /// <param name="collide">true = collide, false = ignore collision</param>
        public void setMaterialPairCollide(int a, int b, bool collide)
        {
            if ((a >= 0) && (a < mMaterialCount) && (b >= 0) && (b < mMaterialCount))
            {
                mMaterialPairs[a, b].Collide = collide;
                mMaterialPairs[b, a].Collide = collide;
            }
        }

        /// <summary>
        /// Set the collision response variables for a pair of materials.
        /// </summary>
        /// <param name="a">material ID A</param>
        /// <param name="b">material ID B</param>
        /// <param name="friction">friction.  [0,1] 0 = no friction, 1 = 100% friction</param>
        /// <param name="elasticity">"bounce" [0,1] 0 = no bounce (plastic), 1 = 100% bounce (super ball)</param>
        public void setMaterialPairData(int a, int b, float friction, float elasticity)
        {
            if ((a >= 0) && (a < mMaterialCount) && (b >= 0) && (b < mMaterialCount))
            {
                mMaterialPairs[a, b].Friction = friction;
                mMaterialPairs[a, b].Elasticity = elasticity;

                mMaterialPairs[b, a].Friction = friction;
                mMaterialPairs[b, a].Elasticity = elasticity;
            }
        }

        /// <summary>
        /// Sets a user function to call when 2 bodies of the given materials collide.
        /// </summary>
        /// <param name="a">Material A</param>
        /// <param name="b">Material B</param>
        /// <param name="filter">User fuction (delegate)</param>
        public void setMaterialPairFilterCallback(int a, int b, collisionFilter filter)
        {
            if ((a >= 0) && (a < mMaterialCount) && (b >= 0) && (b < mMaterialCount))
            {
                mMaterialPairs[a, b].CollisionFilter += filter;

                mMaterialPairs[b, a].CollisionFilter += filter;
            }
        }
        #endregion

        #region ADDING / REMOVING BODIES
        /// <summary>
        /// Add a Body to the world.  Bodies do this automatically, you should NOT need to call this.
        /// </summary>
        /// <param name="b">the body to add to the world</param>
        public void addBody(Body b)
        {
            if (!mBodies.Contains(b))
            {
                mBodies.Add(b);
            }
        }

        /// <summary>
        /// Remove a body from the world.  call this outside of an update to remove the body.
        /// </summary>
        /// <param name="b">the body to remove</param>
        public void removeBody(Body b)
        {
            if (mBodies.Contains(b))
            {
                mBodies.Remove(b);
            }
        }

        /// <summary>
        /// Get a body at a specific index.
        /// </summary>
        /// <param name="index"></param>
        /// <returns></returns>
        public Body getBody(int index)
        {
            if ((index >= 0) && (index < mBodies.Count))
                return mBodies[index];

            return null;
        }
        #endregion

        #region BODY HELPERS
        /// <summary>
        /// Find the closest PointMass in the world to a given point.
        /// </summary>
        /// <param name="pt">global point</param>
        /// <param name="bodyID">index of the body that contains the PointMass</param>
        /// <param name="pmID">index of the PointMass</param>
        public void getClosestPointMass(Vector2 pt, out int bodyID, out int pmID)
        {
            bodyID = -1;
            pmID = -1;

            float closestD = 1000.0f;
            for (int i = 0; i < mBodies.Count; i++)
            {
                float dist = 0f;
                int pm = mBodies[i].getClosestPointMass(pt, out dist);
                if (dist < closestD)
                {
                    closestD = dist;
                    bodyID = i;
                    pmID = pm;
                }
            }
        }

        /// <summary>
        /// Given a global point, get a body (if any) that contains this point.
        /// Useful for picking objects with a cursor, etc.
        /// </summary>
        /// <param name="pt">global point</param>
        /// <returns>Body (or null)</returns>
        public Body getBodyContaining(ref Vector2 pt)
        {
            for (int i = 0; i < mBodies.Count; i++)
            {
                if (mBodies[i].contains(ref pt))
                    return mBodies[i];
            }

            return null;
        }
        #endregion

        #region UPDATE
        /// <summary>
        /// Update the world by a specific timestep.
        /// </summary>
        /// <param name="elapsed">elapsed time in seconds</param>
        public void update(float elapsed)
        {
            mPenetrationCount = 0;

            // first, accumulate all forces acting on PointMasses.
            for (int i = 0; i < mBodies.Count; i++)
            {
                mBodies[i].derivePositionAndAngle(elapsed);
                mBodies[i].accumulateExternalForces();
                mBodies[i].accumulateInternalForces();
            }

            // now integrate.
            for (int i = 0; i < mBodies.Count; i++)
                mBodies[i].integrate(elapsed);

            // update all bounding boxes, and then bitmasks.
            for (int i = 0; i < mBodies.Count; i++)
            {
                mBodies[i].updateAABB(elapsed, false);
                updateBodyBitmask(mBodies[i]);
            }

            // now check for collision.
            // inter-body collision!
            for (int i = 0; i < mBodies.Count; i++)
            {
                for (int j = i + 1; j < mBodies.Count; j++)
                {
                    // early out - these bodies materials are set NOT to collide
                    if (!mMaterialPairs[mBodies[i].Material, mBodies[j].Material].Collide)
                        continue;

                    // another early-out - both bodies are static.
                    if ((mBodies[i].IsStatic) && (mBodies[j].IsStatic))
                        continue;

                    // grid-based early out.
                    if (((mBodies[i].mBitMaskX.mask & mBodies[j].mBitMaskX.mask) == 0) && 
                        ((mBodies[i].mBitMaskY.mask & mBodies[j].mBitMaskY.mask) == 0))
                        continue;

                    // broad-phase collision via AABB.
                    AABB boxA = mBodies[i].getAABB();
                    AABB boxB = mBodies[j].getAABB();

                    // early out
                    if (!boxA.intersects(ref boxB))
                        continue;

                    // okay, the AABB's of these 2 are intersecting.  now check for collision of A against B.
                    bodyCollide(mBodies[i], mBodies[j], mCollisionList);

                    // and the opposite case, B colliding with A
                    bodyCollide(mBodies[j], mBodies[i], mCollisionList);
                }
            }

            // now handle all collisions found during the update at once.
            _handleCollisions();

            // now dampen velocities.
            for (int i = 0; i < mBodies.Count; i++)
                mBodies[i].dampenVelocity();
                
        }

        private void updateBodyBitmask(Body body)
        {
            AABB box = body.getAABB();

            int minX = (int)Math.Floor((box.Min.x - mWorldLimits.Min.x) / mWorldGridStep.x);
            int maxX = (int)Math.Floor((box.Max.x - mWorldLimits.Min.x) / mWorldGridStep.x);

            if (minX < 0) { minX = 0; } else if (minX > 32) { minX = 32; }
            if (maxX < 0) { maxX = 0; } else if (maxX > 32) { maxX = 32; }

            int minY = (int)Math.Floor((box.Min.y - mWorldLimits.Min.y) / mWorldGridStep.y);
            int maxY = (int)Math.Floor((box.Max.y - mWorldLimits.Min.y) / mWorldGridStep.y);

            if (minY < 0) { minY = 0; } else if (minY > 32) { minY = 32; }
            if (maxY < 0) { maxY = 0; } else if (maxY > 32) { maxY = 32; }

            body.mBitMaskX.clear();
            for (int i = minX; i <= maxX; i++)
                body.mBitMaskX.setOn(i);

            body.mBitMaskY.clear();
            for (int i = minY; i <= maxY; i++)
                body.mBitMaskY.setOn(i);

            //Console.WriteLine("Body bitmask: minX{0} maxX{1} minY{2} maxY{3}", minX, maxX, minY, minY, maxY);
        }
        #endregion

        #region COLLISION CHECKS / RESPONSE
        private void bodyCollide(Body bA, Body bB, List<BodyCollisionInfo> infoList)
        {
            int bApmCount = bA.PointMassCount;
            int bBpmCount = bB.PointMassCount;

            AABB boxB = bB.getAABB();

            // check all PointMasses on bodyA for collision against bodyB.  if there is a collision, return detailed info.
            BodyCollisionInfo infoAway = new BodyCollisionInfo();
            BodyCollisionInfo infoSame = new BodyCollisionInfo();
            for (int i = 0; i < bApmCount; i++)
            {
                Vector2 pt = bA.getPointMass(i).Position;

                // early out - if this point is outside the bounding box for bodyB, skip it!
                if (!boxB.contains(ref pt))
                    continue;

                // early out - if this point is not inside bodyB, skip it!
                if (!bB.contains(ref pt))
                    continue;

                int prevPt = (i>0) ? i-1 : bApmCount-1;
                int nextPt = (i < bApmCount - 1) ? i + 1 : 0;

                Vector2 prev = bA.getPointMass(prevPt).Position;
                Vector2 next = bA.getPointMass(nextPt).Position;

                // now get the normal for this point. (NOT A UNIT VECTOR)
                Vector2 fromPrev = new Vector2();
                fromPrev.x = pt.x - prev.x;
                fromPrev.y = pt.y - prev.y;

                Vector2 toNext = new Vector2();
                toNext.x = next.x - pt.x;
                toNext.y = next.y - pt.y;

                Vector2 ptNorm = new Vector2();
                ptNorm.x = fromPrev.x + toNext.x;
                ptNorm.y = fromPrev.y + toNext.y;
                VectorTools.makePerpendicular(ref ptNorm);
                
                // this point is inside the other body.  now check if the edges on either side intersect with and edges on bodyB.          
                float closestAway = 100000.0f;
                float closestSame = 100000.0f;

                infoAway.Clear();
                infoAway.bodyA = bA;
                infoAway.bodyApm = i;
                infoAway.bodyB = bB;

                infoSame.Clear();
                infoSame.bodyA = bA;
                infoSame.bodyApm = i;
                infoSame.bodyB = bB;

                bool found = false;

                int b1 = 0;
                int b2 = 1;
                for (int j = 0; j < bBpmCount; j++)
                {
                    Vector2 hitPt;
                    Vector2 norm;
                    float edgeD;

                    b1 = j;

                    if (j < bBpmCount - 1)
                        b2 = j + 1;
                    else
                        b2 = 0;

                    Vector2 pt1 = bB.getPointMass(b1).Position;
                    Vector2 pt2 = bB.getPointMass(b2).Position;

                    // quick test of distance to each point on the edge, if both are greater than current mins, we can skip!
                    float distToA = ((pt1.x - pt.x) * (pt1.x - pt.x)) + ((pt1.y - pt.y) * (pt1.y - pt.y));
                    float distToB = ((pt2.x - pt.x) * (pt2.x - pt.x)) + ((pt2.y - pt.y) * (pt2.y - pt.y));
                    

                    if ((distToA > closestAway) && (distToA > closestSame) && (distToB > closestAway) && (distToB > closestSame))
                        continue;

                    // test against this edge.
                    float dist = bB.getClosestPointOnEdgeSquared(pt, j, out hitPt, out norm, out edgeD);
                    
                    // only perform the check if the normal for this edge is facing AWAY from the point normal.
                    float dot;
                    //Vector2.Dot(ref ptNorm, ref edgeNorm, out dot);
//                    Vector2.Dot(ref ptNorm, ref norm, out dot);
					dot = Vector2.Dot(ptNorm,norm);
                    if (dot <= 0f)
                    {
                        if (dist < closestAway)
                        {
                            closestAway = dist;
                            infoAway.bodyBpmA = b1;
                            infoAway.bodyBpmB = b2;
                            infoAway.edgeD = edgeD;
                            infoAway.hitPt = hitPt;
                            infoAway.normal = norm;
                            infoAway.penetration = dist;
                            found = true;
                        }
                    }
                    else
                    {
                        if (dist < closestSame)
                        {
                            closestSame = dist;
                            infoSame.bodyBpmA = b1;
                            infoSame.bodyBpmB = b2;
                            infoSame.edgeD = edgeD;
                            infoSame.hitPt = hitPt;
                            infoSame.normal = norm;
                            infoSame.penetration = dist;
                        }
                    }
                }

                // we've checked all edges on BodyB.  add the collision info to the stack.
                if ((found) && (closestAway > mPenetrationThreshold) && (closestSame < closestAway))
                {
                    infoSame.penetration = (float)Math.Sqrt(infoSame.penetration);
                    infoList.Add(infoSame);
                }
                else
                {
                    infoAway.penetration = (float)Math.Sqrt(infoAway.penetration);
                    infoList.Add(infoAway);
                }
            }

        }

        private void _handleCollisions()
        {
            // handle all collisions!
            for (int i = 0; i < mCollisionList.Count; i++)
            {
                BodyCollisionInfo info = mCollisionList[i];

                PointMass A = info.bodyA.getPointMass(info.bodyApm);
                PointMass B1 = info.bodyB.getPointMass(info.bodyBpmA);
                PointMass B2 = info.bodyB.getPointMass(info.bodyBpmB);

                // velocity changes as a result of collision.
                Vector2 bVel = new Vector2();
                bVel.x = (B1.Velocity.x + B2.Velocity.x) * 0.5f;
                bVel.y = (B1.Velocity.y + B2.Velocity.y) * 0.5f;

                Vector2 relVel = new Vector2();
                relVel.x = A.Velocity.x - bVel.x;
                relVel.y = A.Velocity.y - bVel.y;

                float relDot;
//                Vector2.Dot(ref relVel, ref info.normal, out relDot);
				relDot = Vector2.Dot(relVel,info.normal);

                // collision filter!
                if (!mMaterialPairs[info.bodyA.Material, info.bodyB.Material].CollisionFilter(info.bodyA, info.bodyApm, info.bodyB, info.bodyBpmA, info.bodyBpmB, info.hitPt, relDot))
                    continue;

                if (info.penetration > mPenetrationThreshold)
                {
                    //Console.WriteLine("penetration above Penetration Threshold!!  penetration={0}  threshold={1} difference={2}",
                    //    info.penetration, mPenetrationThreshold, info.penetration-mPenetrationThreshold);

                    mPenetrationCount++;
                    continue;
                }

                float b1inf = 1.0f - info.edgeD;
                float b2inf = info.edgeD;

                float b2MassSum = ((float.IsPositiveInfinity(B1.Mass)) || (float.IsPositiveInfinity(B2.Mass))) ? float.PositiveInfinity : (B1.Mass + B2.Mass);

                float massSum = A.Mass + b2MassSum;
                
                float Amove;
                float Bmove;
                if (float.IsPositiveInfinity(A.Mass))
                {
                    Amove = 0f;
                    Bmove = (info.penetration) + 0.001f;
                }
                else if (float.IsPositiveInfinity(b2MassSum))
                {
                    Amove = (info.penetration) + 0.001f;
                    Bmove = 0f;
                }
                else
                {
                    Amove = (info.penetration * (b2MassSum / massSum));
                    Bmove = (info.penetration * (A.Mass / massSum));
                }

                float B1move = Bmove * b1inf;
                float B2move = Bmove * b2inf;

                float AinvMass = (float.IsPositiveInfinity(A.Mass)) ? 0f : 1f / A.Mass;
                float BinvMass = (float.IsPositiveInfinity(b2MassSum)) ? 0f : 1f / b2MassSum;

                float jDenom = AinvMass + BinvMass;
                Vector2 numV = new Vector2();
                float elas = 1f + mMaterialPairs[info.bodyA.Material, info.bodyB.Material].Elasticity;
                numV.x = relVel.x * elas;
                numV.y = relVel.y * elas;

                float jNumerator;
//                Vector2.Dot(ref numV, ref info.normal, out jNumerator);
				jNumerator = Vector2.Dot(numV,info.normal);
                jNumerator = -jNumerator;

                float j = jNumerator / jDenom;

                if (!float.IsPositiveInfinity(A.Mass))
                {
                    A.Position.x += info.normal.x * Amove;
                    A.Position.y += info.normal.y * Amove;
                }

                if (!float.IsPositiveInfinity(B1.Mass))
                {
                    B1.Position.x -= info.normal.x * B1move;
                    B1.Position.y -= info.normal.y * B1move;
                }

                if (!float.IsPositiveInfinity(B2.Mass))
                {
                    B2.Position.x -= info.normal.x * B2move;
                    B2.Position.y -= info.normal.y * B2move;
                }
                
                Vector2 tangent = new Vector2();
                VectorTools.getPerpendicular(ref info.normal, ref tangent);
                float friction = mMaterialPairs[info.bodyA.Material,info.bodyB.Material].Friction;
                float fNumerator;
//                Vector2.Dot(ref relVel, ref tangent, out fNumerator);
				fNumerator = Vector2.Dot(relVel,tangent);
                fNumerator *= friction;
                float f = fNumerator / jDenom;

                // adjust velocity if relative velocity is moving toward each other.
                if (relDot <= 0.0001f)
                {
                    if (!float.IsPositiveInfinity(A.Mass))
                    {
                        A.Velocity.x += (info.normal.x * (j / A.Mass)) - (tangent.x * (f / A.Mass));
                        A.Velocity.y += (info.normal.y * (j / A.Mass)) - (tangent.y * (f / A.Mass));
                    }

                    if (!float.IsPositiveInfinity(b2MassSum))
                    {
                        B1.Velocity.x -= (info.normal.x * (j / b2MassSum) * b1inf) - (tangent.x * (f / b2MassSum) * b1inf);
                        B1.Velocity.y -= (info.normal.y * (j / b2MassSum) * b1inf) - (tangent.y * (f / b2MassSum) * b1inf);
                    }

                    if (!float.IsPositiveInfinity(b2MassSum))
                    {
                        B2.Velocity.x -= (info.normal.x * (j / b2MassSum) * b2inf) - (tangent.x * (f / b2MassSum) * b2inf);
                        B2.Velocity.y -= (info.normal.y * (j / b2MassSum) * b2inf) - (tangent.y * (f / b2MassSum) * b2inf);
                    }
                }
            }
            mCollisionList.Clear();
        }

        #endregion

        #region DEBUG VISUALIZATION
        /// <summary>
        /// draw the world extents on-screen.
        /// </summary>
        /// <param name="device">Graphics Device</param>
        /// <param name="effect">An Effect to draw the lines with (should implement vertex color diffuse)</param>
//		void OnDrawGizmos()
//        {
//            // draw the world limits.
//            VertexPositionColor[] limits = new VertexPositionColor[5];
//
//            limits[0].Position = new Vector3(mWorldLimits.Min.x, mWorldLimits.Max.y, 0);
//            limits[0].Color = Color.grey;
//
//            limits[1].Position = new Vector3(mWorldLimits.Max.x, mWorldLimits.Max.y, 0);
//			limits[1].Color = Color.grey;
//
//			Gizmos.DrawLine (limits [0].Position, limits [1].Position);
//
//			limits[2].Position = new Vector3(mWorldLimits.Max.x, mWorldLimits.Min.y, 0);
//			limits[2].Color = Color.grey;
//			Gizmos.DrawLine (limits [1].Position, limits [2].Position);
//
//			limits[3].Position = new Vector3(mWorldLimits.Min.x, mWorldLimits.Min.y, 0);
//			limits[3].Color = Color.grey;
//			Gizmos.DrawLine (limits [2].Position, limits [3].Position);
//
//			limits[4].Position = new Vector3(mWorldLimits.Min.x, mWorldLimits.Max.y, 0);
//			limits[4].Color = Color.grey;
//			Gizmos.DrawLine (limits [3].Position, limits [4].Position);
//
//        }
		/*
        /// <summary>
        /// draw the velocities of all PointMasses in the simulation on-screen in an orange/yellow color.
        /// </summary>
        /// <param name="device">GraphicsDevice</param>
        /// <param name="effect">An Effect to draw the lines with</param>
        public void debugDrawPointVelocities(GraphicsDevice device, Effect effect)
        {
            if (mVertexDecl == null)
            {
                mVertexDecl = new VertexDeclaration(device, VertexPositionColor.VertexElements);
            }

            for (int i = 0; i < mBodies.Count; i++)
            {
                VertexPositionColor[] vels = new VertexPositionColor[mBodies[i].PointMassCount * 2];

                for (int pm = 0; pm < mBodies[i].PointMassCount; pm++)
                {
                    vels[(pm * 2) + 0].Position = VectorTools.vec3FromVec2(mBodies[i].getPointMass(pm).Position);
                    vels[(pm * 2) + 0].Color = Color.yellow;
                    vels[(pm * 2) + 1].Position = VectorTools.vec3FromVec2(mBodies[i].getPointMass(pm).Position + (mBodies[i].getPointMass(pm).Velocity * 0.25f));
                    vels[(pm * 2) + 1].Color = Color.Orange;
                }

                effect.Begin();
                foreach (EffectPass pass in effect.CurrentTechnique.Passes)
                {
                    pass.Begin();
                    device.DrawUserPrimitives<VertexPositionColor>(PrimitiveType.LineList, vels, 0, mBodies[i].PointMassCount);
                    pass.End();
                }
                effect.End();
            }
        }
*/

        /// <summary>
        /// Draw all of the bodies in the world in debug mode, for quick visualization of the entire scene.
        /// </summary>
        /// <param name="device">GraphicsDevice</param>
        /// <param name="effect">An Effect to draw the lines with</param>
        /// <param name="drawAABBs"></param>
//        public void debugDrawAllBodies(GraphicsDevice device, Effect effect, bool drawAABBs)
//        {
//            for (int i = 0; i < mBodies.Count; i++)
//            {
//                if (drawAABBs)
//                    mBodies[i].debugDrawAABB(device, effect);
//
//                mBodies[i].debugDrawMe(device, effect);
//            }
//        }

        #endregion

		private bool defaultCollisionFilter(Body A, int Apm, Body B, int Bpm1, int Bpm2, Vector2 hitPt, float normSpeed)
		{
			return true;
		}

        #region PUBLIC PROPERTIES
        /// <summary>
        /// This threshold allows objects to be crushed completely flat without snapping through to the other side of objects.
        /// It should be set to a value that is slightly over half the average depth of an object for best results.  Defaults to 0.5.
        /// </summary>
        public float PenetrationThreshold
        {
            set { mPenetrationThreshold = value; }
            get { return mPenetrationThreshold; }
        }

        /// <summary>
        /// How many collisions exceeded the Penetration Threshold last update.  if this is a high number, you can assume that
        /// the simulation has "broken" (one or more objects have penetrated inside each other).
        /// </summary>
        public int PenetrationCount
        {
            get { return mPenetrationCount; }
        }
        #endregion
    }
}
