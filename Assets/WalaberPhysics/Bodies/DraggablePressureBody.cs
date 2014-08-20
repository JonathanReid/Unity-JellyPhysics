using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace JelloPhysics
{
    class DraggablePressureBody : JelloPhysics.PressureBody
    {
        // Variables for dragging point masses in this body.
        private Vector2 dragForce = Vector2.zero;
        private int dragPoint = -1;

//        VertexDeclaration mDecl;
        VertexPositionColor[] mVerts = null;
        int[] mIndices = null;
        List<int> mIndexList;
        Color mColor = Color.white;

        public void Setup(JelloPhysics.World w, JelloPhysics.ClosedShape s, float massPerPoint, float gasPressure,
            float shapeSpringK, float shapeSpringDamp, float edgeSpringK, float edgeSpringDamp, Vector2 pos, float angleInRadians, Vector2 scale)
        {
			base.Setup (w, s, massPerPoint, gasPressure, shapeSpringK, shapeSpringDamp, edgeSpringK, edgeSpringDamp, pos, angleInRadians, scale, false);
            mIndexList = new List<int>();
        }

        public void setDragForce(Vector2 force, int pm)
        {
            dragForce = force;
            dragPoint = pm;
        }

        // add an indexed triangle to this primitive.
        public void addTriangle(int A, int B, int C)
        {
            mIndexList.Add(A);
            mIndexList.Add(B);
            mIndexList.Add(C);
        }

        // finalize triangles
        public void finalizeTriangles(Color c)
        {
            mVerts = new VertexPositionColor[mPointMasses.Count];

            mIndices = new int[mIndexList.Count];
            for (int i = 0; i < mIndexList.Count; i++)
                mIndices[i] = mIndexList[i];

            mColor = c;
        }

        // add gravity, and drag force.
        public override void accumulateExternalForces()
        {
            base.accumulateExternalForces();

            // gravity.
            for (int i = 0; i < mPointMasses.Count; i++)
                mPointMasses[i].Force += new Vector2(0.0f, -9.8f * mPointMasses[i].Mass);

            if (dragPoint != -1)
                mPointMasses[dragPoint].Force += dragForce;

            dragPoint = -1;
        }

		void OnDrawGizmos()
		{
			mBaseShape.transformVertices(ref mDerivedPos, mDerivedAngle, ref mScale, ref mGlobalShape);
			
			VertexPositionColor[] shape = new VertexPositionColor[mPointMasses.Count];
			
			mBaseShape.transformVertices(ref mDerivedPos, mDerivedAngle, ref mScale, ref mGlobalShape);
			for (int i = 0; i < mPointMasses.Count; i++)
			{
				shape[i] = new VertexPositionColor();
				shape[i].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position);
				shape[i].Color = Color.red;
				if(i != 0)
				{
					Gizmos.DrawLine(shape[i-1].Position,shape[i].Position);
				}
			}
			
			Gizmos.DrawLine(shape[mPointMasses.Count-1].Position,shape[0].Position);
		}

    }
}
