using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace JelloPhysics
{
    class DraggableSpringBody : JelloPhysics.SpringBody
    {
        // Variables for dragging point masses in this body.
        private Vector2 dragForce = Vector2.zero;
        private int dragPoint = -1;

        private VertexPositionColor[] mVerts = null;
        private int[] mIndices = null;
        private List<int> mIndexList;
		private GameObject _prevShape;

        public void Setup(JelloPhysics.World w, JelloPhysics.ClosedShape s, float massPerPoint, float shapeSpringK, float shapeSpringDamp,
            float edgeSpringK, float edgeSpringDamp, Vector2 pos, float angleInRadians, Vector2 scale)
        {
			base.Setup (w, s, massPerPoint, shapeSpringK, shapeSpringDamp, edgeSpringK, edgeSpringDamp, pos, angleInRadians, scale, false);
            mIndexList = new List<int>();
        }

        // add an indexed triangle to this primitive.
        public void addTriangle(int A, int B, int C)
        {
            mIndexList.Add(A);
            mIndexList.Add(B);
            mIndexList.Add(C);
        }

        // finalize triangles
        public void finalizeTriangles(Color c, Color d)
        {
            mVerts = new VertexPositionColor[mPointMasses.Count];

            mIndices = new int[mIndexList.Count];
            for (int i = 0; i < mIndexList.Count; i++)
                mIndices[i] = mIndexList[i];
        }

        public void setDragForce(Vector2 force, int pm)
        {
            dragForce = force;
            dragPoint = pm;
        }

        // add gravity, and drag force.
        public override void accumulateExternalForces()
        {
            base.accumulateExternalForces();

            // gravity.
            for (int i = 0; i < mPointMasses.Count; i++)
            {
                mPointMasses[i].Force += new Vector2((JellyWorldManager.Instance.Gravity.x*Gravity), (JellyWorldManager.Instance.Gravity.y*Gravity) * mPointMasses[i].Mass);
            }

            // dragging force.
            if (dragPoint != -1)
                mPointMasses[dragPoint].Force += dragForce;

            dragPoint = -1;

        }
		
		void OnDrawGizmos()
		{
			mBaseShape.transformVertices(DerivedPos, DerivedAngle, ref mScale, ref mGlobalShape);
			
			VertexPositionColor[] shape = new VertexPositionColor[mPointMasses.Count];

			mBaseShape.transformVertices(DerivedPos, DerivedAngle, ref mScale, ref mGlobalShape);
			List<Vector2> points = new List<Vector2> ();
			for (int i = 0; i < mPointMasses.Count; i++)
			{
				shape[i] = new VertexPositionColor();
				shape[i].Position = VectorTools.vec3FromVec2(mPointMasses[i].Position);
				shape[i].Color = Color.red;

				Gizmos.color = Color.white;
				if(i != 0)
				{
					Gizmos.DrawLine(shape[i-1].Position,shape[i].Position);
				}
			}

			for (int i = 0; i < mSprings.Count; i++) {
				Gizmos.color = Color.red;
				Gizmos.DrawLine(VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassA].Position),VectorTools.vec3FromVec2(mPointMasses[mSprings[i].pointMassB].Position));
			}
			Gizmos.DrawLine(shape[mPointMasses.Count-1].Position,shape[0].Position);
		}

    }
}
