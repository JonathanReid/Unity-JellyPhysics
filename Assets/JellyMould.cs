using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Poly2Tri;

namespace JelloPhysics
{
    public class JellyMould : MonoBehaviour
    {

        public enum JellyType
        {
            Spring,
            Pressure,
        }

        public JellyType type;
        public int detail = 1;
		public List<Vector2> springs;
		public float SpringK;
		public float Damping;
		private Shape shape;

        private void Start()
        {
			this.shape = GetComponent<Shape> ();

            SpriteRenderer sprite = gameObject.GetComponent<SpriteRenderer>();
            PolygonCollider2D col = gameObject.AddComponent<PolygonCollider2D>();

            Vector2[] points = col.points;

            Destroy(col);

            ClosedShape shape = gameObject.AddComponent<ClosedShape>();
        
            shape.begin();

			int i = this.shape.Points.Length-1, l = 0;
			for(;i>l;--i)
			{
				shape.addVertex( this.shape.Points[i]);
			}
//			shape.addVertex(this.shape.Points[0]);

//			shape.addVertex(new Vector2(0f, -1.0f));
//			shape.addVertex(new Vector2(-1.0f, 0f));
//	        shape.addVertex(new Vector2(0f, 1.0f));
//	        shape.addVertex(new Vector2(1.0f, 0f));

			shape.finish();

			DraggableSpringBody body = gameObject.AddComponent<DraggableSpringBody>();
			body.Setup(JellyWorldManager.Instance.World, shape, 1f, SpringK, Damping, SpringK, Damping, transform.position, 0, Vector2.one);
			
			foreach (Vector2 v in springs) {
				body.addInternalSpring((int)v.x, (int)v.y,SpringK,Damping);
			}
//			
//			switch (type)
//			{
//			case JellyType.Spring:
//				AddSpringBody(shape);
//				break;
//			case JellyType.Pressure:
//                    AddPressureBody(shape);
//                    break;
//            }
//
			sprite.enabled = false;
        }

        private void AddSpringBody(ClosedShape shape)
        {
            DraggableSpringBody body = gameObject.AddComponent<DraggableSpringBody>();
			body.Setup(JellyWorldManager.Instance.World, shape, 1f, SpringK, Damping, SpringK, Damping, transform.position, transform.rotation.eulerAngles.z, Vector2.one);
             
			foreach (Vector2 v in springs) {
				body.addInternalSpring((int)v.x, (int)v.y,SpringK,Damping);
			}


//			foreach(Pol
//
//			body.addInternalSpring(1, 14, 300.0f, 10.0f);
//			body.addInternalSpring(1, 15, 300.0f, 10.0f);
//			body.addInternalSpring(1, 5, 300.0f, 10.0f);
//			body.addInternalSpring(2, 14, 300.0f, 10.0f);
//			body.addInternalSpring(2, 5, 300.0f, 10.0f);
//			body.addInternalSpring(1, 5, 300.0f, 10.0f);
//			body.addInternalSpring(14, 5, 300.0f, 10.0f);
//			body.addInternalSpring(2, 4, 300.0f, 10.0f);
//			body.addInternalSpring(3, 5, 300.0f, 10.0f);
//			body.addInternalSpring(14, 6, 300.0f, 10.0f);
//			body.addInternalSpring(5, 13, 300.0f, 10.0f);
//			body.addInternalSpring(13, 6, 300.0f, 10.0f);
//			body.addInternalSpring(12, 10, 300.0f, 10.0f);
//			body.addInternalSpring(13, 11, 300.0f, 10.0f);
//			body.addInternalSpring(13, 10, 300.0f, 10.0f);
//			body.addInternalSpring(13, 9, 300.0f, 10.0f);
//			body.addInternalSpring(6, 10, 300.0f, 10.0f);
//			body.addInternalSpring(6, 9, 300.0f, 10.0f);
//			body.addInternalSpring(6, 8, 300.0f, 10.0f);
//			body.addInternalSpring(7, 9, 300.0f, 10.0f);
			
//			// polygons!
//			body.addTriangle(0, 15, 1);
//			body.addTriangle(1, 15, 14);
//			body.addTriangle(1, 14, 5);
//			body.addTriangle(1, 5, 2);
//			body.addTriangle(2, 5, 4);
//			body.addTriangle(2, 4, 3);
//			body.addTriangle(14, 13, 6);
//			body.addTriangle(14, 6, 5);
//			body.addTriangle(12, 11, 10);
//			body.addTriangle(12, 10, 13);
//			body.addTriangle(13, 10, 9);
//			body.addTriangle(13, 9, 6);
//			body.addTriangle(6, 9, 8);
//			body.addTriangle(6, 8, 7);
//			body.finalizeTriangles(Color.grey, Color.grey);
		}
		
		private void AddPressureBody(ClosedShape shape)
		{
			DraggablePressureBody pb = gameObject.AddComponent<DraggablePressureBody>();
			pb.Setup(JellyWorldManager.Instance.World, shape, 1.0f, 40.0f, 10.0f, 1.0f, 300.0f, 20.0f, transform.position, transform.rotation.eulerAngles.z, Vector2.one);
			
			pb.addInternalSpring(0, 2, 400f, 12f);
			pb.addInternalSpring(1, 3, 400f, 12f);
		}
	}
}
