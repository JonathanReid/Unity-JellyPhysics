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
			for(;i>l;i-=detail)
			{
				shape.addVertex( this.shape.Points[i]);
			}

			if(i!=l)
			{
				shape.addVertex(this.shape.Points[0]);
			}

			shape.finish();
				
			switch (type)
			{
			case JellyType.Spring:
				AddSpringBody(shape);
				break;
			case JellyType.Pressure:
                    AddPressureBody(shape);
                    break;
            }

			sprite.enabled = false;
        }

        private void AddSpringBody(ClosedShape shape)
        {
            DraggableSpringBody body = gameObject.AddComponent<DraggableSpringBody>();
			body.Setup(JellyWorldManager.Instance.World, shape, 1f, SpringK, Damping, SpringK, Damping, transform.position, transform.rotation.eulerAngles.z, Vector2.one);
             
			int i = springs.Count-1, l = -1;
			for(;i>l;i-=detail)
			{
				Vector2 v = springs[i];
				try{
				body.addInternalSpring((int)v.x, (int)v.y,SpringK,Damping);
				}
				catch{
//					Debug.Log("couldnt add spring at " +i);
				}
			}

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
