using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Poly2Tri;

namespace JelloPhysics
{
    public class JellyObject : MonoBehaviour
    {

        public enum JellyType
        {
            Spring,
            Pressure,
        }

        public JellyType type;
        public int detail = 1;
        public List<Vector2> springs;
        public float SpringK = 1000;
        public float Damping = 10;
        public float GravityModifier = 1;
        public Material BaseMaterial;
        private Shape shape;
        private Body _body;
        private GameObject _prevShape;
        private Texture2D _spriteTexture;
        private Mesh _mesh;

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

            _spriteTexture = sprite.sprite.texture;
            sprite.enabled = false;

        }

        private void AddSpringBody(ClosedShape shape)
        {
            DraggableSpringBody body = gameObject.AddComponent<DraggableSpringBody>();
            body.Setup(JellyWorldManager.Instance.World, shape, 1f, SpringK, Damping, SpringK, Damping, transform.position, transform.rotation.eulerAngles.z, Vector2.one);
            body.Gravity = GravityModifier; 

            int i = springs.Count-1, l = -1;
            for(;i>l;i-=detail)
            {
                Vector2 v = springs[i];
                try{
                    //TODO: check if triangle is already added, if so, flip triangle.
                    body.addInternalSpring((int)v.x, (int)v.y,SpringK,Damping);
                }
                catch{
//                  Debug.Log("couldnt add spring at " +i);
                }
            }
            _body = body;
        }
        
        private void AddPressureBody(ClosedShape shape)
        {
            DraggablePressureBody body = gameObject.AddComponent<DraggablePressureBody>();
            body.Setup(JellyWorldManager.Instance.World, shape, 1.0f, 40.0f, 10.0f, 1.0f, 300.0f, 20.0f, transform.position, transform.rotation.eulerAngles.z, Vector2.one);
            body.Gravity = GravityModifier;

            body.addInternalSpring(0, 2, 400f, 12f);
            body.addInternalSpring(1, 3, 400f, 12f);
            _body = body;
        }

        void Update()
        {
            _body.Gravity = GravityModifier;
            DrawJellyShape();
        }

        private void DrawJellyShape()
        {   
            _body.mBaseShape.transformVertices(ref _body.mDerivedPos, _body.mDerivedAngle, ref _body.mScale, ref _body.mGlobalShape);
            
            List<Vector2> points = new List<Vector2>();
            for (int i = 0; i < _body.mPointMasses.Count; i++)
            {
                points.Add(VectorTools.vec3FromVec2(_body.mPointMasses [i].Position));
            }
            if (_prevShape == null)
            {
                Material m = new Material(BaseMaterial);
                m.mainTexture = _spriteTexture;



                MeshBuilder.Instance.material = m;
                MeshBuilder.Instance.BuildMesh2D(points, ShapeBuilt);
            } else
            {
                _prevShape.renderer.material.SetFloat("_RotationSpeed",_body.DerivedAngle);
                Vector2 bounds = _prevShape.GetComponent<Shape>().BoundingBox;
                bounds = _prevShape.transform.TransformPoint(bounds);
//                bounds += _prevShape.GetComponent<Shape>().BoundingBox;
                MeshBuilder.Instance.UpdateMeshPoints(_mesh, bounds,_prevShape.transform,points);
            }
        }

        private void ShapeBuilt(Shape shape)
        {
            _prevShape = shape.BuiltGameObject;
            _mesh = _prevShape.GetComponent<MeshFilter>().mesh;
            _prevShape.transform.parent = transform;
            _prevShape.name = "DisplayObject";
        }
    }
}
