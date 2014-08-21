﻿using UnityEditor;
using UnityEngine;
using System.Collections;
using Poly2Tri;
using System.Linq;
using System.Collections.Generic;

namespace JelloPhysics
{
    [CustomEditor (typeof(JellyMould))]
    public class JellyMouldInspector : UnityEditor.Editor
    {

        private JellyMould _target;
        private Texture _texture;
        private List<Vector2> _points;
        private Polygon _polygon;

        private GameObject _oldShape;

        public override void OnInspectorGUI()
        {
			_target = target as JellyMould;

			_target.SpringK = EditorGUILayout.FloatField ("SpringK:",_target.SpringK);
			_target.Damping = EditorGUILayout.FloatField ("Damping:",_target.Damping);

        }

        public void OnSceneGUI()
        {
            if (_target != null)
            {
                LoadOutlinePoints();
            }
        }

        private void LoadOutlinePoints()
        {
            _texture = _target.GetComponent<SpriteRenderer>().sprite.texture;
            if (_target.GetComponent<PolygonCollider2D>())
            {
                _points = _target.GetComponent<PolygonCollider2D>().points.ToList();
            } else
            {
                _points = _target.gameObject.AddComponent<PolygonCollider2D>().points.ToList();
            }

            _points.Add(_points [0]);

            DestroyImmediate(_oldShape);
            MeshBuilder.Instance.BuildMesh2D(_points, ShapeBuilt);
        }
            
        private void ShapeBuilt(Shape shape)
        {
            _oldShape = shape.BuiltGameObject;
            _oldShape.AddComponent<PolygonCollider2D>().points = _points.ToArray();
			Shape tShape = null;
			tShape = _target.gameObject.GetComponent<Shape> ();

			if (tShape == null) {
				tShape = _target.gameObject.AddComponent<Shape> ();
			}
			tShape.Area = shape.Area;
			tShape.BoundingBox = shape.BoundingBox;
			tShape.BuiltGameObject = shape.BuiltGameObject;
			tShape.Points = shape.Points;
			tShape.Polygon = shape.Polygon;
			tShape.UVBounds = shape.UVBounds;
            _target.springs = new List<Vector2>();

            foreach (DelaunayTriangle triangle in shape.Polygon.Triangles) {
                
                int i = 0, l = (_points.Count-2);
                List<int> indexes = new List<int>();
                foreach (TriangulationPoint tp in triangle.Points)
                {
                    int index = shape.Polygon.IndexOf(tp);
                    index -= 2;
                    if(index< 0)
                    {
                        int diff = l-(Mathf.Abs(index)-1);
                        index = diff;
                    }
                    indexes.Add(index);
                }
                _target.springs.Add(new Vector2(indexes[0],indexes[1]));
                _target.springs.Add(new Vector2(indexes[1], indexes[2]));
                _target.springs.Add(new Vector2(indexes[2], indexes[0]));
            }
			DestroyImmediate(_oldShape);

        }
    }
}